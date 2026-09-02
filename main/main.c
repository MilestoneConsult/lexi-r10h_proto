#include "freertos/FreeRTOS.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_modem_api.h"
#include "esp_modem_config.h"
#include "esp_netif.h"

#include "driver/gpio.h"
#include <assert.h>
#include <netdb.h>
#include "app_event.h"
#include "charger.h"
#include "i2c.h"
#include "lexi-r10h.h"
#include "mcio.h"
#include "ping.h"
#include "tca6408a.h"

#define MODEM_POWER_ENABLED 1
#define MODEM_POWER_DISABLED 0

#define MODEM_PERIPHERAL_POWER_ENABLED 1
#define MODEM_PERIPHERAL_POWER_DISABLED 0

#define MODEM_RESET_RELEASED 0
#define MODEM_RESET_HELD 1

#define MODEM_AWAKE 1
#define MODEM_ASLEEP 0

typedef enum
{
	MCIO_PIN_NOT_AVAILABLE = GPIO_NUM_NC,

	// --- Internal ---

	MCIO_BUTTON_UP = GPIO_NUM_4,	 // [BUT_UP] Modem Reset
	MCIO_BUTTON_SELECT = GPIO_NUM_5, // [BUT_SEL] Modem Reset
	MCIO_BUTTON_DOWN = GPIO_NUM_6,	 // [BUT_DN] Modem Reset

	MCIO_BATTERY_VOLTAGE = GPIO_NUM_7, // [BAT_LVL] Battery Voltage (ADC1 Channel 6)

	MCIO_HEALTH_SENSOR_ENABLE = GPIO_NUM_3, // [AS7058_EN] Health Sensor Enable
	MCIO_HEALTH_SENSOR_IRQ = GPIO_NUM_8,	// [AS7058_INT] Health Sensor Interrupt

	MCIO_BATTERY_CHARGER_CONNECTED = GPIO_NUM_9,				   // [nCHRGR_CONNECTED] Battery Charger Connected (active low)
	MCIO_BATTERY_CHARGER_ENABLE = MCIO_PIN_TYPE_TCA6408A_0 | 0x06, // [LED_BLUE] Blue LED

	MCIO_STRAP_IRQ = GPIO_NUM_14, // [nSTRAP_INT] Strap Interrupt (active low)
	MCIO_CLASP_IRQ = GPIO_NUM_21, // [nSTRAP_CLASP] Strap clasp detection (active low)

	MCIO_GNSS_POWER_ENABLE = MCIO_PIN_TYPE_TCA6408A_0 | 0x00, // [GNSS_EN] GNSS Enable / Disable
	MCIO_GNSS_RESET = MCIO_PIN_TYPE_TCA6408A_0 | 0x01,		  // [GNSS_RST] GNSS Reset
	MCIO_GNSS_PM = MCIO_PIN_TYPE_TCA6408A_0 | 0x07,			  // [GNSS_EXTINT] GNSS Interrupt
	// MCIO_GNSS_TIMEPULSE = GPIO_NUM_38,								// [GNSS_TIMEPULSE] GNSS Time Pulse Output

	MCIO_HAPTIC_ENABLE = GPIO_NUM_42, // [HAPTIC_EN] Haptic Enable / Disable

	MCIO_TDAM_ENABLE = GPIO_NUM_47, // [TDAM_EN] TDAM Enable / Disable

	MCIO_LCD_ENABLE = GPIO_NUM_41, // [DISP_EN] LCD Enable
	MCIO_LCD_RESET = GPIO_NUM_46,  // [DISP_nRESET] LCD Reset (active low)
	MCIO_LCD_SPI_CS = GPIO_NUM_10,
	MCIO_LCD_SELECT_CMD_DATA = GPIO_NUM_13,	 // [DISP_RS???] LCD Seclect Command / Data Mode
	MCIO_LCD_BACKLIGHT_PWM = GPIO_NUM_39,	 // [DISP_BK_LGT_PWM] LCD Backlight PWM (brightness)
	MCIO_LCD_BACKLIGHT_ENABLE = GPIO_NUM_40, // [DISP_BK_LGT_EN] LCD Backlight Enable

	MCIO_LED_RED = MCIO_PIN_NOT_AVAILABLE,			  // [LED_RED] Red LED --- IGNORE ---
	MCIO_LED_GREEN = MCIO_PIN_TYPE_TCA6408A_0 | 0x05, // [LED_GREEN] Green LED
	MCIO_LED_BLUE = MCIO_PIN_NOT_AVAILABLE,			  // [LED_BLUE] Blue LED --- IGNORE ---

	MCIO_MODEM_POWER_KEY = MCIO_PIN_TYPE_TCA6408A_0 | 0x03,				  // [ESP_MDM_PWRKEY] Modem Power Key [see datasheet pg 27 for timing requirements]
	MCIO_MODEM_PERIPHERAL_POWER_ENABLE = MCIO_PIN_TYPE_TCA6408A_0 | 0x04, // [MDM_PWR_EN] Modem Power Enable
	// MCIO_MODEM_WAKE = GPIO_NUM_46,            							// [ESP_MDM_WAKE] Modem Wake
	// MCIO_MODEM_POWER_MONITOR = GPIO_NUM_2,    							// [MDM_PWR_MDM] Modem Internal Power Monitor [when low modem is in deep sleep]
	MCIO_MODEM_RESET = MCIO_PIN_TYPE_TCA6408A_0 | 0x02, // [ESP_MDM_RESET] Modem Reset Pin
	MCIO_MODEM_RING = GPIO_NUM_48,						// [DISP_TE???] Modem +URC Ring Indicator (active high)
	MCIO_MODEM_FAST_OFF = GPIO_NUM_45,					// [ESP_MDM_FAST_OFF] Modem Fast Off (active high)

} mcio_pin_t;


// ##############################
// ##############################
// ##############################

static esp_err_t modem_urc_receiver(uint8_t *data, size_t len)
{
	while (len > 0 && (data[0] == '\r' || data[0] == '\n'))
	{
		data++;
		len--;
	}
	while (len > 0 && (data[len - 1] == '\r' || data[len - 1] == '\n'))
	{
		len--;
	}

	// ESP_LOGW(" URC", "");
	ESP_LOGW(" URC", "~~~~~~~~~~ %.*s", (int)len, (char *)data);
	// ESP_LOGW(" URC", "");

	return ESP_OK;
}

void app_main(void)
{
	// esp_log_level_set("command_lib", ESP_LOG_DEBUG);
	// esp_log_level_set("uart_terminal", ESP_LOG_DEBUG);
	// esp_log_level_set("uart-tx", ESP_LOG_DEBUG);
	// esp_log_level_set("uart-rx", ESP_LOG_DEBUG);

	// -----

	esp_netif_init();

	esp_event_loop_create_default();
	app_event_loop_create_default(4 * 1024);

	i2c_initialize(I2C_NUM_0, GPIO_NUM_1, GPIO_NUM_0);

	tca6408a_initialize(I2C_NUM_0); // IO Expander
	mcio_initialize();


	// ********** Configure Expander IO **********

	uint8_t output_levels_tca6408a =						 // ***** Expander Output Levels *****
		0x00 |												 // ## All Output Off (before changing direction) ##
		(1 << (MCIO_GNSS_POWER_ENABLE & 0xFF)) |			 // GNSS Power (high => off)
		(0 << (MCIO_GNSS_RESET & 0xFF)) |					 // GNSS Reset (low => held in reset)
		(1 << (MCIO_MODEM_RESET & 0xFF)) |					 // Modem Reset (high / held in reset)
		(0 << (MCIO_MODEM_PERIPHERAL_POWER_ENABLE & 0xFF)) | // Modem Peripheral Power (low => off)
		(0 << (MCIO_MODEM_POWER_KEY & 0xFF)) |				 // LED Green Off (high / off)
		(1 << (MCIO_LED_GREEN & 0xFF)) |					 // LED Green Off (high / off)
		(1 << (MCIO_BATTERY_CHARGER_ENABLE & 0xFF)) |		 // Battery Charger Enable (high / off)
		(0 << (MCIO_GNSS_PM & 0xFF));						 // GNSS Power Management (low => asleep)

	tca6408a_set_output_levels(
		TCA6408A_I2C_ADDRESS_0,
		output_levels_tca6408a);

	// -----

	uint8_t pin_config_tca6408a_0 =							  // ***** Expander Pin Directions *****
		0xFF &												  // ## All Input (handles unused) ##
		~(1 << (MCIO_GNSS_POWER_ENABLE & 0xFF)) &			  // [port 0] Output
		~(1 << (MCIO_GNSS_RESET & 0xFF)) &					  // [port 1] Output
		~(1 << (MCIO_MODEM_RESET & 0xFF)) &					  // [port 2] Output
		~(1 << (MCIO_MODEM_PERIPHERAL_POWER_ENABLE & 0xFF)) & // [port 3] Output
		~(1 << (MCIO_MODEM_POWER_KEY & 0xFF)) &				  // [port 4] Output
		~(1 << (MCIO_LED_GREEN & 0xFF)) &					  // [port 5] Output
		~(1 << (MCIO_BATTERY_CHARGER_ENABLE & 0xFF)) &		  // [port 6] Output
		~(1 << (MCIO_GNSS_PM & 0xFF));						  // [port 7] Output

	tca6408a_set_pin_config(
		TCA6408A_I2C_ADDRESS_0,
		pin_config_tca6408a_0);

	// -----

	charger_initialize(MCIO_BATTERY_CHARGER_ENABLE, MCIO_BATTERY_CHARGER_CONNECTED);
	charger_power_on_setup();

	// -----

	lexi_initialize(
		UART_NUM_1,
		115200, 115200,
		GPIO_NUM_18, GPIO_NUM_17, GPIO_NUM_20, GPIO_NUM_19,
		1024, 1024,
		MCIO_MODEM_RESET, MCIO_MODEM_POWER_KEY, MCIO_MODEM_PERIPHERAL_POWER_ENABLE);
	lexi_power_on_setup();
	
	printf("Sleeping 15 sec After Power On Setup\n");
	vTaskDelay(15 * 1000 / portTICK_PERIOD_MS);

	while(true)
	{
		lexi_connect(60 * 1000);
		printf("Sleeping 30 sec After Connect\n");
		vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);
		
		lexi_disconnect(30 * 1000);
		printf("Sleeping 30 sec After Disconnect\n");
		vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);
	}

	return;

	/* Configure and create the DTE */
	esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();

	dte_config.uart_config.tx_io_num = GPIO_NUM_18;
	dte_config.uart_config.rx_io_num = GPIO_NUM_17;
	dte_config.uart_config.rts_io_num = GPIO_NUM_20;
	dte_config.uart_config.cts_io_num = GPIO_NUM_19;
	dte_config.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_HW;

	/* esp_modem_new_dev() is the smallest public entry point that creates the DTE (no standalone DTE-only C API exists) */
	esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG("super");
	esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
	esp_netif_t *netif = esp_netif_new(&netif_ppp_config);
	assert(netif);

	esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_GENERIC, &dte_config, &dce_config, netif);
	assert(dce);

	ESP_ERROR_CHECK(esp_modem_set_urc(dce, modem_urc_receiver));


	/* Power Up Modem */

	mcio_set_output_level((gpio_num_t)MCIO_MODEM_PERIPHERAL_POWER_ENABLE, MODEM_PERIPHERAL_POWER_ENABLED); // Modem Peripheral Power
	vTaskDelay(30 / portTICK_PERIOD_MS);

	mcio_set_output_level((gpio_num_t)MCIO_MODEM_RESET, MODEM_RESET_HELD); // Reset Modem
	vTaskDelay(20 / portTICK_PERIOD_MS);
	mcio_set_output_level((gpio_num_t)MCIO_MODEM_RESET, MODEM_RESET_RELEASED);

	mcio_set_output_level((gpio_num_t)MCIO_MODEM_POWER_KEY, 1); // Power On Modem (active low)
	vTaskDelay(50 / portTICK_PERIOD_MS);
	mcio_set_output_level((gpio_num_t)MCIO_MODEM_POWER_KEY, 0);

	// -----

	esp_err_t ret;

	vTaskDelay(2000 / portTICK_PERIOD_MS);


	// char response[ESP_MODEM_C_API_STR_BUF_SIZE];
	// esp_err_t ret = esp_modem_at(dce, "ATE0", response, 500);
	// ESP_LOGW("main", "ATE0: %s, response: %s", esp_err_to_name(ret), response);


	// ret = esp_modem_sync(dce);
	// ret = esp_modem_sync(dce);
	// ret = esp_modem_sync(dce);
	// ESP_LOGI("main", "esp_modem_sync: %s", esp_err_to_name(ret));

	ret = esp_modem_set_mode(dce, ESP_MODEM_MODE_CMUX_MANUAL);
	ESP_LOGW("main", "esp_modem_set_mode(CMUX): %s", esp_err_to_name(ret));

	char response[ESP_MODEM_C_API_STR_BUF_SIZE];
	ret = esp_modem_at(dce, "AT+CEREG=2", response, 500);
	ESP_LOGW("main", "AT+CEREG=2 response: %s", response);
	ESP_LOGW("main", "AT+CEREG=2 ret: %s", esp_err_to_name(ret));

	/* Wait for network registration (home or roaming) before starting data mode */
	int registration_state = -1;
	do
	{
		ret = esp_modem_get_network_registration_state(dce, &registration_state);
		ESP_LOGW("main", "Registration state: %d (%s)", registration_state, esp_err_to_name(ret));
		if (registration_state == 1 || registration_state == 5)
		{
			break;
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	} while (true);

	ret = esp_modem_set_mode(dce, ESP_MODEM_MODE_CMUX_MANUAL_DATA);
	ESP_LOGW("main", "esp_modem_set_mode(CMUX_MANUAL_DATA): %s", esp_err_to_name(ret));

	/* Wait for the PPP interface to obtain an IP address */
	esp_netif_ip_info_t ip_info = { 0 };
	do
	{
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		esp_netif_get_ip_info(netif, &ip_info);
	} while (ip_info.ip.addr == 0);
	ESP_LOGW("main", "PPP got IP: " IPSTR, IP2STR(&ip_info.ip));

	/* Resolve google.com once, then ping it once per second */
	struct addrinfo *dns_result = NULL;
	esp_ip4_addr_t ping_target = { 0 };
	if (getaddrinfo("google.com", NULL, NULL, &dns_result) == 0 && dns_result != NULL)
	{
		struct sockaddr_in *addr = (struct sockaddr_in *)dns_result->ai_addr;
		ping_target.addr = addr->sin_addr.s_addr;
		freeaddrinfo(dns_result);
		ESP_LOGW("main", "google.com resolved to: " IPSTR, IP2STR(&ping_target));
	}

	while (true)
	{
		ping_ip_blocking(ping_target, 1, 1000);
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}
