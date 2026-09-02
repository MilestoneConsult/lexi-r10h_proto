idf_component_register(
	SRCS 
	
		"main.c"
                    
	INCLUDE_DIRS 
	
		"."
		
	PRIV_REQUIRES
	
		lexi-r10h
		mp26053

		mci_app_event
		mci_gpio
		mci_ping
		
		esp_event
		esp_modem)
