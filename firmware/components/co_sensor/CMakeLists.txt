idf_component_register(
	SRCS "co_sensor.c"
	INCLUDE_DIRS "include" "../../main"
	REQUIRES esp_adc_cal sht30
)
