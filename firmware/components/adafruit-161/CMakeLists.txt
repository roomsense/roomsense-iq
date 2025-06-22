idf_component_register(
	SRCS "adafruit-161.c"
	INCLUDE_DIRS "include" "../../main"
	REQUIRES esp_adc_cal sht30
)
