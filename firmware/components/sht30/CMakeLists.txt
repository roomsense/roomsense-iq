idf_component_register(
	SRCS "sht30.c"
	INCLUDE_DIRS "include" "../../main"
	REQUIRES esp_idf_lib_helpers i2cdev ha_mqtt
)
