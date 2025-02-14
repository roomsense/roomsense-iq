idf_component_register(
	SRCS "sht21.c"
	INCLUDE_DIRS "include" "../../main"
	REQUIRES esp_idf_lib_helpers ha_mqtt i2cdev
)
