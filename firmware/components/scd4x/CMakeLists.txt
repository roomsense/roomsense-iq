idf_component_register(
	SRCS "scd4x.c"
	INCLUDE_DIRS "include" "../../main"
	REQUIRES esp_rom i2cdev esp_idf_lib_helpers
)
