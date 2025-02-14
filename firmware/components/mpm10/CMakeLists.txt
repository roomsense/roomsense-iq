idf_component_register(
	SRCS "mpm10.c"
	INCLUDE_DIRS "include" "../../main"
	REQUIRES esp_idf_lib_helpers i2cdev
)
