idf_component_register(
	SRCS "sgp40.c" "sensirion_voc_algorithm.c"
	INCLUDE_DIRS "include" "../../main"
	REQUIRES esp_idf_lib_helpers i2cdev
)

