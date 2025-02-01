#include <esp_err.h>

typedef struct {
    char *manufacturer;
    char *model;
    char *sw_version;
    char *idf_version;
} esp_device_t;

esp_err_t esp_device_init(esp_device_t *device);
