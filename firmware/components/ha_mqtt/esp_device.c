#include "esp_system.h"
#include "esp_device.h"

#define DEVICE_MANUFACTURER "RoomSense Labs"
#define DEVICE_MODEL "IQ"

esp_err_t esp_device_init(esp_device_t *device) {
    device->manufacturer = DEVICE_MANUFACTURER;
    device->model = DEVICE_MODEL;
    device->idf_version = (char *) esp_get_idf_version();

    return ESP_OK;
}
