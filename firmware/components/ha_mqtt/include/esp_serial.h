#include <esp_err.h>

typedef struct {
    char *namespace;
    char *key;
    uint8_t *data;
    uint8_t length;
} esp_serial_t;

esp_err_t get_mac_address(esp_serial_t *esp_serial);

void esp_serial_hex(esp_serial_t *esp_serial, char **hex);
