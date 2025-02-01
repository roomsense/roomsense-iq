#include <esp_err.h>
#include <stdbool.h>
#include "esp_device.h"
#include "esp_serial.h"


typedef struct
{
    esp_serial_t *esp_serial;
    esp_device_t *esp_device;
    char *platform;
    char *schema;
    char *name;
    char *device_tag;
    char *unique_id;
    char *discovery_topic;
    char *state_topic;
    char *status_topic;
    char *unit_of_measurement;
} esp_discovery_sensor_t;

char *esp_discovery_serialize_sensor(esp_discovery_sensor_t *discovery);
esp_err_t esp_discovery_ensure_sensor(esp_discovery_sensor_t *discovery);
esp_err_t esp_discovery_init_sensor(esp_discovery_sensor_t *discovery);


typedef struct
{
    esp_serial_t *esp_serial;
    esp_device_t *esp_device;
    char *platform;
    char *schema;
    char *name;
    char *device_tag;
    char *unique_id;
    char *discovery_topic;
    char *state_topic;
    char *status_topic;
    char *set_topic;
    char *attributes_topic;
    char *unit_of_measurement;
    char *mode;
} esp_discovery_text_t;

char *esp_discovery_serialize_text(esp_discovery_text_t *discovery);
esp_err_t esp_discovery_ensure_text(esp_discovery_text_t *discovery);
esp_err_t esp_discovery_init_text(esp_discovery_text_t *discovery);


typedef struct
{
    esp_serial_t *esp_serial;
    esp_device_t *esp_device;
    char *platform;
    char *schema;
    char *name;
    char *device_tag;
    char *unique_id;
    char *discovery_topic;
    char *state_topic;
    char *status_topic;
} esp_discovery_pir_sensor_t;

char *esp_discovery_serialize_pir_sensor(esp_discovery_pir_sensor_t *discovery);
esp_err_t esp_discovery_init_pir_sensor(esp_discovery_pir_sensor_t *discovery);
esp_err_t esp_discovery_ensure_pir_sensor(esp_discovery_pir_sensor_t *discovery);



typedef struct
{
    esp_serial_t *esp_serial;
    esp_device_t *esp_device;
    char *platform;
    char *schema;
    char *name;
    char *device_tag;
    char *unique_id;
    char *discovery_topic;
    char *state_topic;
    char *status_topic;
    char *unit_of_measurement;
} esp_discovery_room_presence_t;

char *esp_discovery_serialize_room_presence(esp_discovery_room_presence_t *discovery);
esp_err_t esp_discovery_init_room_presence(esp_discovery_room_presence_t *discovery);
esp_err_t esp_discovery_ensure_room_presence(esp_discovery_room_presence_t *discovery);


typedef struct
{
    esp_serial_t *esp_serial;
    esp_device_t *esp_device;
    char *platform;
    char *schema;
    char *name;
    char *device_tag;
    char *unique_id;
    char *discovery_topic;
    char *state_topic;
    char *status_topic;
} esp_discovery_macro_move_sensor_t;

char *esp_discovery_serialize_macro_move_sensor(esp_discovery_macro_move_sensor_t *discovery);
esp_err_t esp_discovery_init_macro_move_sensor(esp_discovery_macro_move_sensor_t *discovery);
esp_err_t esp_discovery_ensure_macro_move_sensor(esp_discovery_macro_move_sensor_t *discovery);

typedef struct
{
    esp_serial_t *esp_serial;
    esp_device_t *esp_device;
    char *platform;
    char *schema;
    char *name;
    char *device_tag;
    char *unique_id;
    char *discovery_topic;
    char *state_topic;
    char *status_topic;
} esp_discovery_micro_move_sensor_t;

char *esp_discovery_serialize_micro_move_sensor(esp_discovery_micro_move_sensor_t *discovery);
esp_err_t esp_discovery_init_micro_move_sensor(esp_discovery_micro_move_sensor_t *discovery);
esp_err_t esp_discovery_ensure_micro_move_sensor(esp_discovery_micro_move_sensor_t *discovery);

typedef struct
{
    esp_serial_t *esp_serial;
    esp_device_t *esp_device;
    char *platform;
    char *schema;
    char *name;
    char *device_tag;
    char *unique_id;
    char *discovery_topic;
    char *state_topic;
    char *status_topic;
    char *unit_of_measurement;
} esp_discovery_light_sensor_t;

char *esp_discovery_serialize_light_sensor(esp_discovery_light_sensor_t *discovery);
esp_err_t esp_discovery_init_light_sensor(esp_discovery_light_sensor_t *discovery);
esp_err_t esp_discovery_ensure_light_sensor(esp_discovery_light_sensor_t *discovery);

typedef struct
{
    esp_serial_t *esp_serial;
    esp_device_t *esp_device;
    char *platform;
    char *schema;
    char *name;
    char *device_tag;
    char *unique_id;
    char *discovery_topic;
    char *state_topic;
    char *set_topic;
    char *status_topic;
} esp_discovery_button_t;

char *esp_discovery_serialize_button(esp_discovery_button_t *discovery);
esp_err_t esp_discovery_ensure_button(esp_discovery_button_t *discovery);
esp_err_t esp_discovery_init_button(esp_discovery_button_t *discovery);

typedef struct
{
    esp_serial_t *esp_serial;
    esp_device_t *esp_device;
    char *platform;
    char *schema;
    char *name;
    char *device_tag;
    char *unique_id;
    char *discovery_topic;
    char *state_topic;
    char *set_topic;
    char *status_topic;
} esp_discovery_switch_t;

char *esp_discovery_serialize_switch(esp_discovery_switch_t *discovery);
esp_err_t esp_discovery_ensure_switch(esp_discovery_switch_t *discovery);
esp_err_t esp_discovery_init_switch(esp_discovery_switch_t *discovery);


typedef struct
{
    esp_serial_t *esp_serial;
    esp_device_t *esp_device;
    char *platform;
    char *schema;
    char *name;
    char *device_tag;
    char *unique_id;
    char *discovery_topic;
    char *state_topic;
    char *set_topic;
    char *status_topic;
    char *attributes_topic;
    char *mode;
    float min;
	float max;
	float step;
} esp_discovery_number_t;


char *esp_discovery_serialize_number(esp_discovery_number_t *discovery);

esp_err_t esp_discovery_ensure_number(esp_discovery_number_t *discovery);

esp_err_t esp_discovery_init_number(esp_discovery_number_t *discovery);
    
