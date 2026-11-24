#include <stdio.h>
#include <string.h>
#include "cJSON.h"
#include "esp_discovery.h"
#include "esp_log.h"
#include "tasks_common.h"

#define LAST_FOUR_CHARS_LENGTH 7

// Home Assistant shows the entity name straight from the discovery payload, and the
// firmware used to publish "<location>-<device_tag>-<mac4>" ("Foyer-PM2-5-0580"). That is
// noise now that each unit is its own device, so publish a plain label instead. Only the
// display name changes: unique_id and the topics keep their old values, so the entities
// Home Assistant has already registered keep their entity IDs.
static const char *esp_discovery_friendly_name(const char *device_tag)
{
	static char generated[40];
	static const struct { const char *tag; const char *label; } labels[] = {
			{ "Temperature-C", "Temperature (\xc2\xb0" "C)" },
			{ "Temperature-F", "Temperature (\xc2\xb0" "F)" },
			{ "Dew-Point", "Dew Point" },
			{ "Humidity", "Humidity" },
			{ "CO2", "CO2" },
			{ "CO", "CO" },
			{ "TVOC", "TVOC" },
			{ "PM1", "PM1" },
			{ "PM2-5", "PM2.5" },
			{ "PM10", "PM10" },
			{ "Light", "Light" },
			{ "PIR", "PIR Motion" },
			{ "Presence", "Presence" },
			{ "Movement-Direction", "Movement Direction" },
			{ "Distance-cm", "Distance (cm)" },
			{ "Distance-ft", "Distance (ft)" },
			{ "Location", "Location" },
			{ "mold-risk", "Mold Risk" },
			{ "co2-status", "CO2 Status" },
			{ "co-status", "CO Status" },
			{ "pm-status", "PM Status" },
			{ "voc-status", "VOC Status" },
			{ "humidity-status", "Humidity Status" },
			{ "start_stop_status", "Start/Stop Status" },
			{ "roomsense-start", "Start" },
			{ "roomsense-read", "Read Config" },
			{ "roomsense-config", "Write Config" },
			{ "roomsense-reset", "Reset" },
			{ "roomsense-bedsense", "Bed Sense" },
			{ "roomsense-calsense", "Calibrate Sensor" },
			{ "roomsense-pirsensitivity", "PIR Sensitivity" },
			{ "roomsense-maxmacrorange", "Max Macro Range" },
			{ "roomsense-maxmicrorange", "Max Micro Range" },
			{ "roomsense-timeout", "Presence Timeout" },
	};
	const char *prefix;
	const char *suffix;
	size_t i;

	for (i = 0; i < sizeof(labels) / sizeof(labels[0]); i++)
	{
		if (!strcmp(device_tag, labels[i].tag))
		{
			return labels[i].label;
		}
	}

	// The per-gate tuning entities are numbered 0-8, so name them from the tag itself
	if (!strncmp(device_tag, "roomsense-macro", 15) || !strncmp(device_tag, "roomsense-micro", 15))
	{
		prefix = device_tag[9] == 'm' ? "Macro" : "Micro";
		suffix = strstr(device_tag, "threshold") ? "Threshold" : "Bin";
		snprintf(generated, sizeof(generated), "%s %s %d", prefix, suffix,
				device_tag[strlen(device_tag) - 1] - '0' + 1);
		return generated;
	}

	// Anything unmapped: drop the vendor prefix and turn separators into spaces
	prefix = !strncmp(device_tag, "roomsense-", 10) ? device_tag + 10 : device_tag;
	snprintf(generated, sizeof(generated), "%s", prefix);
	for (i = 0; generated[i] != '\0'; i++)
	{
		if (generated[i] == '-' || generated[i] == '_')
		{
			generated[i] = ' ';
		}
	}
	return generated;
}


//Discovery prefix should be identical to HA MQTT integration option
static const char *DISCOVERY_TAG_LIGHT = "homeassistant/light";
static const char *DISCOVERY_TAG_SENSOR = "homeassistant/sensor";
static const char *DISCOVERY_TAG_PIR_SENSOR = "homeassistant/binary_sensor";
static const char *DISCOVERY_TAG_ROOM_PRESENCE = "homeassistant/sensor";
static const char *DISCOVERY_TAG_BUTTON = "homeassistant/button";
static const char *DISCOVERY_TAG_SWITCH = "homeassistant/switch";
static const char *DISCOVERY_TAG_NUMBER = "homeassistant/number";
static const char *DISCOVERY_TAG_PHOTO_CELL = "homeassistant/photocell";
static const char *DISCOVERY_TAG_MOVEMENT_DIRECTION = "homeassistant/text";

static const char *DISCOVERY_PATH = "config";
static const char *SET_PATH = "set";
static const char *STATE_PATH = "state";
static const char *STATUS_PATH = "status";
static const char *ATTRS_PATH = "attributes";

esp_err_t esp_discovery_init_number(esp_discovery_number_t *discovery)
{
	esp_discovery_ensure_number(discovery);
	return ESP_OK;
}

esp_err_t esp_discovery_ensure_number(esp_discovery_number_t *discovery)
{
	//char *hex;

	if (!discovery->esp_serial)
	{
		return ESP_FAIL;
	}

	//reads the mac address to create a unique ID
	//esp_serial_hex(discovery->esp_serial, &hex);
	discovery->name = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(discovery->device_tag) + 2 + 1);
	discovery->unique_id = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(discovery->device_tag) + 3 + 1);
	discovery->discovery_topic = malloc(
			strlen(DISCOVERY_TAG_NUMBER) + strlen(discovery->device_tag) + strlen(roomsense_iq_shared.mac_address) + strlen(DISCOVERY_PATH) + 4 + 1);
	discovery->state_topic = malloc(strlen(discovery->device_tag) + strlen(roomsense_iq_shared.mac_address) + strlen(STATE_PATH) + 4 + 1);
	discovery->set_topic = malloc(strlen(discovery->device_tag) + strlen(roomsense_iq_shared.mac_address) + strlen(SET_PATH) + 4 + 1);
	discovery->status_topic = malloc(strlen(discovery->device_tag) + strlen(roomsense_iq_shared.mac_address) + strlen(STATUS_PATH) + 4 + 1);
	discovery->attributes_topic = malloc(strlen(discovery->device_tag) + strlen(roomsense_iq_shared.mac_address) + strlen(ATTRS_PATH) + 4 + 1);

	sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
	sprintf(discovery->unique_id, "%s_%s", discovery->device_tag, roomsense_iq_shared.mac_address);
	sprintf(discovery->discovery_topic, "%s/%s i-%s/%s", DISCOVERY_TAG_NUMBER, discovery->device_tag, roomsense_iq_shared.mac_address, DISCOVERY_PATH);
	sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, roomsense_iq_shared.mac_address, STATE_PATH);
	sprintf(discovery->set_topic, "%s/%s/%s", discovery->device_tag, roomsense_iq_shared.mac_address, SET_PATH);
	sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, roomsense_iq_shared.mac_address, STATUS_PATH);
	sprintf(discovery->attributes_topic, "%s/%s/%s", discovery->device_tag, roomsense_iq_shared.mac_address, ATTRS_PATH);

	//free(hex);
	return ESP_OK;
}

// Every entity a unit publishes carries this device identity. A constant "RoomSense"
// identifier/name made Home Assistant fold the entities of every sensor in the house into
// a single device, which then has to be split up by hand, so key it on the MAC address.
static char esp_discovery_device_identifier[64] = {0};
static char esp_discovery_device_name[96] = {0};

static void esp_discovery_device_identity_init(void)
{
	if (esp_discovery_device_identifier[0] != '\0')
	{
		return;
	}

	// Bounded so the compiler can prove these fit (location is up to 32 chars per the
	// firmware's own limit; it lives in a 100 byte field)
	snprintf(esp_discovery_device_identifier, sizeof(esp_discovery_device_identifier),
			"roomsense_%s", roomsense_iq_shared.mac_address);

	if (roomsense_iq_shared.location[0] != '\0')
	{
		snprintf(esp_discovery_device_name, sizeof(esp_discovery_device_name),
				"RoomSense IQ %.32s (%.12s)", roomsense_iq_shared.location, roomsense_iq_shared.mac_address);
	}
	else
	{
		snprintf(esp_discovery_device_name, sizeof(esp_discovery_device_name),
				"RoomSense IQ %.12s", roomsense_iq_shared.mac_address);
	}
}

char* esp_discovery_serialize_number(esp_discovery_number_t *discovery)
{
	char *json = NULL;
	cJSON *root = cJSON_CreateObject();

	cJSON *device = cJSON_CreateObject();
	cJSON_AddItemToObject(root, "device", device);
	esp_discovery_device_identity_init();
	cJSON *identifiers = cJSON_CreateStringArray((const char*[]
			)
			{ esp_discovery_device_identifier }, 1);
	cJSON_AddItemToObject(device, "identifiers", identifiers);
	cJSON_AddStringToObject(device, "name", esp_discovery_device_name);

	cJSON_AddStringToObject(root, "platform", discovery->platform);
	cJSON_AddStringToObject(root, "schema", discovery->schema);
	cJSON_AddStringToObject(root, "name", discovery->name);
	cJSON_AddStringToObject(root, "unique_id", discovery->unique_id);

	cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
	cJSON_AddStringToObject(root, "command_topic", discovery->set_topic);
	cJSON_AddStringToObject(root, "availability_topic", discovery->status_topic);
	cJSON_AddStringToObject(root, "json_attributes_topic", discovery->attributes_topic);

	cJSON_AddNumberToObject(root, "min", discovery->min);
	cJSON_AddNumberToObject(root, "max", discovery->max);
	cJSON_AddNumberToObject(root, "step", discovery->step);
	cJSON_AddStringToObject(root, "mode", discovery->mode);

	json = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);
	return json;
}

esp_err_t esp_discovery_init_button(esp_discovery_button_t *discovery)
{
	esp_discovery_ensure_button(discovery);
	return ESP_OK;
}

esp_err_t esp_discovery_ensure_button(esp_discovery_button_t *discovery)
{
	char *hex;

	if (!discovery->esp_serial)
	{
		return ESP_FAIL;
	}

	//reads the mac address to create a unique ID
	esp_serial_hex(discovery->esp_serial, &hex);
	discovery->name = malloc(strlen(hex) + strlen(discovery->device_tag) + 2 + 1);
	discovery->unique_id = malloc(strlen(hex) + strlen(discovery->device_tag) + 3 + 1);
	discovery->discovery_topic = malloc(strlen(DISCOVERY_TAG_BUTTON) + strlen(discovery->device_tag) + strlen(hex) + strlen(DISCOVERY_PATH) + 4 + 1);
	discovery->state_topic = malloc(strlen(discovery->device_tag) + strlen(hex) + strlen(STATE_PATH) + 4 + 1);
	discovery->set_topic = malloc(strlen(discovery->device_tag) + strlen(hex) + strlen(SET_PATH) + 4 + 1);
	discovery->status_topic = malloc(strlen(discovery->device_tag) + strlen(hex) + strlen(STATUS_PATH) + 4 + 1);

	sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
	sprintf(discovery->unique_id, "%s_%s", discovery->device_tag, hex);
	sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_BUTTON, discovery->device_tag, hex, DISCOVERY_PATH);
	//sprintf(discovery->discovery_topic, "%s/B-%s/%s", DISCOVERY_TAG_BUTTON, hex, DISCOVERY_PATH);
	sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, hex, STATE_PATH);
	sprintf(discovery->set_topic, "%s/%s/%s", discovery->device_tag, hex, SET_PATH);
	sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, hex, STATUS_PATH);

	free(hex);
	return ESP_OK;
}

char* esp_discovery_serialize_button(esp_discovery_button_t *discovery)
{
	char *json = NULL;
	cJSON *root = cJSON_CreateObject();

	cJSON *device = cJSON_CreateObject();
	cJSON_AddItemToObject(root, "device", device);
	esp_discovery_device_identity_init();
	cJSON *identifiers = cJSON_CreateStringArray((const char*[]
			)
			{ esp_discovery_device_identifier }, 1);
	cJSON_AddItemToObject(device, "identifiers", identifiers);
	cJSON_AddStringToObject(device, "name", esp_discovery_device_name);

	cJSON_AddStringToObject(root, "platform", discovery->platform);
	cJSON_AddStringToObject(root, "schema", discovery->schema);
	cJSON_AddStringToObject(root, "name", discovery->name);
	cJSON_AddStringToObject(root, "unique_id", discovery->unique_id);
	cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
	cJSON_AddStringToObject(root, "command_topic", discovery->set_topic);
	cJSON_AddStringToObject(root, "availability_topic", discovery->status_topic);

	json = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);
	return json;
}
//////////////////////////////////////////////////////////////////////////////////////////
esp_err_t esp_discovery_init_switch(esp_discovery_switch_t *discovery)
{
	esp_discovery_ensure_switch(discovery);
	return ESP_OK;
}

esp_err_t esp_discovery_ensure_switch(esp_discovery_switch_t *discovery)
{
	char *hex;

	if (!discovery->esp_serial)
	{
		return ESP_FAIL;
	}

	//reads the mac address to create a unique ID
	esp_serial_hex(discovery->esp_serial, &hex);
	discovery->name = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(discovery->device_tag) + 2 + 1);
	discovery->unique_id = malloc(strlen(hex) + strlen(discovery->device_tag) + 3 + 1);
	discovery->discovery_topic = malloc(strlen(DISCOVERY_TAG_SWITCH) + strlen(discovery->device_tag) + strlen(hex) + strlen(DISCOVERY_PATH) + 4 + 1);
	discovery->state_topic = malloc(strlen(discovery->device_tag) + strlen(hex) + strlen(STATE_PATH) + 4 + 1);
	discovery->set_topic = malloc(strlen(discovery->device_tag) + strlen(hex) + strlen(SET_PATH) + 4 + 1);
	discovery->status_topic = malloc(strlen(discovery->device_tag) + strlen(hex) + strlen(STATUS_PATH) + 4 + 1);

	sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
	sprintf(discovery->unique_id, "%s_%s", discovery->device_tag, hex);
	sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_SWITCH, discovery->device_tag, hex, DISCOVERY_PATH);
	sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, hex, STATE_PATH);
	sprintf(discovery->set_topic, "%s/%s/%s", discovery->device_tag, hex, SET_PATH);
	sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, hex, STATUS_PATH);

	free(hex);
	return ESP_OK;
}

char* esp_discovery_serialize_switch(esp_discovery_switch_t *discovery)
{
	char *json = NULL;
	cJSON *root = cJSON_CreateObject();

	cJSON *device = cJSON_CreateObject();
	cJSON_AddItemToObject(root, "device", device);
	esp_discovery_device_identity_init();
	cJSON *identifiers = cJSON_CreateStringArray((const char*[]
			)
			{ esp_discovery_device_identifier }, 1);
	cJSON_AddItemToObject(device, "identifiers", identifiers);
	cJSON_AddStringToObject(device, "name", esp_discovery_device_name);

	cJSON_AddStringToObject(root, "platform", discovery->platform);
	cJSON_AddStringToObject(root, "schema", discovery->schema);
	cJSON_AddStringToObject(root, "name", discovery->name);
	cJSON_AddStringToObject(root, "unique_id", discovery->unique_id);
	cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
	cJSON_AddStringToObject(root, "command_topic", discovery->set_topic);
	cJSON_AddStringToObject(root, "availability_topic", discovery->status_topic);

	json = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);
	return json;
}
//////////////////////////////////////////////////////////////////////////////////////////

char* esp_discovery_serialize_room_presence(esp_discovery_room_presence_t *discovery)
{
	char *json = NULL;
	cJSON *root = cJSON_CreateObject();

	cJSON *device = cJSON_CreateObject();
	cJSON_AddItemToObject(root, "device", device);
	esp_discovery_device_identity_init();
	cJSON *identifiers = cJSON_CreateStringArray((const char*[]
			)
			{ esp_discovery_device_identifier }, 1);
	cJSON_AddItemToObject(device, "identifiers", identifiers);
	cJSON_AddStringToObject(device, "name", esp_discovery_device_name);

	cJSON_AddStringToObject(root, "platform", discovery->platform);
	cJSON_AddStringToObject(root, "schema", discovery->schema);
	cJSON_AddStringToObject(root, "name", discovery->name);
	cJSON_AddStringToObject(root, "unique_id", discovery->unique_id);
	cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
	cJSON_AddStringToObject(root, "availability_topic", discovery->status_topic);

	json = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);
	return json;
}

esp_err_t esp_discovery_init_room_presence(esp_discovery_room_presence_t *discovery)
{
	esp_discovery_ensure_room_presence(discovery);
	return ESP_OK;
}

esp_err_t esp_discovery_ensure_room_presence(esp_discovery_room_presence_t *discovery)
{
	char last_four_chars[LAST_FOUR_CHARS_LENGTH + 1]; // Plus one for null terminator

    if (!discovery->esp_serial)
    {
        return ESP_FAIL;
    }

    printf("room_presence");
	    //esp_serial_hex(discovery->esp_serial, &hex);
    discovery->name = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + 2 + 1);
    discovery->unique_id = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + 3 + 1);
    discovery->discovery_topic = malloc(strlen(DISCOVERY_TAG_ROOM_PRESENCE) + strlen(roomsense_iq_shared.location) +  strlen(discovery->device_tag) + strlen(roomsense_iq_shared.mac_address) + strlen(DISCOVERY_PATH) + 4 + 1);
    discovery->state_topic = malloc(strlen(discovery->device_tag) + strlen(roomsense_iq_shared.location) +  strlen(roomsense_iq_shared.mac_address) + strlen(STATE_PATH) + 4 + 1);
    discovery->status_topic = malloc(strlen(discovery->device_tag) + strlen(roomsense_iq_shared.location) +  strlen(roomsense_iq_shared.mac_address) + strlen(STATUS_PATH) + 4 + 1);
    discovery->unit_of_measurement = malloc(2 + 1);

   // Copy the last 4 bytes of the MAC address
    strcpy(last_four_chars, roomsense_iq_shared.mac_address + (MAC_ADDRESS_LENGTH - LAST_FOUR_CHARS_LENGTH));

    if (strlen(roomsense_iq_shared.location) == 0)
    {
        sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
        sprintf(discovery->unique_id, "%s_%s", discovery->device_tag, last_four_chars);
        sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_ROOM_PRESENCE, discovery->device_tag, last_four_chars, DISCOVERY_PATH);
        sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATE_PATH);
        sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATUS_PATH);
//	if (!strcmp(discovery->device_tag, "Distance-cm"))
//	{
//		printf("1----------------> unit cm");
//		sprintf(discovery->unit_of_measurement, "cm");
//	}
    }
    else
    {
        sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
        sprintf(discovery->unique_id, "%s_%s_%s", roomsense_iq_shared.location, discovery->device_tag, last_four_chars);
        sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_ROOM_PRESENCE, discovery->device_tag, last_four_chars, DISCOVERY_PATH);
        sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATE_PATH);
        sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATUS_PATH);
//	if (!strcmp(discovery->device_tag, "Distance-cm"))
//	{
//		printf("2----------------> unit cm");
//		sprintf(discovery->unit_of_measurement, "cm");
//	}
    }

	return ESP_OK;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
char* esp_discovery_serialize_pir_sensor(esp_discovery_pir_sensor_t *discovery)
{
	char *json = NULL;
	cJSON *root = cJSON_CreateObject();

	cJSON *device = cJSON_CreateObject();
	cJSON_AddItemToObject(root, "device", device);
	esp_discovery_device_identity_init();
	cJSON *identifiers = cJSON_CreateStringArray((const char*[]
			)
			{ esp_discovery_device_identifier }, 1);
	cJSON_AddItemToObject(device, "identifiers", identifiers);
	cJSON_AddStringToObject(device, "name", esp_discovery_device_name);

	cJSON_AddStringToObject(root, "platform", discovery->platform);
	cJSON_AddStringToObject(root, "schema", discovery->schema);
	cJSON_AddStringToObject(root, "name", discovery->name);
	cJSON_AddStringToObject(root, "unique_id", discovery->unique_id);
	cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
	cJSON_AddStringToObject(root, "availability_topic", discovery->status_topic);

	json = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);
	return json;
}

esp_err_t esp_discovery_init_pir_sensor(esp_discovery_pir_sensor_t *discovery)
{
	esp_discovery_ensure_pir_sensor(discovery);
	return ESP_OK;
}

esp_err_t esp_discovery_ensure_pir_sensor(esp_discovery_pir_sensor_t *discovery)
{
	char last_four_chars[LAST_FOUR_CHARS_LENGTH + 1]; // Plus one for null terminator

	if (!discovery->esp_serial)
	{
		return ESP_FAIL;
	}

	//esp_serial_hex(discovery->esp_serial, &hex);
	discovery->name = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + 2 + 1);
	discovery->unique_id = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + 3 + 1);
	discovery->discovery_topic = malloc(strlen(DISCOVERY_TAG_PIR_SENSOR) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + strlen(roomsense_iq_shared.mac_address)+ strlen(DISCOVERY_PATH) + 4 + 1);
	discovery->state_topic = malloc(
			strlen(discovery->device_tag) + strlen(roomsense_iq_shared.location) + strlen(roomsense_iq_shared.mac_address) + strlen(STATE_PATH) + 4 + 1);
	discovery->status_topic = malloc(
			strlen(discovery->device_tag) + strlen(roomsense_iq_shared.location) + strlen(roomsense_iq_shared.mac_address) + strlen(STATUS_PATH) + 4 + 1);

	// Copy the last 4 bytes of the MAC address
	strcpy(last_four_chars, roomsense_iq_shared.mac_address + (MAC_ADDRESS_LENGTH - LAST_FOUR_CHARS_LENGTH));

	if (strlen(roomsense_iq_shared.location) == 0)
	{
		sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
		sprintf(discovery->unique_id, "%s_%s", discovery->device_tag, last_four_chars);
		sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_PIR_SENSOR, discovery->device_tag, last_four_chars, DISCOVERY_PATH);
		sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATE_PATH);
		sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATUS_PATH);
	}
	else
	{
		sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
		sprintf(discovery->unique_id, "%s_%s_%s", roomsense_iq_shared.location, discovery->device_tag, last_four_chars);
		sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_PIR_SENSOR, discovery->device_tag, last_four_chars, DISCOVERY_PATH);
		sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATE_PATH);
		sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATUS_PATH);
	}

	return ESP_OK;
}

char* esp_discovery_serialize_macro_move_sensor(esp_discovery_macro_move_sensor_t *discovery)
{
	char *json = NULL;
	cJSON *root = cJSON_CreateObject();

	cJSON *device = cJSON_CreateObject();
	cJSON_AddItemToObject(root, "device", device);
	esp_discovery_device_identity_init();
	cJSON *identifiers = cJSON_CreateStringArray((const char*[]
			)
			{ esp_discovery_device_identifier }, 1);
	cJSON_AddItemToObject(device, "identifiers", identifiers);
	cJSON_AddStringToObject(device, "name", esp_discovery_device_name);

	cJSON_AddStringToObject(root, "platform", discovery->platform);
	cJSON_AddStringToObject(root, "schema", discovery->schema);
	cJSON_AddStringToObject(root, "name", discovery->name);
	cJSON_AddStringToObject(root, "unique_id", discovery->unique_id);
	cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
	cJSON_AddStringToObject(root, "availability_topic", discovery->status_topic);

	json = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);
	return json;
}

esp_err_t esp_discovery_init_macro_move_sensor(esp_discovery_macro_move_sensor_t *discovery)
{
	esp_discovery_ensure_macro_move_sensor(discovery);
	return ESP_OK;
}

esp_err_t esp_discovery_ensure_macro_move_sensor(esp_discovery_macro_move_sensor_t *discovery)
{
	char *hex;

	if (!discovery->esp_serial)
	{
		return ESP_FAIL;
	}

	esp_serial_hex(discovery->esp_serial, &hex);
	discovery->name = malloc(strlen(hex) + strlen(discovery->device_tag) + 2 + 1);
	discovery->unique_id = malloc(strlen(hex) + strlen(discovery->device_tag) + 3 + 1);
	discovery->discovery_topic = malloc(strlen(DISCOVERY_TAG_SENSOR) + strlen(discovery->device_tag) + strlen(hex) + strlen(DISCOVERY_PATH) + 4 + 1);
	discovery->state_topic = malloc(strlen(discovery->device_tag) + strlen(hex) + strlen(STATE_PATH) + 4 + 1);
	discovery->status_topic = malloc(strlen(discovery->device_tag) + strlen(hex) + strlen(STATUS_PATH) + 4 + 1);

	sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
	sprintf(discovery->unique_id, "%s_%s", discovery->device_tag, hex);
	sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_SENSOR, discovery->device_tag, hex, DISCOVERY_PATH);
	sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, hex, STATE_PATH);
	sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, hex, STATUS_PATH);

	free(hex);
	return ESP_OK;
}

char* esp_discovery_serialize_micro_move_sensor(esp_discovery_micro_move_sensor_t *discovery)
{
	char *json = NULL;
	cJSON *root = cJSON_CreateObject();

	cJSON *device = cJSON_CreateObject();
	cJSON_AddItemToObject(root, "device", device);
	esp_discovery_device_identity_init();
	cJSON *identifiers = cJSON_CreateStringArray((const char*[]
			)
			{ esp_discovery_device_identifier }, 1);
	cJSON_AddItemToObject(device, "identifiers", identifiers);
	cJSON_AddStringToObject(device, "name", esp_discovery_device_name);

	cJSON_AddStringToObject(root, "platform", discovery->platform);
	cJSON_AddStringToObject(root, "schema", discovery->schema);
	cJSON_AddStringToObject(root, "name", discovery->name);
	cJSON_AddStringToObject(root, "unique_id", discovery->unique_id);
	cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
	cJSON_AddStringToObject(root, "availability_topic", discovery->status_topic);

	json = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);
	return json;
}

esp_err_t esp_discovery_init_micro_move_sensor(esp_discovery_micro_move_sensor_t *discovery)
{
	esp_discovery_ensure_micro_move_sensor(discovery);
	return ESP_OK;
}

esp_err_t esp_discovery_ensure_micro_move_sensor(esp_discovery_micro_move_sensor_t *discovery)
{
	char *hex;

	if (!discovery->esp_serial)
	{
		return ESP_FAIL;
	}

	esp_serial_hex(discovery->esp_serial, &hex);
	discovery->name = malloc(strlen(hex) + strlen(discovery->device_tag) + 2 + 1);
	discovery->unique_id = malloc(strlen(hex) + strlen(discovery->device_tag) + 3 + 1);
	discovery->discovery_topic = malloc(strlen(DISCOVERY_TAG_SENSOR) + strlen(discovery->device_tag) + strlen(hex) + strlen(DISCOVERY_PATH) + 4 + 1);
	discovery->state_topic = malloc(strlen(discovery->device_tag) + strlen(hex) + strlen(STATE_PATH) + 4 + 1);
	discovery->status_topic = malloc(strlen(discovery->device_tag) + strlen(hex) + strlen(STATUS_PATH) + 4 + 1);

	sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
	sprintf(discovery->unique_id, "%s_%s", discovery->device_tag, hex);
	sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_SENSOR, discovery->device_tag, hex, DISCOVERY_PATH);
	sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, hex, STATE_PATH);
	sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, hex, STATUS_PATH);

	free(hex);
	return ESP_OK;
}

char* esp_discovery_serialize_light_sensor(esp_discovery_light_sensor_t *discovery)
{
	char *json = NULL;
	cJSON *root = cJSON_CreateObject();

	cJSON *device = cJSON_CreateObject();
	cJSON_AddItemToObject(root, "device", device);
	esp_discovery_device_identity_init();
	cJSON *identifiers = cJSON_CreateStringArray((const char*[]
			)
			{ esp_discovery_device_identifier }, 1);
	cJSON_AddItemToObject(device, "identifiers", identifiers);
	cJSON_AddStringToObject(device, "name", esp_discovery_device_name);

	cJSON_AddStringToObject(root, "platform", discovery->platform);
	cJSON_AddStringToObject(root, "schema", discovery->schema);
	cJSON_AddStringToObject(root, "name", discovery->name);
	cJSON_AddStringToObject(root, "unique_id", discovery->unique_id);

	cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
	cJSON_AddStringToObject(root, "availability_topic", discovery->status_topic);
	cJSON_AddStringToObject(root, "unit_of_measurement", discovery->unit_of_measurement);

	json = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);
	return json;
}

esp_err_t esp_discovery_init_light_sensor(esp_discovery_light_sensor_t *discovery)
{
	esp_discovery_ensure_light_sensor(discovery);
	return ESP_OK;
}

esp_err_t esp_discovery_ensure_light_sensor(esp_discovery_light_sensor_t *discovery)
{
	char last_four_chars[LAST_FOUR_CHARS_LENGTH + 1]; // Plus one for null terminator

	if (!discovery->esp_serial)
	{
		return ESP_FAIL;
	}

	//esp_serial_hex(discovery->esp_serial, &hex);
	discovery->name = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + 2 + 1);
	discovery->unique_id = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + 3 + 1);
	discovery->discovery_topic = malloc(
			strlen(DISCOVERY_TAG_SENSOR) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + strlen(roomsense_iq_shared.mac_address)
					+ strlen(DISCOVERY_PATH) + 4 + 1);
	discovery->state_topic = malloc(
			strlen(discovery->device_tag) + strlen(roomsense_iq_shared.location) + strlen(roomsense_iq_shared.mac_address) + strlen(STATE_PATH) + 4 + 1);
	discovery->status_topic = malloc(
			strlen(discovery->device_tag) + strlen(roomsense_iq_shared.location) + strlen(roomsense_iq_shared.mac_address) + strlen(STATUS_PATH) + 4 + 1);
	discovery->unit_of_measurement = malloc(3 + 1);

	// Copy the last 4 bytes of the MAC address
	strcpy(last_four_chars, roomsense_iq_shared.mac_address + (MAC_ADDRESS_LENGTH - LAST_FOUR_CHARS_LENGTH));

	if (strlen(roomsense_iq_shared.location) == 0)
	{
		sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
		sprintf(discovery->unique_id, "%s_%s", discovery->device_tag, last_four_chars);
		sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_SENSOR, discovery->device_tag, last_four_chars, DISCOVERY_PATH);
		sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATE_PATH);
		sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATUS_PATH);
		sprintf(discovery->unit_of_measurement, "Raw");
	}
	else
	{
		sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
		sprintf(discovery->unique_id, "%s_%s_%s", roomsense_iq_shared.location, discovery->device_tag, last_four_chars);
		sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_SENSOR, discovery->device_tag, last_four_chars, DISCOVERY_PATH);
		sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATE_PATH);
		sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATUS_PATH);
		sprintf(discovery->unit_of_measurement, "Raw");
	}

	return ESP_OK;
}

char* esp_discovery_serialize_sensor(esp_discovery_sensor_t *discovery)
{

	char *json = NULL;
	cJSON *root = cJSON_CreateObject();

	cJSON *device = cJSON_CreateObject();
	cJSON_AddItemToObject(root, "device", device);
	esp_discovery_device_identity_init();
	cJSON *identifiers = cJSON_CreateStringArray((const char*[]
			)
			{ esp_discovery_device_identifier }, 1);
	cJSON_AddItemToObject(device, "identifiers", identifiers);
	cJSON_AddStringToObject(device, "name", esp_discovery_device_name);

	cJSON_AddStringToObject(root, "platform", discovery->platform);
	cJSON_AddStringToObject(root, "schema", discovery->schema);
	cJSON_AddStringToObject(root, "name", discovery->name);
	cJSON_AddStringToObject(root, "unique_id", discovery->unique_id);

	cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
	cJSON_AddStringToObject(root, "availability_topic", discovery->status_topic);
	cJSON_AddStringToObject(root, "unit_of_measurement", discovery->unit_of_measurement);

	json = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);
	return json;
}

esp_err_t esp_discovery_init_sensor(esp_discovery_sensor_t *discovery)
{
	esp_discovery_ensure_sensor(discovery);
	return ESP_OK;
}

esp_err_t esp_discovery_ensure_sensor(esp_discovery_sensor_t *discovery)
{
	char last_four_chars[LAST_FOUR_CHARS_LENGTH + 1]; // Plus one for null terminator

	if (!discovery->esp_serial)
	{
		return ESP_FAIL;
	}

	//esp_serial_hex(discovery->esp_serial, &hex);
	discovery->name = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + 2 + 1);
	discovery->unique_id = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + 3 + 1);
	discovery->discovery_topic = malloc(
			strlen(DISCOVERY_TAG_SENSOR) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + strlen(roomsense_iq_shared.mac_address)
					+ strlen(DISCOVERY_PATH) + 4 + 1);
	discovery->state_topic = malloc(
			strlen(discovery->device_tag) + strlen(roomsense_iq_shared.location) + strlen(roomsense_iq_shared.mac_address) + strlen(STATE_PATH) + 4 + 1);
	discovery->status_topic = malloc(
			strlen(discovery->device_tag) + strlen(roomsense_iq_shared.location) + strlen(roomsense_iq_shared.mac_address) + strlen(STATUS_PATH) + 4 + 1);

	// Copy the last 4 bytes of the MAC address
	strcpy(last_four_chars, roomsense_iq_shared.mac_address + (MAC_ADDRESS_LENGTH - LAST_FOUR_CHARS_LENGTH));

	if (strlen(roomsense_iq_shared.location) == 0)
	{
		sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
		sprintf(discovery->unique_id, "%s_%s", discovery->device_tag, last_four_chars);
		sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_SENSOR, discovery->device_tag, last_four_chars, DISCOVERY_PATH);
		sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATE_PATH);
		sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATUS_PATH);
	}
	else
	{
		sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
		sprintf(discovery->unique_id, "%s_%s_%s", roomsense_iq_shared.location, discovery->device_tag, last_four_chars);
		sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_SENSOR, discovery->device_tag, last_four_chars, DISCOVERY_PATH);
		sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATE_PATH);
		sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATUS_PATH);
	}

	return ESP_OK;
}

/////////////////////////////////////////////////////////////
char* esp_discovery_serialize_text(esp_discovery_text_t *discovery)
{

	char *json = NULL;
	cJSON *root = cJSON_CreateObject();

	cJSON *device = cJSON_CreateObject();
	cJSON_AddItemToObject(root, "device", device);
	esp_discovery_device_identity_init();
	cJSON *identifiers = cJSON_CreateStringArray((const char*[]
			)
			{ esp_discovery_device_identifier }, 1);
	cJSON_AddItemToObject(device, "identifiers", identifiers);
	cJSON_AddStringToObject(device, "name", esp_discovery_device_name);

	cJSON_AddStringToObject(root, "platform", discovery->platform);
	cJSON_AddStringToObject(root, "schema", discovery->schema);
	cJSON_AddStringToObject(root, "name", discovery->name);
	cJSON_AddStringToObject(root, "unique_id", discovery->unique_id);
	cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
	cJSON_AddStringToObject(root, "availability_topic", discovery->status_topic);
	cJSON_AddStringToObject(root, "unit_of_measurement", discovery->unit_of_measurement);
	cJSON_AddStringToObject(root, "mode", discovery->mode);

	cJSON_AddStringToObject(root, "command_topic", discovery->set_topic);
	cJSON_AddStringToObject(root, "json_attributes_topic", discovery->attributes_topic);

	json = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);
	return json;
}

esp_err_t esp_discovery_init_text(esp_discovery_text_t *discovery)
{
	esp_discovery_ensure_text(discovery);
	return ESP_OK;
}

esp_err_t esp_discovery_ensure_text(esp_discovery_text_t *discovery)
{
		char last_four_chars[LAST_FOUR_CHARS_LENGTH + 1]; // Plus one for null terminator

		if (!discovery->esp_serial)
		{
			return ESP_FAIL;
		}

	    //esp_serial_hex(discovery->esp_serial, &hex);
	    discovery->name = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + 2 + 1);
	    discovery->unique_id = malloc(strlen(roomsense_iq_shared.mac_address) + strlen(roomsense_iq_shared.location) + strlen(discovery->device_tag) + 3 + 1);
	    discovery->discovery_topic = malloc(strlen(DISCOVERY_TAG_MOVEMENT_DIRECTION) + strlen(roomsense_iq_shared.location) +  strlen(discovery->device_tag) + strlen(roomsense_iq_shared.mac_address) + strlen(DISCOVERY_PATH) + 4 + 1);
	    discovery->state_topic = malloc(strlen(discovery->device_tag) + strlen(roomsense_iq_shared.location) +  strlen(roomsense_iq_shared.mac_address) + strlen(STATE_PATH) + 4 + 1);
	    discovery->status_topic = malloc(strlen(discovery->device_tag) + strlen(roomsense_iq_shared.location) +  strlen(roomsense_iq_shared.mac_address) + strlen(STATUS_PATH) + 4 + 1);
	    discovery->set_topic = malloc(strlen(discovery->device_tag) + strlen(roomsense_iq_shared.mac_address) + strlen(roomsense_iq_shared.location) + strlen(SET_PATH) + 4 + 1);
		discovery->attributes_topic = malloc(strlen(discovery->device_tag) + strlen(roomsense_iq_shared.mac_address) + strlen(roomsense_iq_shared.location) + strlen(ATTRS_PATH) + 4 + 1);

	    // Copy the last 4 bytes of the MAC address
	    strcpy(last_four_chars, roomsense_iq_shared.mac_address + (MAC_ADDRESS_LENGTH - LAST_FOUR_CHARS_LENGTH));

	    if (strlen(roomsense_iq_shared.location) == 0)
	    {
	        sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
	        sprintf(discovery->unique_id, "%s_%s", discovery->device_tag, last_four_chars);
	        sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_MOVEMENT_DIRECTION, discovery->device_tag, last_four_chars, DISCOVERY_PATH);
	        sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATE_PATH);
	        sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATUS_PATH);
	        sprintf(discovery->set_topic, "%s/%s/%s", discovery->device_tag, roomsense_iq_shared.mac_address, SET_PATH);
		    sprintf(discovery->attributes_topic, "%s/%s/%s", discovery->device_tag, roomsense_iq_shared.mac_address, ATTRS_PATH);
	    }
	    else
	    {
	        sprintf(discovery->name, "%s", esp_discovery_friendly_name(discovery->device_tag));
	        sprintf(discovery->unique_id, "%s_%s_%s", roomsense_iq_shared.location, discovery->device_tag, last_four_chars);
	        sprintf(discovery->discovery_topic, "%s/%s-%s/%s", DISCOVERY_TAG_MOVEMENT_DIRECTION, discovery->device_tag, last_four_chars, DISCOVERY_PATH);
	        sprintf(discovery->state_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATE_PATH);
	        sprintf(discovery->status_topic, "%s/%s/%s", discovery->device_tag, last_four_chars, STATUS_PATH);
	        sprintf(discovery->set_topic, "%s/%s/%s", discovery->device_tag, roomsense_iq_shared.mac_address, SET_PATH);
		    sprintf(discovery->attributes_topic, "%s/%s/%s", discovery->device_tag, roomsense_iq_shared.mac_address, ATTRS_PATH);
	    }

		return ESP_OK;
}
