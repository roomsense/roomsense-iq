/*
 * app_nvs.c
 *
 *  Created on: Oct 28, 2021
 *      Author: kjagu
 */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "tasks_common.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "app_nvs.h"
#include "wifi_app.h"
#include "ha_mqtt.h"

// Tag for logging to the monitor
static const char TAG[] = "nvs";

const char nvs_namespace[] = "nvs_storage";

bool app_nvs_load_ap_enable(void){
	nvs_handle handle;
	esp_err_t esp_err;
	char ap_state[32] = {0};
	if (nvs_open("net_config", NVS_READONLY, &handle) == ESP_OK)
	{

		size_t len = 32;
		nvs_get_blob(handle, "AP_STATE", ap_state, &len);
		nvs_close(handle);
		printf("[NVS] AP_STATE: %s ", ap_state);
		if(len > 0){
			if(!strcmp(ap_state,"true"))
			{
				roomsense_iq_shared.ap_enable = true;
				return true;
			}
			else if(!strcmp(ap_state,"false"))
			{
				roomsense_iq_shared.ap_enable = false;
				return false;
			}
			else return false;
		}
	}
	return false;
}


bool app_nvs_load_led_enable(void){
	nvs_handle handle;
	esp_err_t esp_err;
	char led_state[32] = {0};
	if (nvs_open("net_config", NVS_READONLY, &handle) == ESP_OK)
	{

		size_t len = 32;
		nvs_get_blob(handle, "LED_STATE", led_state, &len);
		nvs_close(handle);
		printf("[NVS] led_state: %s ", led_state);
		if(len > 0){
			if(!strcmp(led_state,"true"))
			{
				roomsense_iq_shared.led_enable = true;
				//xEventGroupSetBits(led_event_group, RGB_LED_BIT); // unlock led task
				return true;
			}
			else if(!strcmp(led_state,"false"))
			{
				roomsense_iq_shared.led_enable = false;
				//xEventGroupSetBits(led_event_group, RGB_LED_BIT); // unlock led task
				return false;
			}
			else return false;
		}
	}
	return false;
}

esp_err_t app_nvs_save_ap_key(void)
{
	nvs_handle handle;
	esp_err_t esp_err;
	//ESP_LOGI(TAG, "----------------------------> app_nvs_save_ap_key: Saving access point key %s to flash\n", roomsense_iq_shared.ap_key);

	if (roomsense_iq_shared.ap_key)
	{
		esp_err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_ap_key: Error (%s) opening NVS handle!\n", esp_err_to_name(esp_err));
			return esp_err;
		}

		// Set AP Key
		esp_err = nvs_set_blob(handle, "key", roomsense_iq_shared.ap_key, MAX_SSID_LENGTH);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_ap_key: Error (%s) setting ap Key to NVS!\n", esp_err_to_name(esp_err));
			return esp_err;
		}


		// Commit credentials to NVS
		esp_err = nvs_commit(handle);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_sta_creds: Error (%s) comitting credentials to NVS!\n", esp_err_to_name(esp_err));
			return esp_err;
		}
		nvs_close(handle);
		//ESP_LOGI(TAG, "app_nvs_save_ap_key: wrote AP Key: %s %s", roomsense_iq_shared.ap_key);
	}

	printf("app_nvs_save_ap_key: returned ESP_OK\n");
	return ESP_OK;
}

bool app_nvs_load_ap_key(void)
{
	nvs_handle handle;
	esp_err_t esp_err;

	ESP_LOGI(TAG, "app_nvs_load_ap_key: Loading sensor's ap key from flash");

	size_t ap_key_size = sizeof(roomsense_iq_shared.ap_key);
	uint8_t *ap_key_buff = (uint8_t*) malloc(sizeof(uint8_t) * ap_key_size);
	memset(ap_key_buff, 0x00, sizeof(ap_key_size));

	if (nvs_open(nvs_namespace, NVS_READONLY, &handle) == ESP_OK)
	{

		esp_err = nvs_get_blob(handle, "key", ap_key_buff, &ap_key_size);
		if (esp_err != ESP_OK)
		{
			free(ap_key_buff);
			printf("app_nvs_load_ap_key: (%s) no access point key found in NVS\n", esp_err_to_name(esp_err));
			memcpy(roomsense_iq_shared.ap_key, "password", strlen("password"));
			printf("------> app_nvs_load_ap_key: %s\n", roomsense_iq_shared.ap_key);
			nvs_close(handle);
			return false;
		}
		memcpy(roomsense_iq_shared.ap_key, ap_key_buff, ap_key_size);
		nvs_close(handle);

		printf("------> ap_key_buff %s app_nvs_load_ap_key: %s ap_key_size %d\n", ap_key_buff, roomsense_iq_shared.ap_key, ap_key_size);

		free(ap_key_buff);
		return true;
	}
	else
	{
		free(ap_key_buff);
		return false;
	}
}

esp_err_t app_nvs_save_sta_creds(void)
{
	nvs_handle handle;
	esp_err_t esp_err;
	ESP_LOGI(TAG, "app_nvs_save_sta_creds: Saving station mode credentials to flash");

	wifi_config_t *wifi_sta_config = wifi_app_get_wifi_config();

	if (wifi_sta_config)
	{
		esp_err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_sta_creds: Error (%s) opening NVS handle!\n", esp_err_to_name(esp_err));
			return esp_err;
		}

		// Set SSID
		esp_err = nvs_set_blob(handle, "ssid", wifi_sta_config->sta.ssid, MAX_SSID_LENGTH);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_sta_creds: Error (%s) setting SSID to NVS!\n", esp_err_to_name(esp_err));
			return esp_err;
		}

		// Set Password
		esp_err = nvs_set_blob(handle, "password", wifi_sta_config->sta.password, MAX_PASSWORD_LENGTH);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_sta_creds: Error (%s) setting Password to NVS!\n", esp_err_to_name(esp_err));
			return esp_err;
		}

		// Commit credentials to NVS
		esp_err = nvs_commit(handle);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_sta_creds: Error (%s) comitting credentials to NVS!\n", esp_err_to_name(esp_err));
			return esp_err;
		}
		nvs_close(handle);
		ESP_LOGI(TAG, "app_nvs_save_sta_creds: wrote wifi_sta_config: Station SSID: %s Password: %s", wifi_sta_config->sta.ssid, wifi_sta_config->sta.password);
	}

	printf("app_nvs_save_sta_creds: returned ESP_OK\n");
	return ESP_OK;
}

bool app_nvs_load_sta_creds(void)
{
	nvs_handle handle;
	esp_err_t esp_err;

	ESP_LOGI(TAG, "app_nvs_load_sta_creds: Loading Wifi credentials from flash");

	if (nvs_open(nvs_namespace, NVS_READONLY, &handle) == ESP_OK)
	{
		wifi_config_t *wifi_sta_config = wifi_app_get_wifi_config();

		if (wifi_sta_config == NULL)
		{
			wifi_sta_config = (wifi_config_t*) malloc(sizeof(wifi_config_t));
		}
		memset(wifi_sta_config, 0x00, sizeof(wifi_config_t));

		// Allocate buffer
		size_t wifi_config_size = sizeof(wifi_config_t);
		uint8_t *wifi_config_buff = (uint8_t*) malloc(sizeof(uint8_t) * wifi_config_size);
		memset(wifi_config_buff, 0x00, sizeof(wifi_config_size));

		// Load SSID
		wifi_config_size = sizeof(wifi_sta_config->sta.ssid);
		esp_err = nvs_get_blob(handle, "ssid", wifi_config_buff, &wifi_config_size);
		if (esp_err != ESP_OK)
		{
			free(wifi_config_buff);
			printf("app_nvs_load_sta_creds: (%s) no station SSID found in NVS\n", esp_err_to_name(esp_err));
			return false;
		}
		memcpy(wifi_sta_config->sta.ssid, wifi_config_buff, wifi_config_size);

		// Load Password
		wifi_config_size = sizeof(wifi_sta_config->sta.password);
		esp_err = nvs_get_blob(handle, "password", wifi_config_buff, &wifi_config_size);
		if (esp_err != ESP_OK)
		{
			free(wifi_config_buff);
			printf("app_nvs_load_sta_creds: (%s) retrieving password!\n", esp_err_to_name(esp_err));
			return false;
		}
		memcpy(wifi_sta_config->sta.password, wifi_config_buff, wifi_config_size);

		free(wifi_config_buff);
		nvs_close(handle);

		printf("app_nvs_load_sta_creds: SSID: %s Password: %s\n", wifi_sta_config->sta.ssid, wifi_sta_config->sta.password);
		return wifi_sta_config->sta.ssid[0] != '\0';
	}
	else
	{
		return false;
	}
}

esp_err_t app_nvs_clear_sta_creds(void)
{
	nvs_handle handle;
	esp_err_t esp_err;
	ESP_LOGI(TAG, "app_nvs_clear_sta_creds: Clearing Wifi station mode credentials from flash");

	esp_err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
	if (esp_err != ESP_OK)
	{
		printf("app_nvs_clear_sta_creds: Error (%s) opening NVS handle!\n", esp_err_to_name(esp_err));
		return esp_err;
	}

	// Erase credentials
	esp_err = nvs_erase_all(handle);
	if (esp_err != ESP_OK)
	{
		printf("app_nvs_clear_sta_creds: Error (%s) erasing station mode credentials!\n", esp_err_to_name(esp_err));
		return esp_err;
	}

	// Commit clearing credentials from NVS
	esp_err = nvs_commit(handle);
	if (esp_err != ESP_OK)
	{
		printf("app_nvs_clear_sta_creds: Error (%s) NVS commit!\n", esp_err_to_name(esp_err));
		return esp_err;
	}
	nvs_close(handle);

	printf("app_nvs_clear_sta_creds: returned ESP_OK\n");

	return ESP_OK;
}

esp_err_t app_nvs_save_mqtt_creds(void)
{
	nvs_handle handle;
	esp_err_t esp_err;
	ESP_LOGI(TAG, "app_nvs_save_mqtt_creds: Saving MQTT credentials to flash");

	mqtt_credentials_t *mqtt_credentials = get_mqtt_credentials();

	if (mqtt_credentials)
	{
		esp_err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_mqtt_creds: Error (%s) opening NVS handle!\n", esp_err_to_name(esp_err));
			return esp_err;
		}

		// Set host
		esp_err = nvs_set_blob(handle, "mqtt_host", mqtt_credentials->mqtt_host, MAX_MQTT_HOST_LENGTH);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_mqtt_creds: Error (%s) setting mqtt_host to NVS!\n", esp_err_to_name(esp_err));
			return esp_err;
		}

		// Set port
		esp_err = nvs_set_blob(handle, "mqtt_port", mqtt_credentials->mqtt_port, MAX_MQTT_PORT_LENGTH);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_mqtt_creds: Error (%s) setting mqtt_port to NVS!\n", esp_err_to_name(esp_err));
			return esp_err;
		}

		// Set Username
		esp_err = nvs_set_blob(handle, "username", mqtt_credentials->mqtt_username, MAX_USERNAME_LENGTH);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_mqtt_creds: Error (%s) setting Username to NVS!\n", esp_err_to_name(esp_err));
			return esp_err;
		}

		// Set Password
		esp_err = nvs_set_blob(handle, "mqtt_password", mqtt_credentials->mqtt_password, MAX_MQTT_PASSWORD_LENGTH);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_mqtt_creds: Error (%s) setting Password to NVS!\n", esp_err_to_name(esp_err));
			return esp_err;
		}

		// Commit credentials to NVS
		esp_err = nvs_commit(handle);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_mqtt_creds: Error (%s) comitting MQTT credentials to NVS!\n", esp_err_to_name(esp_err));
			return esp_err;
		}
		nvs_close(handle);
		ESP_LOGI(TAG, "app_nvs_save_mqtt_creds: wrote mqtt_credentials: MQTT Host %s Port %s Username: %s Password: %s", mqtt_credentials->mqtt_host,
				mqtt_credentials->mqtt_port, mqtt_credentials->mqtt_username, mqtt_credentials->mqtt_password);
	}

	printf("app_nvs_save_mqtt_creds: returned ESP_OK\n");
	return ESP_OK;
}

bool app_nvs_load_mqtt_creds(void)
{
	nvs_handle handle;
	esp_err_t esp_err;

	ESP_LOGI(TAG, "app_nvs_load_mqtt_creds: Loading MQTT credentials from flash");

	if (nvs_open(nvs_namespace, NVS_READONLY, &handle) == ESP_OK)
	{
		mqtt_credentials_t *mqtt_credentials = get_mqtt_credentials();

		if (mqtt_credentials == NULL)
		{
			mqtt_credentials = (mqtt_credentials_t*) malloc(sizeof(mqtt_credentials_t));
		}
		memset(mqtt_credentials, 0x00, sizeof(mqtt_credentials_t));

		// Allocate buffer
		size_t mqtt_config_size = sizeof(mqtt_credentials_t);
		uint8_t *mqtt_config_buff = (uint8_t*) malloc(sizeof(uint8_t) * mqtt_config_size);
		memset(mqtt_config_buff, 0x00, sizeof(mqtt_config_size));

		// Load Host
		mqtt_config_size = sizeof(mqtt_credentials->mqtt_host);
		esp_err = nvs_get_blob(handle, "mqtt_host", mqtt_config_buff, &mqtt_config_size);
		if (esp_err != ESP_OK)
		{
			free(mqtt_config_buff);
			printf("app_nvs_load_mqtt_creds: (%s) no MQTT mqtt_host found in NVS\n", esp_err_to_name(esp_err));
			return false;
		}
		memcpy(mqtt_credentials->mqtt_host, mqtt_config_buff, mqtt_config_size);

		// Load Port
		mqtt_config_size = sizeof(mqtt_credentials->mqtt_port);
		esp_err = nvs_get_blob(handle, "mqtt_port", mqtt_config_buff, &mqtt_config_size);
		if (esp_err != ESP_OK)
		{
			free(mqtt_config_buff);
			printf("app_nvs_load_mqtt_creds: (%s) no MQTT mqtt_port found in NVS\n", esp_err_to_name(esp_err));
			return false;
		}
		memcpy(mqtt_credentials->mqtt_port, mqtt_config_buff, mqtt_config_size);

		// Load Username
		mqtt_config_size = sizeof(mqtt_credentials->mqtt_username);
		esp_err = nvs_get_blob(handle, "username", mqtt_config_buff, &mqtt_config_size);
		if (esp_err != ESP_OK)
		{
			free(mqtt_config_buff);
			printf("app_nvs_load_mqtt_creds: (%s) no MQTT Username found in NVS\n", esp_err_to_name(esp_err));
			return false;
		}
		memcpy(mqtt_credentials->mqtt_username, mqtt_config_buff, mqtt_config_size);

		// Load Password
		mqtt_config_size = sizeof(mqtt_credentials->mqtt_password);
		esp_err = nvs_get_blob(handle, "mqtt_password", mqtt_config_buff, &mqtt_config_size);
		if (esp_err != ESP_OK)
		{
			free(mqtt_config_buff);
			printf("app_nvs_load_mqtt_creds: (%s) retrieving password!\n", esp_err_to_name(esp_err));
			return false;
		}
		memcpy(mqtt_credentials->mqtt_password, mqtt_config_buff, mqtt_config_size);

		free(mqtt_config_buff);
		nvs_close(handle);

		printf("app_nvs_load_mqtt_creds: Host:%s, Port:%s Username:%s Password:%s\n", mqtt_credentials->mqtt_host, mqtt_credentials->mqtt_port,
				mqtt_credentials->mqtt_username, mqtt_credentials->mqtt_password);
		return mqtt_credentials->mqtt_username[0] != '\0';
	}
	else
	{
		return false;
	}
}

esp_err_t app_nvs_clear_mqtt_creds(void)
{
	nvs_handle handle;
	esp_err_t esp_err;
	ESP_LOGI(TAG, "app_nvs_clear_mqtt_creds: Clearing MQTT credentials from flash");

	esp_err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
	if (esp_err != ESP_OK)
	{
		printf("app_nvs_clear_mqtt_creds: Error (%s) opening NVS handle!\n", esp_err_to_name(esp_err));
		return esp_err;
	}

	// Erase credentials
	esp_err = nvs_erase_all(handle);
	if (esp_err != ESP_OK)
	{
		printf("app_nvs_clear_mqtt_creds: Error (%s) erasing MQTT credentials!\n", esp_err_to_name(esp_err));
		return esp_err;
	}

	// Commit clearing credentials from NVS
	esp_err = nvs_commit(handle);
	if (esp_err != ESP_OK)
	{
		printf("app_nvs_clear_mqtt_creds: Error (%s) NVS commit!\n", esp_err_to_name(esp_err));
		return esp_err;
	}
	nvs_close(handle);

	printf("app_nvs_clear_mqtt_creds: returned ESP_OK\n");
	return ESP_OK;
}

esp_err_t app_nvs_save_occupancy_status(void)
{
	nvs_handle handle;
	esp_err_t esp_err;
	ESP_LOGI(TAG, "app_nvs_save_occupancy_status: Saving occupancy status to flash");

	esp_err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
	if (esp_err != ESP_OK)
	{
		printf("app_nvs_save_occupancy_status: Error (%s) opening NVS handle!\n", esp_err_to_name(esp_err));
		return esp_err;
	}

	// Set Occupancy
	esp_err = nvs_set_blob(handle, "occu_s", &roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy, sizeof(uint8_t));
	if (esp_err != ESP_OK)
	{
		printf("app_nvs_save_occupancy_status: Error (%s) setting occupancy_status to NVS!\n", esp_err_to_name(esp_err));
		return esp_err;
	}

	// Commit credentials to NVS
	esp_err = nvs_commit(handle);
	if (esp_err != ESP_OK)
	{
		printf("app_nvs_save_occupancy_status: Error (%s) committing occupancy_status to NVS!\n", esp_err_to_name(esp_err));
		return esp_err;
	}
	nvs_close(handle);
	ESP_LOGI(TAG, "app_nvs_save_occupancy_status: wrote occupancy_status %d", roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy);

	printf("app_nvs_save_occupancy_status: returned ESP_OK\n");
	return ESP_OK;
}

bool app_nvs_load_occupancy_status(void)
{
	nvs_handle handle;
	esp_err_t esp_err;
	//uint8_t loaded_occupancy_status;

	ESP_LOGI(TAG, "app_nvs_load_occupancy_status: Loading Occupancy status from flash");

	if (nvs_open(nvs_namespace, NVS_READONLY, &handle) == ESP_OK)
	{
		uint8_t *loaded_occupancy_status = (uint8_t*) malloc(sizeof(uint8_t));
		memset(loaded_occupancy_status, 0x00, 1);

		size_t occu_s_size = sizeof(uint8_t);

		esp_err = nvs_get_blob(handle, "occu_s", loaded_occupancy_status, &occu_s_size);
		if (esp_err != ESP_OK)
		{
			free(loaded_occupancy_status);
			printf("app_nvs_load_occupancy_status: (%s) no occupancy_status found in NVS\n", esp_err_to_name(esp_err));
			return false;
		}
		roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy = *loaded_occupancy_status;

		free(loaded_occupancy_status);
		nvs_close(handle);
		ESP_LOGI(TAG, "app_nvs_load_occupancy_status: g_occupancy_status: %d\n", roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy);
		return true;
	}
	else
	{
		return false;
	}
}

esp_err_t app_nvs_save_sensor_location(void)
{
	nvs_handle handle;
	esp_err_t esp_err;
	ESP_LOGI(TAG, "Saving sensor's location to flash");

	esp_err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
	if (esp_err != ESP_OK)
	{
		printf("app_nvs_save_location: Error (%s) opening NVS handle!\n", esp_err_to_name(esp_err));
		return esp_err;
	}

	esp_err = nvs_set_blob(handle, "location", roomsense_iq_shared.location, MAX_MQTT_PUBLISH_LENGTH);
	if (esp_err != ESP_OK)
	{
		printf("app_nvs_save_location: Error (%s) setting sensor's location NVS!\n", esp_err_to_name(esp_err));
		return esp_err;
	}

	esp_err = nvs_commit(handle);
	if (esp_err != ESP_OK)
	{
		printf("Error (%s) comitting sensor's location to NVS!\n", esp_err_to_name(esp_err));
		return esp_err;
	}
	nvs_close(handle);
	ESP_LOGI(TAG, "app_nvs_save_sensor_location: wrote sensor's location %s", roomsense_iq_shared.location);

	printf("app_nvs_save_sensor_location: returned ESP_OK\n");
	return ESP_OK;
}

bool app_nvs_load_sensor_location(void)
{
	nvs_handle handle;
	esp_err_t esp_err;

	ESP_LOGI(TAG, "app_nvs_load_sensor_location: Loading sensor's location from flash");

	size_t location_size = sizeof(roomsense_iq_shared.location);
	uint8_t *location_buff = (uint8_t*) malloc(sizeof(uint8_t) * location_size);
	memset(location_buff, 0x00, sizeof(location_size));

	if (nvs_open(nvs_namespace, NVS_READONLY, &handle) == ESP_OK)
	{

		esp_err = nvs_get_blob(handle, "ssid", location_buff, &location_size);
		if (esp_err != ESP_OK)
		{
			free(location_buff);
			printf("app_nvs_load_location: (%s) no sensor's location found in NVS\n", esp_err_to_name(esp_err));
			return false;
		}
		memcpy(roomsense_iq_shared.location, location_buff, location_size);
		nvs_close(handle);

		//printf("app_nvs_load_sensor_location: %s\n", roomsense_iq_shared->location);
		return true;
	}
	else
	{
		return false;
	}
}

