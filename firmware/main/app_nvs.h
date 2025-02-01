/*
 * app_nvs.h
 *
 *  Created on: Oct 28, 2021
 *      Author: kjagu
 */

#ifndef MAIN_APP_NVS_H_
#define MAIN_APP_NVS_H_

bool app_nvs_load_ap_enable(void);
bool app_nvs_load_led_enable(void);

esp_err_t app_nvs_save_ap_key(void);
bool      app_nvs_load_ap_key(void);

/**
 * Saves station mode Wifi credentials to NVS
 * @return ESP_OK if successful.
 */
esp_err_t app_nvs_save_sta_creds(void);

/**
 * Loads the previously saved credentials from NVS.
 * @return true if previously saved credentials were found.
 */
bool app_nvs_load_sta_creds(void);

/**
 * Clears station mode credentials from NVS
 * @return ESP_OK if successful.
 */
esp_err_t app_nvs_clear_sta_creds(void);



esp_err_t app_nvs_save_mqtt_creds(void);

bool app_nvs_load_mqtt_creds(void);

esp_err_t app_nvs_clear_mqtt_creds(void);

esp_err_t app_nvs_save_occupancy_status(void);
bool      app_nvs_load_occupancy_status(void);

esp_err_t app_nvs_save_sensor_location(void);
bool      app_nvs_load_sensor_location(void);

#endif /* MAIN_APP_NVS_H_ */
