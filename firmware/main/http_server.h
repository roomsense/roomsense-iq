/*
 * http_server.h
 *
 *  Created on: Oct 20, 2021
 *      Author: Sina
 */

#ifndef MAIN_HTTP_SERVER_H_
#define MAIN_HTTP_SERVER_H_

#define OTA_UPDATE_PENDING 		0
#define OTA_UPDATE_SUCCESSFUL	1
#define OTA_UPDATE_FAILED		-1


typedef struct {
    char ipType[20];
    char staticIpValue[16];
    bool mdnsStatus;
    char mdnsUrlValue[256];
} network_config_t;


typedef enum http_server_ap_key_update_status
{
	KEY_UPDATE_SUCESS = 1,
	KEY_UPDATE_FAIL,
} http_server_ap_key_update_status_e;

/**
 * Connection status for Wifi
 */
typedef enum http_server_wifi_connect_status
{
	NONE = 0,
	HTTP_WIFI_STATUS_CONNECTING,
	HTTP_WIFI_STATUS_CONNECT_FAILED,
	HTTP_WIFI_STATUS_CONNECT_SUCCESS,
	HTTP_WIFI_STATUS_DISCONNECTED,
} http_server_wifi_connect_status_e;


/**
 * Connection status for MQTT
 */
typedef enum http_server_mqtt_connect_status
{
	MQTT_NONE = 0,
	HTTP_MQTT_STATUS_CONNECTING,
	HTTP_MQTT_STATUS_CONNECT_FAILED,
	HTTP_MQTT_STATUS_CONNECT_SUCCESS,
	HTTP_MQTT_STATUS_DISCONNECTED,
	HTTP_MQTT_STATUS_ALREADY_CONNECTTED,
} http_server_mqtt_connect_status_e;

/**
 * Messages for the HTTP monitor
 */
typedef enum http_server_message
{
	HTTP_MSG_WIFI_CONNECT_INIT = 0,
	HTTP_MSG_WIFI_CONNECT_SUCCESS,
	HTTP_MSG_WIFI_CONNECT_FAIL,
	HTTP_MSG_WIFI_USER_DISCONNECT,
	HTTP_MSG_OTA_UPDATE_SUCCESSFUL,
	HTTP_MSG_OTA_UPDATE_FAILED,
	HTTP_MSG_MQTT_CONNECT_INIT,
	HTTP_MSG_MQTT_CONNECT_SUCCESS,
	HTTP_MSG_MQTT_CONNECT_FAIL,
	HTTP_MSG_MQTT_DISCONNECT,
	HTTP_MSG_MQTT_ALREADY_CONNECTTED,
	HTTP_MSG_AP_KEY_UPDATE_SUCCESS,
	HTTP_MSG_AP_KEY_UPDATE_FAIL,
} http_server_message_e;

/**
 * Structure for the message queue
 */
typedef struct http_server_queue_message
{
	http_server_message_e msgID;
} http_server_queue_message_t;

/**
 * Sends a message to the queue
 * @param msgID message ID from the http_server_message_e enum.
 * @return pdTRUE if an item was successfully sent to the queue, otherwise pdFALSE.
 * @note Expand the parameter list based on your requirements e.g. how you've expanded the http_server_queue_message_t.
 */
BaseType_t http_server_monitor_send_message(http_server_message_e msgID);

/**
 * Starts the HTTP server.
 */
void http_server_start(void);

/**
 * Stops the HTTP server.
 */
void http_server_stop(void);

/**
 * Timer callback function which calls esp_restart upon successful firmware update.
 */
void http_server_fw_update_reset_callback(void *arg);



#endif /* MAIN_HTTP_SERVER_H_ */
