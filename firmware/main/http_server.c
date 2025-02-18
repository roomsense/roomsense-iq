/*
 * http_server.c
 *
 *  Created on: Oct 20, 2021
 *      Author: Sina
 */

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "sys/param.h"
#include <esp_wifi.h>
#include "lwip/sockets.h"
#include "nvs.h"
#include "http_server.h"
#include "tasks_common.h"
#include "wifi_app.h"
#include "driver/gpio.h"

#include "ha_mqtt.h"
#include "app_nvs.h"
#include "rgb_led.h"

#include "dashboard.h"

bool g_ws_pause = false;

// Tag used for ESP serial console messages
static const char TAG[] = "http_server";

static int g_ap_save_status = NONE;

// Wifi connect status
static int g_wifi_connect_status = NONE;

static int g_mqtt_connect_status = NONE;

// Firmware update status
static int g_fw_update_status = OTA_UPDATE_PENDING;

// HTTP server task handle
static httpd_handle_t http_server_handle = NULL;

// HTTP server monitor task handle
static TaskHandle_t task_http_server_monitor = NULL;

// Queue handle used to manipulate the main queue of events
static QueueHandle_t http_server_monitor_queue_handle;

/**
 * ESP32 timer configuration passed to esp_timer_create.
 */
const esp_timer_create_args_t fw_update_reset_args = {
		.callback = &http_server_fw_update_reset_callback,
		.arg = NULL,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "fw_update_reset"
};
esp_timer_handle_t fw_update_reset;

extern EventGroupHandle_t led_event_group;
extern const int RGB_LED_BIT;

// Embedded files: JQuery, index.html, app.css, app.js and favicon.ico files
extern const uint8_t jquery_3_3_1_min_js_start[]	asm("_binary_jquery_3_3_1_min_js_start");
extern const uint8_t jquery_3_3_1_min_js_end[]		asm("_binary_jquery_3_3_1_min_js_end");
extern const uint8_t index_html_start[]				asm("_binary_index_html_start");
extern const uint8_t index_html_end[]				asm("_binary_index_html_end");
extern const uint8_t app_css_start[]				asm("_binary_app_css_start");
extern const uint8_t app_css_end[]					asm("_binary_app_css_end");
extern const uint8_t app_js_start[]					asm("_binary_app_js_start");
extern const uint8_t app_js_end[]					asm("_binary_app_js_end");
extern const uint8_t favicon_ico_start[]			asm("_binary_favicon_ico_start");
extern const uint8_t favicon_ico_end[]				asm("_binary_favicon_ico_end");
extern const uint8_t icon_png_start[]			    asm("_binary_icon_png_start");
extern const uint8_t icon_png_end[]				    asm("_binary_icon_png_end");

/**
 * Checks the g_fw_update_status and creates the fw_update_reset timer if g_fw_update_status is true.
 */
static void http_server_fw_update_reset_timer(void)
{
	if (g_fw_update_status == OTA_UPDATE_SUCCESSFUL)
	{
		ESP_LOGI(TAG, "http_server_fw_update_reset_timer: FW updated successful starting FW update reset timer");

		// Give the web page a chance to receive an acknowledge back and initialize the timer
		ESP_ERROR_CHECK(esp_timer_create(&fw_update_reset_args, &fw_update_reset));
		ESP_ERROR_CHECK(esp_timer_start_once(fw_update_reset, 8000000));
	}
	else
	{
		ESP_LOGI(TAG, "http_server_fw_update_reset_timer: FW update unsuccessful");
	}
}

/**
 * HTTP server monitor task used to track events of the HTTP server
 * @param pvParameters parameter which can be passed to the task.
 */
static void http_server_monitor(void *parameter)
{
	http_server_queue_message_t msg;

	for (;;)
	{
		if (xQueueReceive(http_server_monitor_queue_handle, &msg, portMAX_DELAY))
		{
			switch (msg.msgID)
			{
				case HTTP_MSG_WIFI_CONNECT_INIT:
					ESP_LOGI(TAG, "HTTP_MSG_WIFI_CONNECT_INIT");

					g_wifi_connect_status = HTTP_WIFI_STATUS_CONNECTING;

					break;

				case HTTP_MSG_WIFI_CONNECT_SUCCESS:
					ESP_LOGI(TAG, "HTTP_MSG_WIFI_CONNECT_SUCCESS");

					g_wifi_connect_status = HTTP_WIFI_STATUS_CONNECT_SUCCESS;
					roomsense_iq_shared.rgb_led_state = WIFI_CONNECTED;
					xEventGroupSetBits(led_event_group, RGB_LED_BIT);

					break;

				case HTTP_MSG_WIFI_CONNECT_FAIL:
					ESP_LOGI(TAG, "HTTP_MSG_WIFI_CONNECT_FAIL");

					g_wifi_connect_status = HTTP_WIFI_STATUS_CONNECT_FAILED;
					roomsense_iq_shared.rgb_led_state = WIFI_DISCONNECTED;
					xEventGroupSetBits(led_event_group, RGB_LED_BIT);

					break;

				case HTTP_MSG_WIFI_USER_DISCONNECT:
					ESP_LOGI(TAG, "HTTP_MSG_WIFI_USER_DISCONNECT");

					g_wifi_connect_status = HTTP_WIFI_STATUS_DISCONNECTED;
					roomsense_iq_shared.rgb_led_state = WIFI_DISCONNECTED;
					xEventGroupSetBits(led_event_group, RGB_LED_BIT);

					break;

				case HTTP_MSG_OTA_UPDATE_SUCCESSFUL:
					ESP_LOGI(TAG, "HTTP_MSG_OTA_UPDATE_SUCCESSFUL");
					g_fw_update_status = OTA_UPDATE_SUCCESSFUL;
					http_server_fw_update_reset_timer();

					break;

				case HTTP_MSG_OTA_UPDATE_FAILED:
					ESP_LOGI(TAG, "HTTP_MSG_OTA_UPDATE_FAILED");
					g_fw_update_status = OTA_UPDATE_FAILED;

					break;

				case HTTP_MSG_MQTT_CONNECT_INIT:
					ESP_LOGI(TAG, "HTTP_MSG_MQTT_CONNECT_INIT");

					g_mqtt_connect_status = HTTP_MQTT_STATUS_CONNECTING;

					break;

				case HTTP_MSG_MQTT_CONNECT_SUCCESS:
					ESP_LOGI(TAG, "HTTP_MSG_MQTT_CONNECT_SUCCESS");

					g_mqtt_connect_status = HTTP_MQTT_STATUS_CONNECT_SUCCESS;

					break;

				case HTTP_MSG_MQTT_CONNECT_FAIL:
					ESP_LOGI(TAG, "HTTP_MSG_MQTT_CONNECT_FAIL");
					g_mqtt_connect_status = HTTP_MQTT_STATUS_CONNECT_FAILED;

					break;

				case HTTP_MSG_MQTT_DISCONNECT:
					ESP_LOGI(TAG, "HTTP_MSG_MQTT_DISCONNECT");
					g_mqtt_connect_status = HTTP_MQTT_STATUS_DISCONNECTED;

					break;

				case HTTP_MSG_MQTT_ALREADY_CONNECTTED:
					ESP_LOGI(TAG, "HTTP_MSG_MQTT_ALREADY_CONNECTTED");
					g_mqtt_connect_status = HTTP_MQTT_STATUS_ALREADY_CONNECTTED;

					break;

				case HTTP_MSG_AP_KEY_UPDATE_SUCCESS:
					ESP_LOGI(TAG, "HTTP_MSG_AP_KEY_UPDATE_SUCCESS");
					g_ap_save_status = KEY_UPDATE_SUCESS;

					break;

				case HTTP_MSG_AP_KEY_UPDATE_FAIL:
					ESP_LOGI(TAG, "HTTP_MSG_AP_KEY_UPDATE_FAIL");
					g_ap_save_status = KEY_UPDATE_FAIL;

					break;

				default:
					break;
			}
		}
	}
}

/**
 * Jquery get handler is requested when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_jquery_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "Jquery requested");

	httpd_resp_set_type(req, "application/javascript");
	httpd_resp_send(req, (const char *)jquery_3_3_1_min_js_start, jquery_3_3_1_min_js_end - jquery_3_3_1_min_js_start);

	return ESP_OK;
}

/**
 * Sends the index.html page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_index_html_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "-------------------->index.html requested");

	httpd_resp_set_type(req, "text/html");
	httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);

	return ESP_OK;
}

/**
 * app.css get handler is requested when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_app_css_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "app.css requested");

	httpd_resp_set_type(req, "text/css");
	httpd_resp_send(req, (const char *)app_css_start, app_css_end - app_css_start);

	return ESP_OK;
}

/**
 * app.js get handler is requested when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_app_js_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "app.js requested");

	httpd_resp_set_type(req, "application/javascript");
	httpd_resp_send(req, (const char *)app_js_start, app_js_end - app_js_start);

	return ESP_OK;
}

/**
 * Sends the .ico (icon) file when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_favicon_ico_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "favicon.ico requested");

	httpd_resp_set_type(req, "image/x-icon");
	httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_end - favicon_ico_start);

	g_ws_pause = false;

	return ESP_OK;
}

/**
 * Sends the .ico (icon) file when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_icon_png_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "icon.png requested");

	httpd_resp_set_type(req, "image/x-icon");
	httpd_resp_send(req, (const char *)icon_png_start, icon_png_end - icon_png_start);

	return ESP_OK;
}

/**
 * Receives the .bin file fia the web page and handles the firmware update
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK, otherwise ESP_FAIL if timeout occurs and the update cannot be started.
 */
esp_err_t http_server_OTA_update_handler(httpd_req_t *req)
{
	esp_ota_handle_t ota_handle;

	char ota_buff[1024];
	int content_length = req->content_len;
	int content_received = 0;
	int recv_len;
	bool is_req_body_started = false;
	bool flash_successful = false;

	const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

	do
	{
		// Read the data for the request
		if ((recv_len = httpd_req_recv(req, ota_buff, MIN(content_length, sizeof(ota_buff)))) < 0)
		{
			// Check if timeout occurred
			if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
			{
				ESP_LOGI(TAG, "http_server_OTA_update_handler: Socket Timeout");
				continue; ///> Retry receiving if timeout occurred
			}
			ESP_LOGI(TAG, "http_server_OTA_update_handler: OTA other Error %d", recv_len);
			return ESP_FAIL;
		}
		printf("http_server_OTA_update_handler: OTA RX: %d of %d\r", content_received, content_length);

		// Is this the first data we are receiving
		// If so, it will have the information in the header that we need.
		if (!is_req_body_started)
		{
			is_req_body_started = true;

			// Get the location of the .bin file content (remove the web form data)
			char *body_start_p = strstr(ota_buff, "\r\n\r\n") + 4;
			int body_part_len = recv_len - (body_start_p - ota_buff);

			printf("http_server_OTA_update_handler: OTA file size: %d\r\n", content_length);

			esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
			if (err != ESP_OK)
			{
				printf("http_server_OTA_update_handler: Error with OTA begin, cancelling OTA\r\n");
				return ESP_FAIL;
			}
			else
			{
				printf("http_server_OTA_update_handler: Writing to partition subtype %d at offset 0x%x\r\n", update_partition->subtype, update_partition->address);
			}

			// Write this first part of the data
			esp_ota_write(ota_handle, body_start_p, body_part_len);
			content_received += body_part_len;
		}
		else
		{
			// Write OTA data
			esp_ota_write(ota_handle, ota_buff, recv_len);
			content_received += recv_len;
		}

	} while (recv_len > 0 && content_received < content_length);

	if (esp_ota_end(ota_handle) == ESP_OK)
	{
		// Lets update the partition
		if (esp_ota_set_boot_partition(update_partition) == ESP_OK)
		{
			const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
			ESP_LOGI(TAG, "http_server_OTA_update_handler: Next boot partition subtype %d at offset 0x%x", boot_partition->subtype, boot_partition->address);
			flash_successful = true;
		}
		else
		{
			ESP_LOGI(TAG, "http_server_OTA_update_handler: FLASHED ERROR!!!");
		}
	}
	else
	{
		ESP_LOGI(TAG, "http_server_OTA_update_handler: esp_ota_end ERROR!!!");
	}

	// We won't update the global variables throughout the file, so send the message about the status
	if (flash_successful)
	{
		http_server_monitor_send_message(HTTP_MSG_OTA_UPDATE_SUCCESSFUL);

		//factory reset ld2410 and pir sensitivity before fw update.
		roomsense_iq_shared.buttons_shared.g_button_factory_reset = true;
	}
	else
	{
		http_server_monitor_send_message(HTTP_MSG_OTA_UPDATE_FAILED);
	}

	return ESP_OK;
}

/**
 * OTA status handler responds with the firmware update status after the OTA update is started
 * and responds with the compile time/date when the page is first requested
 * @param req HTTP request for which the uri needs to be handled
 * @return ESP_OK
 */
esp_err_t http_server_OTA_status_handler(httpd_req_t *req)
{
	char otaJSON[100];

	ESP_LOGI(TAG, "OTAstatus requested");

	sprintf(otaJSON, "{\"ota_update_status\":%d,\"compile_time\":\"%s\",\"compile_date\":\"%s\"}", g_fw_update_status, __TIME__, __DATE__);

	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, otaJSON, strlen(otaJSON));

	return ESP_OK;
}

/**
 * key.json handler is invoked after the key update button is pressed
 * and handles receiving the new ap key entered by the user
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_key_save_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "/key.json requested");

	size_t len_key  = 0;
	char   *key_str = NULL;

	// Get Key header
	len_key = httpd_req_get_hdr_value_len(req, "my-key") + 1;
	if (len_key >= 8 && len_key <32)
	{
		key_str = malloc(len_key);
		if (httpd_req_get_hdr_value_str(req, "my-key", key_str, len_key) == ESP_OK)
		{
			ESP_LOGI(TAG, "http_server_key_save_json_handler: Found header => my-key: %s", key_str);
		}
	}

	else
	{
		ESP_LOGI(TAG, " access point key should be between 8 and 32 characters\n");
		return ESP_FAIL;
	}

	// Update the AP key and let the wifi application know
	memset(roomsense_iq_shared.ap_key, 0x00, sizeof(roomsense_iq_shared.ap_key));
	memcpy(roomsense_iq_shared.ap_key, key_str, sizeof(roomsense_iq_shared.ap_key));

	wifi_app_send_message(WIFI_APP_MSG_UPDATE_AP_KEY);

	free(key_str);

	return ESP_OK;
}

static esp_err_t http_server_key_save_status_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "/key_save_status requested");

	char statusJSON[100];

	sprintf(statusJSON, "{\"ap_save_status\":%d}", g_ap_save_status);

	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, statusJSON, strlen(statusJSON));

	return ESP_OK;
}

/**
 * wifiConnect.json handler is invoked after the connect button is pressed
 * and handles receiving the SSID and password entered by the user
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_wifi_connect_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "/wifiConnect.json requested");

	size_t len_ssid = 0, len_pass = 0;
	char *ssid_str = NULL, *pass_str = NULL;

	// Get SSID header
	len_ssid = httpd_req_get_hdr_value_len(req, "my-connect-ssid") + 1;
	if (len_ssid > 1)
	{
		ssid_str = malloc(len_ssid);
		if (httpd_req_get_hdr_value_str(req, "my-connect-ssid", ssid_str, len_ssid) == ESP_OK)
		{
			ESP_LOGI(TAG, "http_server_wifi_connect_json_handler: Found header => my-connect-ssid: %s", ssid_str);
		}
	}

	// Get Password header
	len_pass = httpd_req_get_hdr_value_len(req, "my-connect-pwd") + 1;
	if (len_pass > 1)
	{
		pass_str = malloc(len_pass);
		if (httpd_req_get_hdr_value_str(req, "my-connect-pwd", pass_str, len_pass) == ESP_OK)
		{
			ESP_LOGI(TAG, "http_server_wifi_connect_json_handler: Found header => my-connect-pwd: %s", pass_str);
		}
	}

	// Update the Wifi networks configuration and let the wifi application know
	wifi_config_t* wifi_config = wifi_app_get_wifi_config();
	memset(wifi_config, 0x00, sizeof(wifi_config_t));
	memcpy(wifi_config->sta.ssid, ssid_str, len_ssid);
	memcpy(wifi_config->sta.password, pass_str, len_pass);
	wifi_app_send_message(WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER);

	free(ssid_str);
	free(pass_str);

	return ESP_OK;
}

/**
 * MQTTConnect.json handler is invoked after the submit button is pressed
 * and handles receiving the username and password entered by the user
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
mqtt_credentials_t* mqtt_credentials ;
static esp_err_t http_server_mqtt_connect_json_handler(httpd_req_t *req)
{

	ESP_LOGI(TAG, "/MQTTConnect.json requested");

	size_t len_host = 0, len_port = 0, len_username = 0, len_pass = 0;
	char *host_str = NULL, *port_str = NULL, *username_str = NULL, *pass_str = NULL;

	// Get host header
	len_host = httpd_req_get_hdr_value_len(req, "my-connect-host") + 1;
	if (len_host > 1)
	{
		host_str = malloc(len_host);
		if (httpd_req_get_hdr_value_str(req, "my-connect-host", host_str, len_host) == ESP_OK)
		{
			ESP_LOGI(TAG, "http_server_mqtt_connect_json_handler: Found header => my-connect-host: %s", host_str);
		}
	}

	// Get port header
	len_port = httpd_req_get_hdr_value_len(req, "my-connect-port") + 1;
	if (len_port > 1)
	{
		port_str = malloc(len_port);
		if (httpd_req_get_hdr_value_str(req, "my-connect-port", port_str, len_port) == ESP_OK)
		{
			ESP_LOGI(TAG, "http_server_mqtt_connect_json_handler: Found header => my-connect-port: %s", port_str);
		}
	}

	// Get username header
	len_username = httpd_req_get_hdr_value_len(req, "my-connect-username") + 1;
	if (len_username > 1)
	{
		username_str = malloc(len_username);
		if (httpd_req_get_hdr_value_str(req, "my-connect-username", username_str, len_username) == ESP_OK)
		{
			ESP_LOGI(TAG, "http_server_mqtt_connect_json_handler: Found header => my-connect-username: %s", username_str);
		}
	}

	// Get Password header
	len_pass = httpd_req_get_hdr_value_len(req, "my-connect-mqtt-pwd") + 1;
	if (len_pass > 1)
	{
		pass_str = malloc(len_pass);
		if (httpd_req_get_hdr_value_str(req, "my-connect-mqtt-pwd", pass_str, len_pass) == ESP_OK)
		{
			ESP_LOGI(TAG, "http_server_mqtt_connect_json_handler: Found header => my-connect-mqtt-pwd: %s", pass_str);
		}
	}

	// Update the MQTT credetials and let the mqtt application know

	mqtt_credentials = get_mqtt_credentials();
	memset(mqtt_credentials, 0x00, sizeof(mqtt_credentials_t));
	memcpy(mqtt_credentials->mqtt_host, host_str, len_host);
	memcpy(mqtt_credentials->mqtt_port, port_str, len_port);
	memcpy(mqtt_credentials->mqtt_username, username_str, len_username);
	memcpy(mqtt_credentials->mqtt_password, pass_str, len_pass);

	mqtt_app_send_message(MQTT_APP_MSG_CONNECTING_FROM_HTTP_SERVER);

	free(host_str);
	free(port_str);
	free(username_str);
	free(pass_str);

	return ESP_OK;
}

/**
 * MqttConnectStatus handler updates the connection status for the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_mqtt_connect_status_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "/MQTTConnectStatus requested");

	char statusJSON[100];

	sprintf(statusJSON, "{\"mqtt_connect_status\":%d}", g_mqtt_connect_status);

	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, statusJSON, strlen(statusJSON));

	return ESP_OK;
}

/**
 * mqtt_connect_info.json handler updates the web page with connection information.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_get_mqtt_connect_info_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "/mqtt_connect_info.json requested");

	char mqttInfoJSON[200];
	char username_mqtt[MAX_USERNAME_LENGTH];
    char host_mqtt[MAX_MQTT_HOST_LENGTH];
    char port_mqtt[MAX_MQTT_PORT_LENGTH];

	memset(mqttInfoJSON, 0, sizeof(mqttInfoJSON));


	if (g_mqtt_connect_status == HTTP_MQTT_STATUS_CONNECT_SUCCESS)
	{
		mqtt_credentials_t* mqtt_credentials = get_mqtt_credentials();
		strcpy(username_mqtt, mqtt_credentials->mqtt_username);
		strcpy(host_mqtt, mqtt_credentials->mqtt_host);
		strcpy(port_mqtt, mqtt_credentials->mqtt_port);

		sprintf(mqttInfoJSON, "{\"user\":\"%s\",\"host_ip\":\"%s\",\"port\":\"%s\"}", username_mqtt, host_mqtt, port_mqtt);
	}

	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, mqttInfoJSON, strlen(mqttInfoJSON));

	g_ws_pause = false;

	return ESP_OK;
}

/**
 * wifiConnectStatus handler updates the connection status for the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_wifi_connect_status_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "/wifiConnectStatus requested");

	char statusJSON[100];

	sprintf(statusJSON, "{\"wifi_connect_status\":%d}", g_wifi_connect_status);

	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, statusJSON, strlen(statusJSON));

	return ESP_OK;
}

/**
 * wifiConnectInfo.json handler updates the web page with connection information.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_get_wifi_connect_info_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "/wifiConnectInfo.json requested");

	char ipInfoJSON[200];
	memset(ipInfoJSON, 0, sizeof(ipInfoJSON));

	char ip[IP4ADDR_STRLEN_MAX];
	char netmask[IP4ADDR_STRLEN_MAX];
	char gw[IP4ADDR_STRLEN_MAX];

	if (g_wifi_connect_status == HTTP_WIFI_STATUS_CONNECT_SUCCESS)
	{
		wifi_ap_record_t wifi_data;
		ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&wifi_data));
		char *ssid = (char*)wifi_data.ssid;

		esp_netif_ip_info_t ip_info;
		ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netif_sta, &ip_info));
		esp_ip4addr_ntoa(&ip_info.ip, ip, IP4ADDR_STRLEN_MAX);
		esp_ip4addr_ntoa(&ip_info.netmask, netmask, IP4ADDR_STRLEN_MAX);
		esp_ip4addr_ntoa(&ip_info.gw, gw, IP4ADDR_STRLEN_MAX);

		sprintf(ipInfoJSON, "{\"ip\":\"%s\",\"netmask\":\"%s\",\"gw\":\"%s\",\"ap\":\"%s\"}", ip, netmask, gw, ssid);
	}

	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, ipInfoJSON, strlen(ipInfoJSON));

	return ESP_OK;
}

/**
 * wifiDisconnect.json handler responds by sending a message to the Wifi application to disconnect.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_wifi_disconnect_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "wifiDisconect.json requested");

	wifi_app_send_message(WIFI_APP_MSG_USER_REQUESTED_STA_DISCONNECT);

	return ESP_OK;
}
esp_err_t http_server_device_location_handler(httpd_req_t*req){
	ESP_LOGI(TAG,"Get device location tag");

	char dbuf[128]={0};
	char ipType[32]={0};
	char loc[LOCATION_SIZE + 1] = {0};
	size_t loc_sz = LOCATION_SIZE;

	nvs_handle handle;
	esp_err_t esp_err;
	esp_err = nvs_open("net_config", NVS_READWRITE, &handle);
	esp_err = nvs_get_blob(handle, "dev_loc_tag", loc, &loc_sz);
	nvs_close(handle);
	ESP_LOGI("LTAG","location tag %s",loc);
	if (nvs_open("net_config", NVS_READONLY, &handle) == ESP_OK)
	{
		size_t len=16;
		nvs_get_blob(handle, "ipType", ipType, &len);
		nvs_close(handle);

	}
	sprintf(dbuf,"{\"device_location\":\"%s\", \"ipType\":\"%s\"}",loc,ipType);
	printf("sent -> %s\r\n",dbuf);
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, dbuf, strlen(dbuf));
	return ESP_OK;
}

esp_err_t http_server_settings_parameters_handler(httpd_req_t*req){

	char dbuf[128]={0};
	char ap_state[16]={0};
	char led_state[16]={0};
	nvs_handle handle;
	esp_err_t esp_err;

	if (nvs_open("net_config", NVS_READONLY, &handle) == ESP_OK)
	{
		size_t len=16;
		nvs_get_blob(handle, "AP_STATE", ap_state, &len);
		nvs_get_blob(handle, "LED_STATE", led_state, &len);
		nvs_close(handle);
	}
	sprintf(dbuf,"{\"AP_STATE\":\"%s\", \"LED_STATE\":\"%s\"}",ap_state,led_state);
	printf("sent -> %s\r\n",dbuf);
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, dbuf, strlen(dbuf));
	return ESP_OK;
}

esp_err_t http_server_set_device_location_handler(httpd_req_t *req) {
	ESP_LOGI(TAG, "Get device location tag");
	char content[100] = {0};
	size_t loc_sz = LOCATION_SIZE;

	int read_len = httpd_req_recv(req, content, MIN(req->content_len, sizeof(content) - 1));
	if (read_len <= 0)
	{
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read request payload");
		return ESP_FAIL;
	}

	cJSON *json = cJSON_Parse(content);
	if (json == NULL)
	{
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to parse JSON payload");
		return ESP_FAIL;
	}

	cJSON *device_location_json = cJSON_GetObjectItem(json, "device_location");
	if (device_location_json == NULL || !cJSON_IsString(device_location_json) || device_location_json->valuestring == NULL) // allow empty
	{
		cJSON_Delete(json);
		const char *response = "device_location is missing";
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, response);
		printf("%s\n", response);
		return ESP_FAIL;
	}

	printf("RECEIVED : %s\n", device_location_json->valuestring);

	if (strlen(device_location_json->valuestring) > loc_sz)
	{
		cJSON_Delete(json);
		const char *response = "device location tag exceeds max length";
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, response);
		printf("%s\n", response);
		return ESP_FAIL;
	}

	memset(&roomsense_iq_shared.location, 0x00, loc_sz + 1);
	strcpy(roomsense_iq_shared.location, device_location_json->valuestring);

	cJSON_Delete(json);

	nvs_handle handle;
	esp_err_t esp_err;
	esp_err = nvs_open("net_config", NVS_READWRITE, &handle);
	esp_err = nvs_set_blob(handle, "dev_loc_tag", roomsense_iq_shared.location, loc_sz);
	if (esp_err != ESP_OK)
	{
		nvs_close(handle);
		const char *response = "failed to update device location tag";
		httpd_resp_send(req, response, strlen(response));
		return esp_err;
	}

	esp_err = nvs_commit(handle);
	nvs_close(handle);

	const char *response = "device location tag updated";
	httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
	printf("%s\n", response);

	return ESP_OK;
}

esp_err_t http_server_network_config_handler(httpd_req_t *req)
{
    char content[100] = {0};

    int read_len = httpd_req_recv(req, content, MIN(req->content_len, sizeof(content) - 1));
    if (read_len <= 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read request payload");
        return ESP_FAIL;
    }

    cJSON *json = cJSON_Parse(content);
    if (json == NULL)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to parse JSON payload");
        return ESP_FAIL;
    }
    cJSON *staticIpValue_json = cJSON_GetObjectItem(json, "staticIpValue");
    cJSON *ipType_json = cJSON_GetObjectItem(json, "ipType");
//    cJSON *mdnsUrlValue_json = cJSON_GetObjectItem(json, "mdnsUrlValue");
    if (staticIpValue_json == NULL )
    {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing network or password field");
        printf("error parsing json\r\n");
        return ESP_FAIL;
    }


    const char *staticIpValue = staticIpValue_json->valuestring;
//    const char *mdnsUrlValue = mdnsUrlValue_json->valuestring;
	if((strlen(staticIpValue_json->valuestring) > 1)){

		printf("RECEIVED : %s %s\n",staticIpValue,ipType_json->valuestring);
		nvs_handle handle;
		esp_err_t esp_err;
		ESP_LOGI(TAG, "app_nvs_save_sta_creds: Saving station mode credentials to flash");
		esp_err = nvs_open("net_config", NVS_READWRITE, &handle);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_sta_creds: Error (%s) opening NVS handle!\n", esp_err_to_name(esp_err));
			return esp_err;
		}

		/* Set Static IP */
		esp_err = nvs_set_blob(handle, "staticIpValue", staticIpValue_json->valuestring, 16);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_sta_creds: Error (%s) setting static ip to NVS!\n", esp_err_to_name(esp_err));
			return esp_err;
		}

		esp_err = nvs_set_blob(handle, "ipType", ipType_json->valuestring, 16);
		if (esp_err != ESP_OK)
		{
			printf("app_nvs_save_sta_creds: Error (%s) setting ipAddrType to NVS!\n", esp_err_to_name(esp_err));
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
		printf("app_nvs_save_netconfig_settings: returned ESP_OK\n");
		const char *response = "Network config updated successfully";
		httpd_resp_send(req, response, strlen(response));

	}
	else {
		const char *response = "Parameters length error";
		httpd_resp_send(req, response, strlen(response));
	}
    cJSON_Delete(json);
    return ESP_OK;
}

esp_err_t access_point_state_handler(httpd_req_t *req) {
	char content[100] = {0};

	int read_len = httpd_req_recv(req, content, MIN(req->content_len, sizeof(content) - 1));
	if (read_len <= 0)
	{
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read request payload");
		return ESP_FAIL;
	}

	printf("received %s\r\n",content);
    cJSON *json = cJSON_Parse(content);
    if (json == NULL)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to parse JSON payload");
        return ESP_FAIL;
    }
    cJSON *ap_state_json = cJSON_GetObjectItem(json, "AP_STATE");
    if (ap_state_json == NULL )
    {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing network or password field");
        printf("error parsing json\r\n");
        return ESP_FAIL;
    }



	if(strlen(ap_state_json->valuestring) > 0){
		printf("RECEIVED AP_STATE : %s\n",ap_state_json->valuestring);
		if(!strcmp(ap_state_json->valuestring,"true"))
		{
			roomsense_iq_shared.ap_enable = true;
		}
		else if(!strcmp(ap_state_json->valuestring,"false"))
		{
			roomsense_iq_shared.ap_enable = false;

		}
		nvs_handle handle;
		esp_err_t esp_err;
		esp_err = nvs_open("net_config", NVS_READWRITE, &handle);
		if (esp_err != ESP_OK)
		{
			return esp_err;
		}

		/* Set Static IP */
		esp_err = nvs_set_blob(handle, "AP_STATE", ap_state_json->valuestring, 8);
		if (esp_err != ESP_OK)
		{
			return esp_err;
		}
		// Commit credentials to NVS
		esp_err = nvs_commit(handle);
		if (esp_err != ESP_OK)
		{
			return esp_err;
		}
		nvs_close(handle);
		const char *response = "AP_STATE Updated";
		httpd_resp_send(req, response, strlen(response));
	}
	else {
		const char *response = "Parameters length error";
		httpd_resp_send(req, response, strlen(response));
	}
    cJSON_Delete(json);
    wifi_app_send_message(WIFI_APP_MSG_UPDATE_AP_ENABLE);
    return ESP_OK;
}


esp_err_t led_state_handler(httpd_req_t *req) {
	char content[100] = {0};

	int read_len = httpd_req_recv(req, content, MIN(req->content_len, sizeof(content) - 1));
	if (read_len <= 0)
	{
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read request payload");
		return ESP_FAIL;
	}

    cJSON *json = cJSON_Parse(content);
    if (json == NULL)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to parse JSON payload");
        return ESP_FAIL;
    }
    cJSON *led_state_json = cJSON_GetObjectItem(json, "LED_STATE");
    if (led_state_json == NULL )
    {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing network or password field");
        printf("error parsing json\r\n");
        return ESP_FAIL;
    }


    const char *led_state = led_state_json->valuestring;
	if((strlen(led_state) > 1)){
		printf("RECEIVED LED_STATE : %s\n",led_state);

		if(!strcmp(led_state,"true"))
		{
			roomsense_iq_shared.led_enable = true;
			xEventGroupSetBits(led_event_group, RGB_LED_BIT); // unlock led task
			if (roomsense_iq_shared.climatesense_connection)
			{
				 gpio_set_level(PRESENCE_ENABLE_PIN, 1);
			}
		}
		else if(!strcmp(led_state,"false"))
		{
			roomsense_iq_shared.led_enable = false;
			xEventGroupSetBits(led_event_group, RGB_LED_BIT); // unlock led task
			if (roomsense_iq_shared.climatesense_connection)
			{
				 gpio_set_level(PRESENCE_ENABLE_PIN, 0);
			}
		}

		nvs_handle handle;
		esp_err_t esp_err;
		esp_err = nvs_open("net_config", NVS_READWRITE, &handle);
		if (esp_err != ESP_OK)
		{
			return esp_err;
		}
		/* Set Static IP */
		esp_err = nvs_set_blob(handle, "LED_STATE", led_state, 8);
		if (esp_err != ESP_OK)
		{
			return esp_err;
		}
		// Commit credentials to NVS
		esp_err = nvs_commit(handle);
		if (esp_err != ESP_OK)
		{
			return esp_err;
		}
		nvs_close(handle);
		const char *response = "LED_STATE Updated";
		httpd_resp_send(req, response, strlen(response));
	}
	else {
		const char *response = "Parameters length error";
		httpd_resp_send(req, response, strlen(response));
	}
    cJSON_Delete(json);
    return ESP_OK;
}

#include "esp_tls.h"

void onClose(httpd_handle_t hd, int sockfd) {
    esp_tls_t *tls = httpd_sess_get_transport_ctx(hd, sockfd);
    esp_tls_conn_delete(tls);
    struct linger linger;
    linger.l_onoff = 1;
    linger.l_linger = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_LINGER, &linger, sizeof(linger) == 0)){
         ESP_LOGI(TAG, "Set SO_LINGER success");
         close(sockfd); // Now call close will send RST to peer and free pcb immediately.
    }

    ESP_LOGI(TAG, "Closing Socket %d, Heap:%d", sockfd, esp_get_free_heap_size());
}

/**
 * Sets up the default httpd server configuration.
 * @return http server instance handle if successful, NULL otherwise.
 */
static httpd_handle_t http_server_configure(void)
{
	// Generate the default configuration
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();

	// Create HTTP server monitor task
	xTaskCreatePinnedToCore(&http_server_monitor, "http_server_monitor", HTTP_SERVER_MONITOR_STACK_SIZE, NULL, HTTP_SERVER_MONITOR_PRIORITY, &task_http_server_monitor, HTTP_SERVER_MONITOR_CORE_ID);

	// Create the message queue
	http_server_monitor_queue_handle = xQueueCreate(3, sizeof(http_server_queue_message_t));

	config.lru_purge_enable = true;
	config.max_open_sockets =6;
//	config.enable_so_linger = true;
//	config.linger_timeout = 1;

	// The core that the HTTP server will run on
	config.core_id = HTTP_SERVER_TASK_CORE_ID;

	// Adjust the default priority to 1 less than the wifi application task
	config.task_priority = HTTP_SERVER_TASK_PRIORITY;

	// Bump up the stack size (default is 4096)
	config.stack_size = HTTP_SERVER_TASK_STACK_SIZE;

	// Increase uri handlers
	config.max_uri_handlers = 42;

	// Increase the timeout limits
	config.recv_wait_timeout = 500;
	config.send_wait_timeout = 500;
	config.close_fn = onClose;
	ESP_LOGI(TAG, "http_server_configure: Starting server on port: '%d' with task priority: '%d'", config.server_port, config.task_priority);


	// Start the httpd server
	if (httpd_start(&http_server_handle, &config) == ESP_OK)
	{
		ESP_LOGI(TAG, "http_server_configure: Registering URI handlers");
		start_dashboard_server_component(http_server_handle);
		// register query handler
		httpd_uri_t jquery_js = {
				.uri = "/jquery-3.3.1.min.js",
				.method = HTTP_GET,
				.handler = http_server_jquery_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &jquery_js);

		// register index.html handler
		httpd_uri_t index_html = {
				.uri = "/",
				.method = HTTP_GET,
				.handler = http_server_index_html_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &index_html);

		// register app.css handler
		httpd_uri_t app_css = {
				.uri = "/app.css",
				.method = HTTP_GET,
				.handler = http_server_app_css_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &app_css);

		// register app.js handler
		httpd_uri_t app_js = {
				.uri = "/app.js",
				.method = HTTP_GET,
				.handler = http_server_app_js_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &app_js);

		// register favicon.ico handler
		httpd_uri_t favicon_ico = {
				.uri = "/favicon.ico",
				.method = HTTP_GET,
				.handler = http_server_favicon_ico_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &favicon_ico);

		// register icon.png handler
		httpd_uri_t icon_png = {
				.uri = "/icon.png",
				.method = HTTP_GET,
				.handler = http_server_icon_png_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &icon_png);

		// register OTAupdate handler
		httpd_uri_t OTA_update = {
				.uri = "/OTAupdate",
				.method = HTTP_POST,
				.handler = http_server_OTA_update_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &OTA_update);

		// register OTAstatus handler
		httpd_uri_t OTA_status = {
				.uri = "/OTAstatus",
				.method = HTTP_POST,
				.handler = http_server_OTA_status_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &OTA_status);

		// register wifiConnect.json handler
		httpd_uri_t wifi_connect_json = {
				.uri = "/wifiConnect.json",
				.method = HTTP_POST,
				.handler = http_server_wifi_connect_json_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &wifi_connect_json);

		// register wifiConnectStatus.json handler
		httpd_uri_t wifi_connect_status_json = {
				.uri = "/wifiConnectStatus",
				.method = HTTP_POST,
				.handler = http_server_wifi_connect_status_json_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &wifi_connect_status_json);

		// register wifiConnectInfo.json handler
		httpd_uri_t wifi_connect_info_json = {
				.uri = "/wifiConnectInfo.json",
				.method = HTTP_GET,
				.handler = http_server_get_wifi_connect_info_json_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &wifi_connect_info_json);

		// register wifiDisconnect.json handler
		httpd_uri_t wifi_disconnect_json = {
				.uri = "/wifiDisconnect.json",
				.method = HTTP_DELETE,
				.handler = http_server_wifi_disconnect_json_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &wifi_disconnect_json);

		// register MqttConnect.json handler
		httpd_uri_t mqtt_connect_json = {
				.uri = "/mqtt_connect.json",
				.method = HTTP_POST,
				.handler = http_server_mqtt_connect_json_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &mqtt_connect_json);

		// register MqttConnectStatus.json handler
		httpd_uri_t mqtt_connect_status_json = {
				.uri = "/mqtt_connect_status",
				.method = HTTP_POST,
				.handler = http_server_mqtt_connect_status_json_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &mqtt_connect_status_json);

		// register mqtt_connect_info.json handler
		httpd_uri_t mqtt_connect_info_json = {
				.uri = "/mqtt_connect_info.json",
				.method = HTTP_GET,
				.handler = http_server_get_mqtt_connect_info_json_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &mqtt_connect_info_json);

		httpd_uri_t device_location_handle = {
						.uri = "/get_dev_loc.json",
						.method = HTTP_GET,
						.handler = http_server_device_location_handler,
						.user_ctx = NULL
				};
		httpd_register_uri_handler(http_server_handle, &device_location_handle);

		httpd_uri_t set_device_location_handle = {
						.uri = "/set_dev_loc.json",
						.method = HTTP_POST,
						.handler = http_server_set_device_location_handler,
						.user_ctx = NULL
				};
		httpd_register_uri_handler(http_server_handle, &set_device_location_handle);


		httpd_uri_t network_config_handle = {
						.uri = "/network_config.json",
						.method = HTTP_POST,
						.handler = http_server_network_config_handler,
						.user_ctx = NULL
				};
		httpd_register_uri_handler(http_server_handle, &network_config_handle);

		httpd_uri_t access_point_state_handle = {
						.uri = "/AP_STATE.json",
						.method = HTTP_POST,
						.handler = access_point_state_handler,
						.user_ctx = NULL
				};
		httpd_register_uri_handler(http_server_handle, &access_point_state_handle);

		httpd_uri_t led_state_handle = {
						.uri = "/LED_STATE.json",
						.method = HTTP_POST,
						.handler = led_state_handler,
						.user_ctx = NULL
				};
		httpd_register_uri_handler(http_server_handle, &led_state_handle);

		httpd_uri_t settings_parameters_handle = {
						.uri = "/settings_parameters.json",
						.method = HTTP_GET,
						.handler = http_server_settings_parameters_handler,
						.user_ctx = NULL
				};
		httpd_register_uri_handler(http_server_handle, &settings_parameters_handle);

		httpd_uri_t key_save_jason = {
				.uri = "/key.json",
				.method = HTTP_POST,
				.handler = http_server_key_save_json_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &key_save_jason);


		httpd_uri_t key_save_status_json = {
				.uri = "/apKeyStatus",
				.method = HTTP_POST,
				.handler = http_server_key_save_status_json_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &key_save_status_json);



		return http_server_handle;
	}

	return NULL;
}

void http_server_start(void)
{
	if (http_server_handle == NULL)
	{
		http_server_handle = http_server_configure();
	}
}

void http_server_stop(void)
{
	if (http_server_handle)
	{
		httpd_stop(http_server_handle);
		ESP_LOGI(TAG, "http_server_stop: stopping HTTP server");
		http_server_handle = NULL;
	}
	if (task_http_server_monitor)
	{
		vTaskDelete(task_http_server_monitor);
		ESP_LOGI(TAG, "http_server_stop: stopping HTTP server monitor");
		task_http_server_monitor = NULL;
	}
}

BaseType_t http_server_monitor_send_message(http_server_message_e msgID)
{
	http_server_queue_message_t msg;
	msg.msgID = msgID;
	return xQueueSend(http_server_monitor_queue_handle, &msg, portMAX_DELAY);
}

void http_server_fw_update_reset_callback(void *arg)
{
	ESP_LOGI(TAG, "http_server_fw_update_reset_callback: Timer timed-out, restarting the device");
	esp_restart();
}
