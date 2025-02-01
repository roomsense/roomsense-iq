/*
 * doppler-iq.c
 *
 *  Created on: Sep. 17, 2022
 *      Author: sinam
 */

#include <string.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_sntp.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include <ld2410.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include "tasks_common.h"
#include "esp_timer.h"
#include "../ha_mqtt/include/ha_mqtt.h"

#include "../direction-detection/include/direction-detection.h"

static const char *TAG = "ld2410";


static int8_t  baseline_threshold_micro[9];
static int8_t  baseline_threshold_macro[9];

uint16_t raw_detected_distance;

SemaphoreHandle_t xMutex_g_ld2410_data = NULL;
SemaphoreHandle_t baseline_semaphore = NULL;
SemaphoreHandle_t blindspot_semaphore = NULL;

// Queue handle used to manipulate the main queue of events
static QueueHandle_t ld2410_app_queue_handle;
extern QueueHandle_t distance_queue_handle;

static uint8_t ld2410_header[4] =
{ 0xFD, 0xFC, 0xFB, 0xFA };
static uint8_t ld2410_tail[4] =
{ 0x04, 0x03, 0x02, 0x01 };

#define BIN_NUM 9
#define MAX_ATTEMPTS 5

#define BASELINE_CALIBRATION_DURATION_SECONDS 10

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

enum target_status
{
	NoTarget = 0x00, Moving = 0x01, Motionless = 0x02, Moving_Motionless = 0x03
};

// Function to initialize the moving average filter
void moving_average_init(MovingAverageFilter *filter, uint16_t window_size)
{
	filter->buffer = (uint16_t*) calloc(window_size, sizeof(uint16_t));
	filter->window_size = window_size;
	filter->index = 0;
	filter->sum = 0;
}

// Function to update the moving average filter with a new data point and return the filtered output
uint16_t moving_average_update(MovingAverageFilter *filter, uint16_t new_data)
{

	int delta;
	static uint16_t ave_data = 0;

	// Subtract the oldest data point from the sum
	filter->sum -= filter->buffer[filter->index];

	delta = new_data - ave_data;
	if (delta > 10)
	{

		new_data = ave_data + 10;
	}

	if (delta < -10)
	{

		new_data = ave_data - 10;
	}

	// Add the new data point to the sum
	filter->sum += new_data;

	// Update the buffer with the new data point
	filter->buffer[filter->index] = new_data;

	// Increment the index circularly
	filter->index = (filter->index + 1) % filter->window_size;

	ave_data = filter->sum / filter->window_size;

	// Calculate and return the moving average
	return ave_data;
}

// Function to free the memory used by the moving average filter
void moving_average_cleanup(MovingAverageFilter *filter)
{
	free(filter->buffer);
}

esp_err_t ld2410_init(ld2410_dev_t *dev, uart_port_t uart_port, gpio_num_t tx_gpio, gpio_num_t rx_gpio)
{

	CHECK_ARG(dev);
	uart_config_t uart_config =
	{ .baud_rate = 256000, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
			.source_clk = UART_SCLK_APB,
#endif
			};
	CHECK(uart_driver_install(uart_port, LD2410_SERIAL_BUF_LEN * 2, 0, 0, NULL, 0));
	CHECK(uart_param_config(uart_port, &uart_config));
	CHECK(uart_set_pin(uart_port, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	dev->uart_port = uart_port;
	// buffer for the incoming data
	dev->buf = malloc(LD2410_SERIAL_BUF_LEN);
	if (!dev->buf)
		return ESP_ERR_NO_MEM;
	dev->last_value = -1;
	dev->last_ts = esp_timer_get_time();

	return ESP_OK;
}

esp_err_t ld2410_connect(ld2410_dev_t *dev)
{
	esp_err_t ack = 0;
	ack = set_config_mode(dev);
	if (ack != ESP_OK)
		return ack;
	vTaskDelay((150 - LD2410_SERIAL_RX_TIMEOUT_MS) / portTICK_PERIOD_MS);
	ack = set_config_mode(dev);
	if (ack != ESP_OK)
		return ack;
	ack = blank_command(dev);
	if (ack != ESP_OK)
		return ack;

	return ack;
}

esp_err_t ld2410_set_config(ld2410_dev_t *dev, ld2410_config_t *ld2410_config)
{

	uint8_t value[30];
	esp_err_t ack = 0;
	value[0] = ld2410_config->max_macro_range / 75;
	value[1] = ld2410_config->max_micro_range / 75;
	value[2] = (ld2410_config->absence_time_out) & 0xFF;
	value[3] = (ld2410_config->absence_time_out) >> 8;

	ack = set_config_mode(dev);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() set config mode failed.");
		return ack;
	}
	//vTaskDelay((150 - LD2410_SERIAL_RX_TIMEOUT_MS) / portTICK_PERIOD_MS);
	vTaskDelay(153 / portTICK_PERIOD_MS);

	ack = set_config_mode(dev);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() set config mode failed.");
		return ack;
	}
	vTaskDelay(553 / portTICK_PERIOD_MS);

	ack = ld2410_send_command(dev, SET_RANGE, value, 0x14, NULL); //set max range
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() SET_RANGE failed.");
		return ack;
	}
	vTaskDelay(6 / portTICK_PERIOD_MS);
	value[0] = ld2410_config->macro_threshold[0];
	value[1] = ld2410_config->micro_threshold[0];
	ack = ld2410_send_command(dev, SET_BIN_SENSITIVITY, value, 0x14, 0);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() SET_BIN_SENSITIVITY bin0 failed.");
		return ack;
	}
	vTaskDelay(6 / portTICK_PERIOD_MS);
	value[0] = ld2410_config->macro_threshold[1];
	value[1] = ld2410_config->micro_threshold[1];
	ack = ld2410_send_command(dev, SET_BIN_SENSITIVITY, value, 0x14, 1);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() SET_BIN_SENSITIVITY bin1 failed.");
		return ack;
	}
	vTaskDelay(6 / portTICK_PERIOD_MS);
	value[0] = ld2410_config->macro_threshold[2];
	value[1] = ld2410_config->micro_threshold[2];
	ack = ld2410_send_command(dev, SET_BIN_SENSITIVITY, value, 0x14, 2);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() SET_BIN_SENSITIVITY bin2 failed.");
		return ack;
	}

	vTaskDelay(6 / portTICK_PERIOD_MS);
	value[0] = ld2410_config->macro_threshold[3];
	value[1] = ld2410_config->micro_threshold[3];
	ack = ld2410_send_command(dev, SET_BIN_SENSITIVITY, value, 0x14, 3);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() SET_BIN_SENSITIVITY bin3 failed.");
		return ack;
	}

	vTaskDelay(6 / portTICK_PERIOD_MS);
	value[0] = ld2410_config->macro_threshold[4];
	value[1] = ld2410_config->micro_threshold[4];
	ack = ld2410_send_command(dev, SET_BIN_SENSITIVITY, value, 0x14, 4);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() SET_BIN_SENSITIVITY bin4 failed.");
		return ack;
	}

	vTaskDelay(6 / portTICK_PERIOD_MS);
	value[0] = ld2410_config->macro_threshold[5];
	value[1] = ld2410_config->micro_threshold[5];
	ack = ld2410_send_command(dev, SET_BIN_SENSITIVITY, value, 0x14, 5);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() SET_BIN_SENSITIVITY bin5 failed.");
		return ack;
	}
	vTaskDelay(6 / portTICK_PERIOD_MS);
	value[0] = ld2410_config->macro_threshold[6];
	value[1] = ld2410_config->micro_threshold[6];
	ack = ld2410_send_command(dev, SET_BIN_SENSITIVITY, value, 0x14, 6);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() SET_BIN_SENSITIVITY bin6 failed.");
		return ack;
	}

	vTaskDelay(6 / portTICK_PERIOD_MS);
	value[0] = ld2410_config->macro_threshold[7];
	value[1] = ld2410_config->micro_threshold[7];
	ack = ld2410_send_command(dev, SET_BIN_SENSITIVITY, value, 0x14, 7);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() SET_BIN_SENSITIVITY bin7 failed.");
		return ack;
	}

	vTaskDelay(6 / portTICK_PERIOD_MS);
	value[0] = ld2410_config->macro_threshold[8];
	value[1] = ld2410_config->micro_threshold[8];
	ack = ld2410_send_command(dev, SET_BIN_SENSITIVITY, value, 0x14, 8);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() SET_BIN_SENSITIVITY bin8 failed.");
		return ack;
	}

	ack = end_config_mode(dev);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "ld2410_set_config() end_config_mode() failed.");
		return ack;
	}
	return ack;
}

esp_err_t ld2410_start(ld2410_dev_t *dev, uint8_t mode)
{

	esp_err_t ack = 0;

	ack = set_config_mode(dev);
	if (ack != ESP_OK)
		return ack;
	vTaskDelay(150 / portTICK_PERIOD_MS);
	ack = set_config_mode(dev);
	if (ack != ESP_OK)
		return ack;
	vTaskDelay(550 / portTICK_PERIOD_MS);

	//set the mode to NORMAL OR ENGINEERING
	ack = ld2410_send_command(dev, mode, NULL, 0x02, NULL);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "Failed to set the mode.");
		return ack;
	}

	ack = end_config_mode(dev);
	if (ack != ESP_OK)
		return ack;
	return ack;
}

esp_err_t set_config_mode(ld2410_dev_t *dev)
{
	uint8_t value[2] =
	{ 0x02, 0x00 };
	esp_err_t ack;
	ack = ld2410_send_command(dev, ENABLE_CONFIG, value, 0x04, NULL);
	return ack;
}

esp_err_t end_config_mode(ld2410_dev_t *dev)
{
	esp_err_t ack;
	ack = ld2410_send_command(dev, END_CONFIG, NULL, 0x02, NULL);
	return ack;
}

esp_err_t reboot_module(ld2410_dev_t *dev)
{
	esp_err_t ack;
	ack = ld2410_send_command(dev, REBOOT_MODULE, NULL, 0x02, NULL);
	return ack;
}

esp_err_t blank_command(ld2410_dev_t *dev)
{
	esp_err_t ack;
	ack = ld2410_send_command(dev, BLANK_CMD, NULL, 0x02, NULL);
	return ack;
}

esp_err_t factory_reset(ld2410_dev_t *dev)
{
	esp_err_t ack;

	roomsense_iq_shared.ld2410_config_shared.absence_time_out = 20;
	roomsense_iq_shared.ld2410_config_shared.max_macro_range = 600;
	roomsense_iq_shared.ld2410_config_shared.max_micro_range = 600;

	roomsense_iq_shared.ld2410_config_shared.macro_threshold[0] = 35;
	roomsense_iq_shared.ld2410_config_shared.macro_threshold[1] = 25;
	roomsense_iq_shared.ld2410_config_shared.macro_threshold[2] = 20;
	roomsense_iq_shared.ld2410_config_shared.macro_threshold[3] = 15;
	roomsense_iq_shared.ld2410_config_shared.macro_threshold[4] = 15;
	roomsense_iq_shared.ld2410_config_shared.macro_threshold[5] = 15;
	roomsense_iq_shared.ld2410_config_shared.macro_threshold[6] = 15;
	roomsense_iq_shared.ld2410_config_shared.macro_threshold[7] = 15;
	roomsense_iq_shared.ld2410_config_shared.macro_threshold[8] = 15;

	roomsense_iq_shared.ld2410_config_shared.micro_threshold[0] = 25;
	roomsense_iq_shared.ld2410_config_shared.micro_threshold[1] = 15;
	roomsense_iq_shared.ld2410_config_shared.micro_threshold[2] = 10;
	roomsense_iq_shared.ld2410_config_shared.micro_threshold[3] = 10;
	roomsense_iq_shared.ld2410_config_shared.micro_threshold[4] = 10;
	roomsense_iq_shared.ld2410_config_shared.micro_threshold[5] = 10;
	roomsense_iq_shared.ld2410_config_shared.micro_threshold[6] = 10;
	roomsense_iq_shared.ld2410_config_shared.micro_threshold[7] = 10;
	roomsense_iq_shared.ld2410_config_shared.micro_threshold[8] = 10;

	for (int i = 0; i < 9; i++) //update baseline static variables
	{
	   baseline_threshold_micro[i] = roomsense_iq_shared.ld2410_config_shared.micro_threshold[i];
	   baseline_threshold_macro[i] = roomsense_iq_shared.ld2410_config_shared.macro_threshold[i];
	}

	ack = ld2410_set_config(dev, &roomsense_iq_shared.ld2410_config_shared);

	if (ESP_OK != ack)
	{
		ESP_LOGE(TAG, "LD241 Factory reset error.");
	}
	else
	{
		ESP_LOGI(TAG, "LD241 Factory reset successfully.");
	}

	return ack;
}

esp_err_t ld2410_send_command(ld2410_dev_t *dev, uint16_t command_word, uint8_t *command_value, uint16_t commandvalue_len, uint8_t bin_num)
{

	esp_err_t ack = 0;
	uint8_t i = 0;
	uint8_t state;
	uint8_t txBuffer[LD2410_SERIAL_TX_BYTES];
	uint16_t len = 0;
	int length = 0;

	esp_timer_handle_t timer = NULL;
	uint64_t start_time, end_time;
	uint64_t start_time_us, current_time_us;
	uint32_t elapsed_time_ms = 0;
	uint32_t timeout_ms = 1000; // Set the timeout to 1 second
	bool timeout_flag = false;

	CHECK_ARG(dev && dev->buf);

	state = SEND_STATE;

	while (state != IDLE_STATE)
	{
		switch (state)
		{
		case SEND_STATE:
			if (command_value != NULL)
			{
				len = commandvalue_len + 10; ////packet size = 4-byte header + 4byte tail + 2byte length +  commandvalue_len
			}
			else
			{
				len = 12;
			}
			txBuffer[0] = 0xFD;
			txBuffer[1] = 0xFC;
			txBuffer[2] = 0xFB;
			txBuffer[3] = 0xFA;
			if (command_value != NULL)
			{
				txBuffer[4] = (uint8_t) (commandvalue_len & 0xFF);
				txBuffer[5] = (uint8_t) (commandvalue_len >> 8);
			}
			else
			{
				txBuffer[4] = 0x02;
				txBuffer[5] = 0x00;
			}
			txBuffer[6] = (uint8_t) (command_word & 0xFF);
			txBuffer[7] = (uint8_t) (command_word >> 8);

			if (command_word == ENABLE_CONFIG)
			{
				for (i = 0; i < commandvalue_len; i++)
				{
					txBuffer[i + 8] = command_value[i];
				}
				txBuffer[6 + commandvalue_len] = 0x04;
				txBuffer[7 + commandvalue_len] = 0x03;
				txBuffer[8 + commandvalue_len] = 0x02;
				txBuffer[9 + commandvalue_len] = 0x01;
			}
			else if (command_word == SET_RANGE)
			{
				txBuffer[8] = 0x00; //macro range parameter word
				txBuffer[9] = 0x00;
				txBuffer[10] = command_value[0]; //macro range value
				txBuffer[11] = 0x00;
				txBuffer[12] = 0x00;
				txBuffer[13] = 0x00;

				txBuffer[14] = 0x01; //micro range parameter word
				txBuffer[15] = 0x00;
				txBuffer[16] = command_value[1]; //micro range 2-8
				txBuffer[17] = 0x00;
				txBuffer[18] = 0x00;
				txBuffer[19] = 0x00;

				txBuffer[20] = 0x02; // Absence timeout parameter word
				txBuffer[21] = 0x00;
				txBuffer[22] = command_value[2]; //timeout value 0-65535 low byte
				txBuffer[23] = command_value[3]; //timeout value 0-65535 high byte
				txBuffer[24] = 0x00;
				txBuffer[25] = 0x00;

				txBuffer[26] = 0x04;
				txBuffer[27] = 0x03;
				txBuffer[28] = 0x02;
				txBuffer[29] = 0x01;

			}
			else if (command_word == SET_BIN_SENSITIVITY)
			{
				txBuffer[8] = 0x00; //macro range parameter word
				txBuffer[9] = 0x00;
				txBuffer[10] = bin_num; // set bin number
				txBuffer[11] = 0x00;
				txBuffer[12] = 0x00;
				txBuffer[13] = 0x00;

				txBuffer[14] = 0x01; //macro threshold parameter word
				txBuffer[15] = 0x00;
				txBuffer[16] = command_value[0]; //macro threshold
				txBuffer[17] = 0x00;
				txBuffer[18] = 0x00;
				txBuffer[19] = 0x00;

				txBuffer[20] = 0x02; // micro threshold parameter word
				txBuffer[21] = 0x00;
				txBuffer[22] = command_value[1]; //micro threshold
				txBuffer[23] = 0x00;
				txBuffer[24] = 0x00;
				txBuffer[25] = 0x00;

				txBuffer[26] = 0x04;
				txBuffer[27] = 0x03;
				txBuffer[28] = 0x02;
				txBuffer[29] = 0x01;
				//-----------------------------------------

			}
			else
			{
				txBuffer[len - 4] = 0x04;
				txBuffer[len - 3] = 0x03;
				txBuffer[len - 2] = 0x02;
				txBuffer[len - 1] = 0x01;
			}

#if HELPER_TARGET_IS_ESP32 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
			if (!uart_is_driver_installed(dev->uart_port))
				return ESP_ERR_INVALID_STATE;
#endif
			uart_flush(dev->uart_port);
			uart_write_bytes(dev->uart_port, (char*) txBuffer, len);
			memset(dev->buf, 0, LD2410_SERIAL_BUF_LEN);
			timeout_flag = false;
			state = RECEIVE_STATE;
			break;

		case RECEIVE_STATE:

			// Start the timer
			start_time_us = esp_timer_get_time();

			while (!length)
			{

				vTaskDelay(5 / portTICK_PERIOD_MS);
				ESP_ERROR_CHECK_WITHOUT_ABORT(uart_get_buffered_data_len(dev->uart_port, (size_t*) &length));
				current_time_us = esp_timer_get_time();
				elapsed_time_ms = (current_time_us - start_time_us) / 1000;
				if (elapsed_time_ms >= timeout_ms)
				{
					ESP_LOGE(TAG, "ld2410 Receive timeout");
					timeout_flag = true;
					break;
				}
			}

			//ESP_LOGI(TAG, "Elapsed time %d ms", elapsed_time_ms);

			memset(dev->buf, 0, LD2410_SERIAL_BUF_LEN);
			len = uart_read_bytes(dev->uart_port, dev->buf, length,
			LD2410_SERIAL_RX_TIMEOUT_MS / portTICK_PERIOD_MS);
			ack = ld2410_verify_ack(dev, command_word, commandvalue_len, len);
			state = IDLE_STATE;
			break;
		}
	}
	return (ack);
}

esp_err_t ld2410_get_data(ld2410_dev_t *dev, ld2410_data_t *ld2410_data, uint8_t mode)
{

	int len;
	int i;
	bool vacancy_flag = false;
	esp_err_t ret;

	// Clear receive buffer
	memset(dev->buf, 0, LD2410_SERIAL_BUF_LEN);
	len = uart_read_bytes(dev->uart_port, dev->buf, LD2410_SERIAL_RX_BYTES,
	LD2410_SERIAL_RX_TIMEOUT_MS / portTICK_PERIOD_MS);

	if (mode == NORMAL_MODE)
	{
		if (dev->buf[7] == HEAD && dev->buf[17] == TAIL && dev->buf[18] == CHECK_MSG)
		{

			ld2410_data->target_status = dev->buf[8];
			ld2410_data->macro_distance = (dev->buf[10] << 8) + dev->buf[9];
			ld2410_data->macro_level = dev->buf[11];
			ld2410_data->micro_distance = (dev->buf[13] << 8) + dev->buf[12];
			ld2410_data->micro_level = dev->buf[14];
			ld2410_data->detected_distance = (dev->buf[16] << 8) + dev->buf[15];
		}
		else
		{
			ESP_LOGI(TAG, "SLD2410 wrong format received in normal mode.");
			ret = ESP_FAIL;
		}
	}
	if (mode == ENGINEERING_MODE)
	{
		if (dev->buf[7] == HEAD && dev->buf[39] == TAIL && dev->buf[40] == CHECK_MSG)
		{

			fflush(stdout);
			ld2410_data->target_status = dev->buf[8];
			if (!(ld2410_data->target_status) && !vacancy_flag) {
			{
				vacancy_flag = true;
			}
	        } else if (ld2410_data->target_status) {
	        	vacancy_flag = false;  // Reset the flag when target_status is set to non-zero values
	        }
			ld2410_data->macro_distance = (dev->buf[10] << 8) + dev->buf[9];
			ld2410_data->macro_level = dev->buf[11];
			ld2410_data->micro_distance = (dev->buf[13] << 8) + dev->buf[12];
			ld2410_data->micro_level = dev->buf[14];
			ld2410_data->detected_distance = (dev->buf[16] << 8) + dev->buf[15];

			//ld2410_app_send_distance(ld2410_data->detected_distance);

			for (i = 0; i < BIN_NUM; i++)
			{
				ld2410_data->macro_bin[i] = dev->buf[19 + i];
				ld2410_data->micro_bin[i] = dev->buf[28 + i];
			}
			ret = ESP_OK;

		}
		else
		{
			ESP_LOGI(TAG, "LD2410 wrong format received in in ENG mode.");
			ret = ESP_FAIL;
		}
	}

	return (ret);
}

esp_err_t ld2410_get_config(ld2410_dev_t *dev, ld2410_config_t *ld2410_config)
{

	uint8_t txBuffer[12];
	uint16_t macro_distance, micro_distance, detected_distance;
	uint8_t macro_level, micro_level;
	int len;
	uint8_t state;
	uint8_t feature_index;
	uint8_t at_first_index, at_second_index, micro_index;
	enum target_status target_state;
	struct timeval tv_now;
	int i;
	uint8_t macro_profile[BIN_NUM];
	uint8_t micro_profile[BIN_NUM];
	esp_err_t ack = 0;

	CHECK_ARG(dev && dev->buf);

	ack = set_config_mode(dev);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "LD241 set config mode failed.");
		ld2410_config->valid = ESP_FAIL;
		return ld2410_config;
	}
	vTaskDelay(151 / portTICK_PERIOD_MS);

	ack = set_config_mode(dev);
	if (ack != ESP_OK)
	{
		ESP_LOGI(TAG, "LD241 set config mode failed.");
		ld2410_config->valid = ESP_FAIL;
		return ack;
	}
	vTaskDelay(544 / portTICK_PERIOD_MS);

	state = SEND_STATE;
	while (state != IDLE_STATE)
	{
		switch (state)
		{
		case SEND_STATE:
			txBuffer[0] = 0xFD;
			txBuffer[1] = 0xFC;
			txBuffer[2] = 0xFB;
			txBuffer[3] = 0xFA;
			txBuffer[4] = 0x02;
			txBuffer[5] = 0x00;
			txBuffer[6] = 0x61;
			txBuffer[7] = 0x00;
			txBuffer[8] = 0x04;
			txBuffer[9] = 0x03;
			txBuffer[10] = 0x02;
			txBuffer[11] = 0x01;
			uart_flush(dev->uart_port);
			uart_write_bytes(dev->uart_port, (char*) txBuffer, 12);
			memset(dev->buf, 0, LD2410_SERIAL_BUF_LEN);
			state = RECEIVE_STATE;
			break;
		case RECEIVE_STATE:

			memset(dev->buf, 0, LD2410_SERIAL_BUF_LEN);
			len = uart_read_bytes(dev->uart_port, dev->buf, LD2410_SERIAL_RX_BYTES, LD2410_SERIAL_RX_TIMEOUT_MS / portTICK_PERIOD_MS);

			//ESP_LOGI(TAG, "LD241 read config bytes %d", len);

			if (dev->buf[6] == 0x61 && dev->buf[7] == 0x01 && dev->buf[8] == 0x00 && dev->buf[9] == 0x00 && dev->buf[10] == 0xAA && dev->buf[11] == 0x08)
			{
				ESP_LOGI(TAG, "LD241 RECEIVE_STATE successfully.");
				ld2410_config->valid = ESP_OK;
				ld2410_config->max_macro_range = dev->buf[12] * 75;
				ld2410_config->max_micro_range = dev->buf[13] * 75;

				ld2410_config->macro_threshold[0] = dev->buf[14];
				ld2410_config->macro_threshold[1] = dev->buf[15];
				ld2410_config->macro_threshold[2] = dev->buf[16];
				ld2410_config->macro_threshold[3] = dev->buf[17];
				ld2410_config->macro_threshold[4] = dev->buf[18];
				ld2410_config->macro_threshold[5] = dev->buf[19];
				ld2410_config->macro_threshold[6] = dev->buf[20];
				ld2410_config->macro_threshold[7] = dev->buf[21];
				ld2410_config->macro_threshold[8] = dev->buf[22];

				ld2410_config->micro_threshold[0] = dev->buf[23];
				ld2410_config->micro_threshold[1] = dev->buf[24];
				ld2410_config->micro_threshold[2] = dev->buf[25];
				ld2410_config->micro_threshold[3] = dev->buf[26];
				ld2410_config->micro_threshold[4] = dev->buf[27];
				ld2410_config->micro_threshold[5] = dev->buf[28];
				ld2410_config->micro_threshold[6] = dev->buf[29];
				ld2410_config->micro_threshold[7] = dev->buf[30];
				ld2410_config->micro_threshold[8] = dev->buf[31];

				baseline_threshold_macro[0] = dev->buf[14];
				baseline_threshold_macro[1] = dev->buf[15];
				baseline_threshold_macro[2] = dev->buf[16];
				baseline_threshold_macro[3] = dev->buf[17];
				baseline_threshold_macro[4] = dev->buf[18];
				baseline_threshold_macro[5] = dev->buf[19];
				baseline_threshold_macro[6] = dev->buf[20];
				baseline_threshold_macro[7] = dev->buf[21];
				baseline_threshold_macro[8] = dev->buf[22];

				baseline_threshold_micro[0] = dev->buf[23];
				baseline_threshold_micro[1] = dev->buf[24];
				baseline_threshold_micro[2] = dev->buf[25];
				baseline_threshold_micro[3] = dev->buf[26];
				baseline_threshold_micro[4] = dev->buf[27];
				baseline_threshold_micro[5] = dev->buf[28];
				baseline_threshold_micro[6] = dev->buf[29];
				baseline_threshold_micro[7] = dev->buf[30];
				baseline_threshold_micro[8] = dev->buf[31];

				ld2410_config->absence_time_out = (dev->buf[33] << 8) + dev->buf[32];
				state = END_CONFIG_STATE;
			}
			else
			{
				ESP_LOGI(TAG, "LD241 read config wrong format.");
				ld2410_config->valid = ESP_FAIL;
				return ESP_FAIL;
			}

			break;
		case END_CONFIG_STATE:
			ESP_LOGI(TAG, "END_CONFIG_STATE.");
			ack = end_config_mode(dev);
			if (ack != ESP_OK)
			{
				ld2410_config->valid = ESP_FAIL;
				ESP_LOGI(TAG, "ld2410_get_config() end_config_mode() failed.");
				return ack;
			}
			state = IDLE_STATE;
			break;

		}
	}
	ESP_LOGI(TAG, "return ld2410_config ok");
	return ESP_OK;
}

esp_err_t ld2410_verify_ack(ld2410_dev_t *dev, uint16_t command_word, uint16_t commandvalue_len, uint16_t len)
{

	uint16_t rec_command_word;
	esp_err_t ack = 0;

	if (dev->buf[0] == ld2410_header[0] && dev->buf[1] == ld2410_header[1] && dev->buf[2] == ld2410_header[2] && dev->buf[3] == ld2410_header[3])
	{
		rec_command_word = dev->buf[6] + (dev->buf[7] << 8);
		if (rec_command_word == (command_word & 0x0100)) // command word check
			if (dev->buf[8] == 0x00 && dev->buf[9] == 0x00) //status check
				if (dev->buf[len - 1] == 0x01 && dev->buf[len - 2] == 0x02 && dev->buf[len - 3] == 0x03 && dev->buf[len - 4] == 0x04) //tail check
					ack = ESP_OK;
	}
	else
		ack = ESP_FAIL;
	return ack;
}

static void ld2410_task(void *pvParameters)
{

	ld2410_dev_t dev;
	int i;
	ld2410_app_queue_message_t msg;
	int attempts = 0;
	bool success = false;
	//ld2410_data_t *ld2410_data = NULL;
	esp_err_t ack;
	uint16_t movement_threshold;

	int rand_value;

	MovingAverageFilter filter;
	int window_size = 10; // You can adjust the window size as per your requirement
	moving_average_init(&filter, window_size);

	memset(&roomsense_iq_shared.ld2410_config_shared, 0x00, sizeof(ld2410_config_t));
	memset(&roomsense_iq_shared.ld2410_data_shared, 0x00, sizeof(ld2410_data_t));

	esp_log_level_set(TAG, ESP_LOG_INFO);

	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_add(NULL));
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_status(NULL));


	// Send first event message
	ld2410_app_send_message(LD2410_APP_MSG_INIT);

	for (;;)
	{
		ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_reset());
		if (xQueueReceive(ld2410_app_queue_handle, &msg, portMAX_DELAY))
		{
			if (roomsense_iq_shared.buttons_shared.g_button_get_config)
			{
				msg.msgID = LD2410_APP_MSG_GET_CONFIG;
				roomsense_iq_shared.buttons_shared.g_button_get_config = false;
				ESP_LOGI(TAG, "LD2410_APP_MSG_GET_CONFIG");

			}
			if (roomsense_iq_shared.buttons_shared.g_button_set_config)
			{
				msg.msgID = LD2410_APP_MSG_SET_CONFIG;
				roomsense_iq_shared.buttons_shared.g_button_set_config = false;
			}
			if (roomsense_iq_shared.buttons_shared.g_button_factory_reset)
			{
				msg.msgID = LD2410_APP_MSG_FACTORY_RESET;
				roomsense_iq_shared.buttons_shared.g_button_factory_reset = false;
			}

			switch (msg.msgID)
			{
			case LD2410_APP_MSG_INIT:
				ESP_LOGI(TAG, "LD2410 Init.");
				ESP_ERROR_CHECK_WITHOUT_ABORT(ld2410_init(&dev, UART_NUM_2, CONFIG_LD2410_UART_TX, CONFIG_LD2410_UART_RX));
				ld2410_app_send_message(LD2410_APP_MSG_CONNECT);
				break;

			case LD2410_APP_MSG_CONNECT:
				success = false;
				attempts = 0;
				while (attempts < MAX_ATTEMPTS && !success)
				{
					if (ESP_OK == ld2410_connect(&dev))
					{
						success = true;
					}
					else
					{
						vTaskDelay(100 / portTICK_PERIOD_MS);
						attempts++;
					}
				}
				if (!success)
				{
					ESP_LOGE(TAG, "ld2410_connect() failed after MAX_ATTEMPTS attempts");
					roomsense_iq_shared.alert_flag = true; //set the failing singal to overwrite other led signals.
					reboot_module(&dev);
				}
				vTaskDelay(200 / portTICK_PERIOD_MS);
				ld2410_app_send_message(LD2410_APP_MSG_GET_CONFIG);
				break;

			case LD2410_APP_MSG_START:
				success = false;
				attempts = 0;
				while (attempts < MAX_ATTEMPTS && !success)
				{
					if (ESP_OK == ld2410_connect(&dev))
					{
						success = true;
					}
					else
					{
						vTaskDelay(100 / portTICK_PERIOD_MS);
						attempts++;
					}
				}
				if (!success)
				{
					ESP_LOGE(TAG, "ld2410_connect() in start failed after %d attempts.", MAX_ATTEMPTS);
					reboot_module(&dev);
				}

				success = false;
				attempts = 0;
				while (attempts < MAX_ATTEMPTS && !success)
				{
					if (ESP_OK == ld2410_start(&dev, ENGINEERING_MODE))
					{
						success = true;
					}
					else
					{
						vTaskDelay(100 / portTICK_PERIOD_MS);
						attempts++;
					}
				}
				if (!success)
				{
					ESP_LOGE(TAG, "ld2410_start() failed after %d attempts", MAX_ATTEMPTS);
					reboot_module(&dev);
				}

				vTaskDelay(200 / portTICK_PERIOD_MS);
				ld2410_app_send_message(LD2410_APP_MSG_COLLECT_DATA);
				break;

			case LD2410_APP_MSG_COLLECT_DATA:
				success = false;
				attempts = 0;
				while (attempts < MAX_ATTEMPTS && !success)
				{
					if (ESP_OK == ld2410_get_data(&dev, &roomsense_iq_shared.ld2410_data_shared, ENGINEERING_MODE))
					{
						if(roomsense_iq_shared.ld2410_data_shared.target_status == 0 && !roomsense_iq_shared.climatesense_connection) // if vacancy detected reset the presence pin
						{
						   gpio_set_level(PRESENCE_ENABLE_PIN, 0);
						}

						success = true;
						if (xMutex_g_ld2410_data != NULL)
						{
							if (xSemaphoreTake(xMutex_g_ld2410_data, (TickType_t) 10) == pdTRUE)
							{
								//only if the room is occupied then update distance
								if (roomsense_iq_shared.ha_mqtt_shared.g_occupancy_status == OCCUPIED)
								{
									if (roomsense_iq_shared.ld2410_data_shared.detected_distance < 150 || roomsense_iq_shared.ld2410_data_shared.macro_distance < 150)
									{
										rand_value = rand() % DISTANCE_RESOLUTION;
										if (rand() % 2 == 0)
										{
											rand_value *= -1;
										}
										roomsense_iq_shared.ld2410_data_shared.detected_distance = 130 + rand_value;
										roomsense_iq_shared.ld2410_data_shared.macro_distance = 130 + rand_value;
									}

									//detected_distance is not accurate in distances lower than 190cm
									if (roomsense_iq_shared.ld2410_data_shared.detected_distance > 250)
									{
										raw_detected_distance = roomsense_iq_shared.ld2410_data_shared.detected_distance;
										roomsense_iq_shared.ld2410_data_shared.detected_distance = moving_average_update(&filter, raw_detected_distance);
										movement_threshold = 50;
									}
									else
									{
										raw_detected_distance = roomsense_iq_shared.ld2410_data_shared.macro_distance;
										roomsense_iq_shared.ld2410_data_shared.detected_distance = moving_average_update(&filter, raw_detected_distance);
										movement_threshold = 90;
									}
								}
								else
								{
									movement_threshold = 30;
								}

								for (i = 0; i < BIN_NUM; i++)
								{
									if (roomsense_iq_shared.ld2410_data_shared.macro_bin[i] > movement_threshold)
									{
										roomsense_iq_shared.direction_detection_shared.macro_movement = 1;
									}
								}
								xSemaphoreGive(xMutex_g_ld2410_data);
							}
						}
					}
					else
					{
						vTaskDelay(100 / portTICK_PERIOD_MS);
						attempts++;
					}
				}
				if (!success)
				{
					ESP_LOGE(TAG, "ld2410_get_data() failed after 5 attempts");
					reboot_module(&dev);
				}
				ld2410_app_send_message(LD2410_APP_MSG_COLLECT_DATA);
				break;

			case LD2410_APP_MSG_SET_CONFIG:

				success = false;
				attempts = 0;
				while (attempts < MAX_ATTEMPTS && !success)
				{
					if (ESP_OK == ld2410_set_config(&dev, &roomsense_iq_shared.ld2410_config_shared))
					{
						success = true;
					}
					else
					{
						vTaskDelay(100 / portTICK_PERIOD_MS);
						attempts++;
					}
				}
				if (!success)
				{
					ESP_LOGE(TAG, "ld2410_set_config() failed after %d attempts", MAX_ATTEMPTS);
					reboot_module(&dev);
				}

				ld2410_app_send_message(LD2410_APP_MSG_START);
				break;

			case LD2410_APP_MSG_GET_CONFIG:
				success = false;
				attempts = 0;
				while (attempts < MAX_ATTEMPTS && !success)
				{
					if (ESP_OK == ld2410_get_config(&dev, &roomsense_iq_shared.ld2410_config_shared))
					{
						success = true;
						roomsense_iq_shared.ha_mqtt_shared.g_macro_threshold_changed = 1;
						roomsense_iq_shared.ha_mqtt_shared.g_micro_threshold_changed = 1;
						roomsense_iq_shared.ha_mqtt_shared.g_max_macro_range_changed = 1;
						roomsense_iq_shared.ha_mqtt_shared.g_max_micro_range_changed = 1;
						roomsense_iq_shared.ha_mqtt_shared.g_timeout_changed = 1;
					}
					else
					{
						vTaskDelay(100 / portTICK_PERIOD_MS);
						attempts++;
					}
				}
				if (!success)
				{
					ESP_LOGE(TAG, "ld2410_get_config() failed after %d attempts", MAX_ATTEMPTS);
					reboot_module(&dev);
				}
				ld2410_app_send_message(LD2410_APP_MSG_START);
				break;

			case LD2410_APP_MSG_FACTORY_RESET:
				success = false;
				attempts = 0;
				while (attempts < MAX_ATTEMPTS && !success)
				{
					if (ESP_OK == factory_reset(&dev))
					{
						success = true;
						ESP_LOGI(TAG, "factory_reset() success-----------------");
					}
					else
					{
						vTaskDelay(100 / portTICK_PERIOD_MS);
						attempts++;
					}
				}
				if (!success)
				{
					ESP_LOGE(TAG, "factory_reset() failed after %d attempts", MAX_ATTEMPTS);
					reboot_module(&dev);
				}
				ld2410_app_send_message(LD2410_APP_MSG_GET_CONFIG);
				break;

			default:
				ESP_LOGI(TAG, "LD241 default state.");
				break;
			}
		}
	}
}
void ld2410_task_start()
{

	ESP_LOGI(TAG, "STARTING LD2410...");

	esp_log_level_set(TAG, ESP_LOG_INFO);

	xMutex_g_ld2410_data = xSemaphoreCreateMutex();
	if (xMutex_g_ld2410_data == NULL)
	{
		ESP_LOGI("Main", "Error: unable to create xMutex_g_ld2410_data");
	}
	else
	{
		ld2410_app_queue_handle = xQueueCreate(3, sizeof(ld2410_app_queue_message_t));
		xTaskCreatePinnedToCore(&ld2410_task, "ld2410_task", LD2410_TASK_STACK_SIZE, NULL, LD2410_TASK_PRIORITY, NULL, LD2410_TASK_CORE_ID);
		xTaskCreatePinnedToCore(&baseline_calibration_task, "baseline_calibration_task", 4 * 1024, NULL, 4, NULL, LD2410_TASK_CORE_ID);
		xTaskCreatePinnedToCore(&blindspot_calibration_task,"blindspot_calibration_task", 4 * 1024, NULL, 4, NULL, LD2410_TASK_CORE_ID);
	}
}

BaseType_t ld2410_app_send_message(ld2410_app_message_e msgID)
{
	ld2410_app_queue_message_t msg;
	msg.msgID = msgID;
	return xQueueSend(ld2410_app_queue_handle, &msg, portMAX_DELAY);
}

BaseType_t ld2410_app_send_distance(uint16_t distance)
{
	return xQueueSend(distance_queue_handle, &distance, portMAX_DELAY);
}


static void baseline_calibration_task(void *pvParameters)
{
	int i;
	uint8_t data_macro[9], data_micro[9];
	uint8_t max_values_macro[9] = { 0 };
	uint8_t max_values_micro[9] = { 0 };
	TickType_t xTicksToWait;
	TickType_t ticks;

	while (1)
	{
		if (baseline_semaphore != NULL)
		{
			if (xSemaphoreTake(baseline_semaphore, portMAX_DELAY))
			{
				roomsense_iq_shared.ld2410_config_shared.micro_threshold[0] = 0;

				// Calculate the number of ticks for the scanning duration
				xTicksToWait = BASELINE_CALIBRATION_DURATION_SECONDS * 1000 / portTICK_PERIOD_MS;

				// Loop for the specified duration
				for (ticks = 0; ticks < xTicksToWait; ticks += 100 / portTICK_PERIOD_MS)
				{
					for (int i = 0; i < 9; i++)
					{
						data_macro[i] = roomsense_iq_shared.ld2410_data_shared.macro_bin[i];
					}

					for (int i = 0; i < 9; i++)
					{
						if (data_macro[i] > max_values_macro[i])
						{
							max_values_macro[i] = data_macro[i];
						}
					}

					for (int i = 0; i < 9; i++)
					{
						data_micro[i] = roomsense_iq_shared.ld2410_data_shared.micro_bin[i];
					}

					for (int i = 0; i < 9; i++)
					{
						if (data_micro[i] > max_values_micro[i])
						{
							max_values_micro[i] = data_micro[i];
						}
					}

					vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay for 100 milliseconds
				}

				// Add 20% to the max values for threshold
				for (int i = 0; i < 9; i++)
				{
					roomsense_iq_shared.ld2410_config_shared.macro_threshold[i] = max_values_macro[i] + 5; //0.5 * max_values_macro[i] + max_values_macro[i];
					roomsense_iq_shared.ld2410_config_shared.micro_threshold[i] = max_values_micro[i] + 5;  //0.2 * max_values_micro[i] + max_values_micro[i];

					if(roomsense_iq_shared.ld2410_config_shared.macro_threshold[i] >= 100)
						roomsense_iq_shared.ld2410_config_shared.macro_threshold[i] = 100;

					if(roomsense_iq_shared.ld2410_config_shared.micro_threshold[i] >= 100)
						roomsense_iq_shared.ld2410_config_shared.micro_threshold[i] = 100;

					if(roomsense_iq_shared.ld2410_config_shared.macro_threshold[i] == 0)
						roomsense_iq_shared.ld2410_config_shared.macro_threshold[i] = 5;

					if(roomsense_iq_shared.ld2410_config_shared.micro_threshold[i] == 0)
						roomsense_iq_shared.ld2410_config_shared.micro_threshold[i] = 5;
				}
				for (int i = 0; i < 9; i++) //update baseline static variables
				{
				   baseline_threshold_micro[i] = roomsense_iq_shared.ld2410_config_shared.micro_threshold[i];
				   baseline_threshold_macro[i] = roomsense_iq_shared.ld2410_config_shared.macro_threshold[i];
				}

				//send to web-ui
				send_config_setting(roomsense_iq_shared.ld2410_config_shared.max_macro_range,
						            roomsense_iq_shared.ld2410_config_shared.max_micro_range,
						            roomsense_iq_shared.ld2410_config_shared.absence_time_out,
									roomsense_iq_shared.pir_sensor_shared.pir_sensitivity,
									roomsense_iq_shared.ha_mqtt_shared.g_bedsense_status,
									roomsense_iq_shared.ld2410_config_shared.macro_threshold,
									roomsense_iq_shared.ld2410_config_shared.micro_threshold);
                //write to ld2410
				roomsense_iq_shared.buttons_shared.g_button_set_config = true;
			}
		}
		else
		{
			ESP_LOGI(TAG, "Could not take baseline_semaphore");
		}
	}
}

static void blindspot_calibration_task(void *pvParameters)
{
	int i;
	uint8_t    data_macro[9], data_micro[9];
	int8_t    min_values_macro[9] = { 100, 100, 100, 100, 100, 100, 100, 100, 100 };
	int8_t    min_values_micro[9] = { 100, 100, 100, 100, 100, 100, 100, 100, 100 };
	TickType_t xTicksToWait;
	TickType_t ticks;

	while (1)
	{
		if (blindspot_semaphore != NULL)
		{
			if (xSemaphoreTake(blindspot_semaphore, portMAX_DELAY))
			{
				roomsense_iq_shared.ld2410_config_shared.micro_threshold[0] = 0;

				// Calculate the number of ticks for the scanning duration
				xTicksToWait = BASELINE_CALIBRATION_DURATION_SECONDS * 1000 / portTICK_PERIOD_MS;

				// Loop for the specified duration
				for (ticks = 0; ticks < xTicksToWait; ticks += 100 / portTICK_PERIOD_MS)
				{
					for (int i = 0; i < 9; i++)
					{
						data_macro[i] = roomsense_iq_shared.ld2410_data_shared.macro_bin[i];
					}

					for (int i = 0; i < 9; i++)
					{
						if (data_macro[i] < min_values_macro[i])
						{
							min_values_macro[i] = data_macro[i];
						}
					}

					for (int i = 0; i < 9; i++)
					{
						data_micro[i] = roomsense_iq_shared.ld2410_data_shared.micro_bin[i];
					}

					for (int i = 0; i < 9; i++)
					{
						if (data_micro[i] < min_values_micro[i])
						{
							min_values_micro[i] = data_micro[i];
						}
					}

					vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay for 100 milliseconds
				}

				// Add 20% to the max values for threshold
				for (int i = 0; i < 9; i++)
				{
					if(min_values_macro[i] <= 5)
					{
						roomsense_iq_shared.ld2410_config_shared.macro_threshold[i] = baseline_threshold_macro[i];
					}
					else
					{
						roomsense_iq_shared.ld2410_config_shared.macro_threshold[i] = min_values_macro[i] - 5;
						if(roomsense_iq_shared.ld2410_config_shared.macro_threshold[i] <= baseline_threshold_macro[i]) // make sure thresholds won't go below baseline
						{
							roomsense_iq_shared.ld2410_config_shared.macro_threshold[i] = baseline_threshold_macro[i];
							if(roomsense_iq_shared.ld2410_config_shared.absence_time_out < 30)
								roomsense_iq_shared.ld2410_config_shared.absence_time_out = 30;
						}

						if(roomsense_iq_shared.ld2410_config_shared.macro_threshold[i] <= 0)
							roomsense_iq_shared.ld2410_config_shared.macro_threshold[i] = baseline_threshold_macro[i];
					}

					if(min_values_micro[i] <= 5)
					{
						roomsense_iq_shared.ld2410_config_shared.micro_threshold[i] = baseline_threshold_micro[i];
					}
					else
					{
						roomsense_iq_shared.ld2410_config_shared.micro_threshold[i] = min_values_micro[i] - 5;  //0.2 * max_values_micro[i] + max_values_micro[i];
						if(roomsense_iq_shared.ld2410_config_shared.micro_threshold[i] <= baseline_threshold_micro[i]) // make sure thresholds won't go below baseline
						{
							roomsense_iq_shared.ld2410_config_shared.micro_threshold[i] = baseline_threshold_micro[i];
							if(roomsense_iq_shared.ld2410_config_shared.absence_time_out < 30) // increases timeout as an alternative to avoid blind spots
								roomsense_iq_shared.ld2410_config_shared.absence_time_out = 30;
						}
						if(roomsense_iq_shared.ld2410_config_shared.micro_threshold[i] <= 0)
							roomsense_iq_shared.ld2410_config_shared.micro_threshold[i] = baseline_threshold_micro[i];
					}
				}

				//send to web-ui
				send_config_setting(roomsense_iq_shared.ld2410_config_shared.max_macro_range,
						            roomsense_iq_shared.ld2410_config_shared.max_micro_range,
						            roomsense_iq_shared.ld2410_config_shared.absence_time_out,
									roomsense_iq_shared.pir_sensor_shared.pir_sensitivity,
									roomsense_iq_shared.ha_mqtt_shared.g_bedsense_status,
									roomsense_iq_shared.ld2410_config_shared.macro_threshold,
									roomsense_iq_shared.ld2410_config_shared.micro_threshold);
                //write to ld2410
				roomsense_iq_shared.buttons_shared.g_button_set_config = true;
			}
		}
		else
		{
			ESP_LOGI(TAG, "Could not take blindspot_semaphore");
		}
	}
}
