/*
 * doppler_iq.h
 *
 *  Created on: Sep. 17, 2022
 *      Author: sinam
 */

#ifndef MAIN_DOPPLER_IQ_H_
#define MAIN_DOPPLER_IQ_H_


#include <stdbool.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include "tasks_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MM5D91 sensor device data structure
 */
typedef struct {
	uart_port_t uart_port;  ///< UART port used to communicate
	uint8_t *buf;           ///< read buffer attached to this device
	int16_t last_value;     ///< last read value
	int64_t last_ts;        ///< timestamp of the last sensor co2 level reading
} ld2410_dev_t;

#define DISTANCE_RESOLUTION    11

#define ENABLED   1
#define DISABLED  0

#define LD2410_SUCCESS   1
#define LD2410_FAIL      0

#define SERIAL_DEBUG_ENABLED            1

#define IDLE_STATE       0
#define SEND_STATE       1
#define RECEIVE_STATE    2
#define PARSING_STATE    3
#define VALIDATE_STATE   4
#define END_CONFIG_STATE 5
#define RETURN_STATE     6

#define HEADER                          0xFDFCFBFA
#define HEAD                            0xAA
#define TAIL                            0x55
#define CHECK_MSG                       0x00
#define MFR                             0x4321
#define BLANK_CMD                       0x0000
#define ENABLE_CONFIG                   0x00FF
#define END_CONFIG                      0x00FE
#define SET_RANGE                       0x0060
#define READ_CONFIG                     0x0061
#define ENGINEERING_MODE                0x0062
#define NORMAL_MODE                     0x0063
#define SET_BIN_SENSITIVITY             0x0064
#define READ_FW_VERSION                 0x00A0
#define SET_BAUD_RATE                   0x00A1
#define FACTORY_RESET                   0x00A2
#define REBOOT_MODULE                   0x00A3



#define CONFIG_LD2410_UART_RX 8
#define CONFIG_LD2410_UART_TX 9

//! 3 minutes warming-up time after power-on before valid data returned
#define MM5D91_WARMING_UP_TIME_MS       (3UL * 60000UL)
#define MM5D91_WARMING_UP_TIME_US       (3UL * 60000UL * 1000UL)

//! Minimum response time between CO2 reads (EXPERIMENTALLY DEFINED)
#define MM5D91_READ_INTERVAL_MS         (5UL * 1000UL)

#define LD2410_SERIAL_TX_BYTES          256
//! Fixed 9 Bytes response
#define LD2410_SERIAL_RX_BYTES          256
//! 128 is the minimal value the UART driver will accept (at least on esp32)
#define LD2410_SERIAL_BUF_LEN           256

//! Response timeout between 15..120 ms at 9600 baud works reliable for all commands
#define LD2410_SERIAL_RX_TIMEOUT_MS     60

#define NUM_SAMPLES                     40



/**
 * Message IDs for the LD2410 application task
 * @note Expand this based on your application requirements.
 */
typedef enum ld2410_app_message
{
	LD2410_APP_MSG_INIT = 0,
	LD2410_APP_MSG_CONNECT,
	LD2410_APP_MSG_START,
	LD2410_APP_MSG_SET_CONFIG,
	LD2410_APP_MSG_GET_CONFIG,
	LD2410_APP_MSG_COLLECT_DATA,
	LD2410_APP_MSG_FACTORY_RESET,
} ld2410_app_message_e;

/**
 * Structure for the message queue
 * @note Expand this based on application requirements e.g. add another type and parameter as required
 */
typedef struct ld2410_app_queue_message
{
	ld2410_app_message_e msgID;
} ld2410_app_queue_message_t;

uint16_t macro_distance_g, micro_distance_g, detection_distance_g;
uint16_t  macro_level_g, micro_level_g;
unsigned long time_us_g;


typedef struct {
	uint16_t* buffer; // Buffer to hold the data points
	uint16_t window_size; // Size of the moving average window
	uint16_t index; // Index of the current data point in the buffer
	uint16_t sum; // Sum of the data points in the buffer
} MovingAverageFilter;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Pointer to the sensor device data structure
 * @param uart_port UART poert number
 * @param tx_gpio GPIO pin number for TX
 * @param rx_gpio GPIO pin number for RX
 *
 * @return ESP_OK on success
 */



void moving_average_init(MovingAverageFilter* filter, uint16_t window_size);
uint16_t moving_average_update(MovingAverageFilter* filter, uint16_t new_data);
void moving_average_cleanup(MovingAverageFilter* filter);

esp_err_t ld2410_init(ld2410_dev_t *dev, uart_port_t uart_port, gpio_num_t tx_gpio, gpio_num_t rx_gpio);
esp_err_t ld2410_connect(ld2410_dev_t *dev);
esp_err_t ld2410_start(ld2410_dev_t *dev, uint8_t mode);
esp_err_t ld2410_get_data(ld2410_dev_t *dev, ld2410_data_t *ld2410_data, uint8_t mode);
esp_err_t set_config_mode(ld2410_dev_t *dev);
esp_err_t end_config_mode(ld2410_dev_t *dev);
esp_err_t reboot_module(ld2410_dev_t *dev);
esp_err_t blank_command(ld2410_dev_t *dev);
esp_err_t factory_reset(ld2410_dev_t *dev);
esp_err_t ld2410_send_command(ld2410_dev_t *dev, uint16_t command_word, uint8_t *command_value, uint16_t commandvalue_len, uint8_t bin_num);
esp_err_t ld2410_verify_ack(ld2410_dev_t *dev, uint16_t command_word, uint16_t commandvalue_len, uint16_t len);
esp_err_t ld2410_set_config(ld2410_dev_t *dev, ld2410_config_t *ld2410_config);
esp_err_t ld2410_get_config(ld2410_dev_t *dev, ld2410_config_t *ld2410_config);

void ld2410_task_start(void);
static void baseline_calibration_task(void *pvParameters);
static void blindspot_calibration_task(void *pvParameters);

BaseType_t ld2410_app_send_message(ld2410_app_message_e msgID);
BaseType_t ld2410_app_send_distance(uint16_t distance);




#ifdef __cplusplus
}
#endif /* End of CPP guard */

/**@}*/

#endif /* MAIN_DOPPLER_IQ_H_ */
