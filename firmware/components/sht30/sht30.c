/*
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file sht3x.c
 *
 * ESP-IDF driver for Sensirion SHT3x digital temperature and humidity sensor
 *
 * Forked from <https://github.com/gschorcht/sht3x-esp-idf>
 *
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>\n
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_timer.h>
#include "esp_idf_lib_helpers.h"
#include <string.h>
#include <esp_err.h>
#include "sht30.h"
#include "tasks_common.h"
#include "ha_mqtt.h"
#include "freertos/event_groups.h"


#define ONE_MINUTE_MS 60000
#define FIFTEEN_MINUTES_MS 900000

#define I2C_FREQ_HZ 1000000 // 1MHz
#define CONFIG_ROOMSENSE_SHT3X_ADDR 0x44

const char *TAG = "sht3x";

#define SHT3X_STATUS_CMD               0xF32D
#define SHT3X_CLEAR_STATUS_CMD         0x3041
#define SHT3X_RESET_CMD                0x30A2
#define SHT3X_FETCH_DATA_CMD           0xE000
#define SHT3X_STOP_PERIODIC_MEAS_CMD   0x3093
#define SHT3X_HEATER_ON_CMD            0x306D
#define SHT3X_HEATER_OFF_CMD           0x3066

#define TEMPERATURE_OFFSET   19
#define HUMIDITY_OFFSET   16.5

static sht3x_t dev;

CircularBuffer buffer_temperature;
CircularBuffer buffer_humidity;

extern const int TEMPERATUR_READY_BIT_W;
extern const int TEMPERATUR_READY_BIT_R;

extern CircularBuffer* buffer_light;
extern EventGroupHandle_t dashboard_event_group;
bool temprature_data_change = false ;

static const uint16_t SHT3X_MEASURE_CMD[6][3] =
{
        { 0x2400, 0x240b, 0x2416 }, // [SINGLE_SHOT][H,M,L] without clock stretching
		{ 0x2032, 0x2024, 0x202f }, // [PERIODIC_05][H,M,L]
		{ 0x2130, 0x2126, 0x212d }, // [PERIODIC_1 ][H,M,L]
		{ 0x2236, 0x2220, 0x222b }, // [PERIODIC_2 ][H,M,L]
		{ 0x2334, 0x2322, 0x2329 }, // [PERIODIC_4 ][H,M,L]
		{ 0x2737, 0x2721, 0x272a }  // [PERIODIC_10][H,M,L]
};

static const float t_offset[31] =
{6.91, 8.93, 10.48, 11.73, 12.7, 13.45, 14.21, 14.62, 15.04, 15.53, 15.79, 16.33, 16.34, 16.43, 16.49,
	16.61, 16.54, 16.78, 16.81, 16.9, 17.04, 17, 17.15, 17.19, 17.22, 17.31, 17.28, 17.04, 17.14, 17.25, 17.24
};


static const float h_offset[31] =
{-8.37, -11.65, -13.84, -15.17, -16.39, -17.36, -17.96, -18.59, -19.06, -19.5, -19.75, -20.19, -20.4, -20.56,
-20.47, -20.58, -20.57, -20.76, -20.74, -20.86, -20.99, -21.03, -21.2, -21.21, -21.18, -21.26, -21.29, -21.1,
-21.24, -21.35, -21.3};

// due to the fact that ticks can be smaller than portTICK_PERIOD_MS, one and
// a half tick period added to the duration to be sure that waiting time for
// the results is long enough
#define TIME_TO_TICKS(ms) (1 + ((ms) + (portTICK_PERIOD_MS-1) + portTICK_PERIOD_MS/2 ) / portTICK_PERIOD_MS)

#define SHT3X_MEAS_DURATION_REP_HIGH   15
#define SHT3X_MEAS_DURATION_REP_MEDIUM 6
#define SHT3X_MEAS_DURATION_REP_LOW    4

// measurement durations in us
static const uint16_t SHT3X_MEAS_DURATION_US[3] =
{
SHT3X_MEAS_DURATION_REP_HIGH * 1000,
SHT3X_MEAS_DURATION_REP_MEDIUM * 1000,
SHT3X_MEAS_DURATION_REP_LOW * 1000 };

// measurement durations in RTOS ticks
static const uint8_t SHT3X_MEAS_DURATION_TICKS[3] =
{ TIME_TO_TICKS(SHT3X_MEAS_DURATION_REP_HIGH), TIME_TO_TICKS(SHT3X_MEAS_DURATION_REP_MEDIUM), TIME_TO_TICKS(SHT3X_MEAS_DURATION_REP_LOW) };

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define G_POLYNOM 0x31


void init_buffer(CircularBuffer *buffer)
{
	buffer->readIndex    = 0;
	buffer->writeIndex   = 0;
	buffer->bufferLength = 0;
	buffer->valid_data_len = 0;
}

bool write_to_buffer(CircularBuffer *buffer, int value)
{
	// Check if buffer is full
	if (buffer->bufferLength == BUFFER_SIZE)
	{
		//printf("\n    Buffer is full!\n\n    ");
		return false;
	}

	buffer->buffer[buffer->writeIndex] = value;
	//buffer[writeIndex] = value;

	if(buffer->writeIndex == 1)buffer->valid_data_len = 0;
	else buffer->valid_data_len++;

	buffer->bufferLength++;	 //	Increase buffer size after writing
	buffer->writeIndex++;	 //	Increase writeIndex position to prepare for next write

	// If at last index in buffer, set writeIndex back to 0
	if (buffer->writeIndex == BUFFER_SIZE) {
		buffer->writeIndex = 0;
	}
	return true;
}

bool read_from_buffer(CircularBuffer *buffer, int *value)
{
    // Check if buffer is empty
	if (buffer->bufferLength == 0)
	{
		//printf("\n    Buffer is empty!\n\n    ");
		return false;
	}

	//*value = circularBuffer[readIndex];
	*value = buffer->buffer[buffer->readIndex];

	buffer->bufferLength--;	 //	Decrease buffer size after reading
	buffer->readIndex++;	 //	Increase readIndex position to prepare for next read

    // If at last index in buffer, set readIndex back to 0
	if (buffer->readIndex == BUFFER_SIZE)
	{
		buffer->readIndex = 0;
	}
	return true;
}

static inline uint16_t shuffle(uint16_t val)
{
	return (val >> 8) | (val << 8);
}

static uint8_t crc8(uint8_t data[], int len)
{
	// initialization value
	uint8_t crc = 0xff;

	// iterate over all bytes
	for (int i = 0; i < len; i++)
	{
		crc ^= data[i];
		for (int i = 0; i < 8; i++)
		{
			bool xor = crc & 0x80;
			crc = crc << 1;
			crc = xor ? crc ^ G_POLYNOM : crc;
		}
	}
	return crc;
}

static esp_err_t send_cmd_nolock(sht3x_t *dev, uint16_t cmd)
{
	cmd = shuffle(cmd);

	return i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, 2);
}

static esp_err_t send_cmd(sht3x_t *dev, uint16_t cmd)
{
	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, send_cmd_nolock(dev, cmd));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	return ESP_OK;
}

static esp_err_t start_nolock(sht3x_t *dev, sht3x_mode_t mode, sht3x_repeat_t repeat)
{
	dev->mode = mode;
	dev->repeatability = repeat;
	CHECK(send_cmd_nolock(dev, SHT3X_MEASURE_CMD[mode][repeat]));
	dev->meas_start_time = esp_timer_get_time();
	dev->meas_started = true;
	dev->meas_first = true;

	return ESP_OK;
}

static inline bool is_measuring(sht3x_t *dev)
{
	// not running if measurement is not started at all or
	// it is not the first measurement in periodic mode
	if (!dev->meas_started || !dev->meas_first)
		return false;

	// not running if time elapsed is greater than duration
	uint64_t elapsed = esp_timer_get_time() - dev->meas_start_time;

	return elapsed < SHT3X_MEAS_DURATION_US[dev->repeatability];
}

static esp_err_t get_raw_data_nolock(sht3x_t *dev, sht3x_raw_data_t raw_data)
{
	if (!dev->meas_started)
	{
		ESP_LOGE(TAG, "Measurement is not started");
		return ESP_ERR_INVALID_STATE;
	}
	if (is_measuring(dev))
	{
		ESP_LOGE(TAG, "Measurement is still running");
		return ESP_ERR_INVALID_STATE;
	}

	// read raw data
	uint16_t cmd = shuffle(SHT3X_FETCH_DATA_CMD);
	CHECK(i2c_dev_read(&dev->i2c_dev, &cmd, 2, raw_data, sizeof(sht3x_raw_data_t)));

	// reset first measurement flag
	dev->meas_first = false;

	// reset measurement started flag in single shot mode
	if (dev->mode == SHT3X_SINGLE_SHOT)
		dev->meas_started = false;

	// check temperature crc
	if (crc8(raw_data, 2) != raw_data[2])
	{
		ESP_LOGE(TAG, "CRC check for temperature data failed");
		return ESP_ERR_INVALID_CRC;
	}

	// check humidity crc
	if (crc8(raw_data + 3, 2) != raw_data[5])
	{
		ESP_LOGE(TAG, "CRC check for humidity data failed");
		return ESP_ERR_INVALID_CRC;
	}

	return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t sht3x_init_desc(sht3x_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
	CHECK_ARG(dev);

	dev->i2c_dev.port = port;
	dev->i2c_dev.addr = addr;
	dev->i2c_dev.cfg.sda_io_num = sda_gpio;
	dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
	dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

	return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t sht3x_free_desc(sht3x_t *dev)
{
	CHECK_ARG(dev);

	return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t sht3x_init(sht3x_t *dev)
{
	CHECK_ARG(dev);

	dev->mode = SHT3X_SINGLE_SHOT;
	dev->meas_start_time = 0;
	dev->meas_started = false;
	dev->meas_first = false;

	return send_cmd(dev, SHT3X_CLEAR_STATUS_CMD);
}

esp_err_t sht3x_set_heater(sht3x_t *dev, bool enable)
{
	CHECK_ARG(dev);

	return send_cmd(dev, enable ? SHT3X_HEATER_ON_CMD : SHT3X_HEATER_OFF_CMD);
}

esp_err_t sht3x_compute_values(sht3x_raw_data_t raw_data, float *temperature, float *humidity)
{
	CHECK_ARG(raw_data && (temperature || humidity));

	if (temperature)
		*temperature = ((((raw_data[0] * 256.0) + raw_data[1]) * 175) / 65535.0) - 45;

	if (humidity)
		*humidity = ((((raw_data[3] * 256.0) + raw_data[4]) * 100) / 65535.0);

	return ESP_OK;
}

esp_err_t sht3x_measure(sht3x_t *dev, float *temperature, float *humidity)
{
	CHECK_ARG(dev && (temperature || humidity));

	sht3x_raw_data_t raw_data;

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, start_nolock(dev, SHT3X_SINGLE_SHOT, SHT3X_HIGH));
	vTaskDelay(SHT3X_MEAS_DURATION_TICKS[SHT3X_HIGH]);
	I2C_DEV_CHECK(&dev->i2c_dev, get_raw_data_nolock(dev, raw_data));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	return sht3x_compute_values(raw_data, temperature, humidity);
}

uint8_t sht3x_get_measurement_duration(sht3x_repeat_t repeat)
{
	return SHT3X_MEAS_DURATION_TICKS[repeat];  // in RTOS ticks
}

esp_err_t sht3x_start_measurement(sht3x_t *dev, sht3x_mode_t mode, sht3x_repeat_t repeat)
{
	CHECK_ARG(dev);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, start_nolock(dev, mode, repeat));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	return ESP_OK;
}

esp_err_t sht3x_stop_periodic_measurement(sht3x_t *dev)
{
	CHECK_ARG(dev);

	CHECK(send_cmd(dev, SHT3X_STOP_PERIODIC_MEAS_CMD));
	dev->mode = SHT3X_SINGLE_SHOT;
	dev->meas_start_time = 0;
	dev->meas_started = false;
	dev->meas_first = false;

	return ESP_OK;
}

esp_err_t sht3x_get_raw_data(sht3x_t *dev, sht3x_raw_data_t raw_data)
{
	CHECK_ARG(dev && raw_data);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, get_raw_data_nolock(dev, raw_data));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	return ESP_OK;
}

esp_err_t sht3x_get_results(sht3x_t *dev, float *temperature, float *humidity)
{
	CHECK_ARG(dev && (temperature || humidity));

	sht3x_raw_data_t raw_data;

	CHECK(sht3x_get_raw_data(dev, raw_data));

	return sht3x_compute_values(raw_data, temperature, humidity);
}

/**
 * SHT3X Sensor task
 */
static void SHT3X_task(void *pvParameter)
{
	float temperature, humidity;
	uint64_t current_time_us;
	uint current_time_ms;
	esp_err_t res;
	int counter = 0, start_counter = 0;
	start_tier_e start_tier = TIER_0;
	measurment_state_e measurment_state = TIER_ADJUSTMENT;

	init_buffer(&buffer_temperature);
	init_buffer(&buffer_humidity);

	vTaskDelay(1000 / portTICK_RATE_MS);
	// Start periodic measurements with 1 measurement per second.
	ESP_ERROR_CHECK_WITHOUT_ABORT(sht3x_start_measurement(&dev, SHT3X_PERIODIC_1MPS, SHT3X_HIGH));

	// Wait until first measurement is ready (constant time of at least 30 ms
	// or the duration returned from *sht3x_get_measurement_duration*).
	vTaskDelay(sht3x_get_measurement_duration(SHT3X_HIGH));

	TickType_t last_wakeup = xTaskGetTickCount();
	counter = 0;


	while (1)
	{

		switch (measurment_state)
		{
		case TIER_ADJUSTMENT:
			if ((res = sht3x_get_results(&dev, &temperature, &humidity)) == ESP_OK)
			{
				if (temperature < 31.31)
				{
					start_tier = TIER_0;
					counter = 0;
				}
				else if (temperature > 31.31 && temperature < 37.2)
				{
					start_tier = TIER_1;
					counter = 1;
				}
				else if (temperature > 37.2 && temperature < 39.94)
				{
					start_tier = TIER_2;
					counter = 6;
				}
				else if (temperature > 39.94 && temperature < 40.90)
				{
					start_tier = TIER_3;
					counter = 11;
				}
				else if (temperature > 40.90 && temperature < 41.31)
				{
					start_tier = TIER_4;
					counter = 16;
				}
				else if (temperature > 41.31 && temperature < 41.62)
				{
					start_tier = TIER_5;
					counter = 21;
				}
				else if (temperature > 41.62 && temperature < 41.65)
				{
					start_tier = TIER_6;
					counter = 26;
				}
				else if (temperature > 41.65)
				{
					start_tier = TIER_7;
					counter = 31;
				}
				measurment_state = SS_MEASURMENT;
			}
			else
			{
				printf("Could not get results: %d (%s)", res, esp_err_to_name(res));
				measurment_state = TIER_ADJUSTMENT;
				vTaskDelay(1000 / portTICK_RATE_MS);
			}
			break;

		case SS_MEASURMENT:

			if ((res = sht3x_get_results(&dev, &temperature, &humidity)) == ESP_OK)
			{
				temperature -= t_offset[counter];
				humidity -= h_offset[counter];

				if (counter < 30)
				{
					counter++;
				}
				else
				{
					counter = 30;
				}
			}
			else
				printf("Could not get results: %d (%s)", res, esp_err_to_name(res));

            roomsense_iq_shared.sht30_shared.g_temperature = temperature;
			roomsense_iq_shared.sht30_shared.g_humidity = humidity;
			printf("WRITING TH DATA \r\n");
			write_to_buffer(&buffer_temperature, roomsense_iq_shared.sht30_shared.g_temperature);
			write_to_buffer(&buffer_humidity, roomsense_iq_shared.sht30_shared.g_humidity);

			vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(2000));
			//vTaskDelay(2000 / portTICK_RATE_MS);
			start_counter++;
			if(start_counter < 6)
			{
				counter--;
				if(counter < 0)
					counter = 0;
			}
			else //avoid the delay for the first 5 measurments to avoid non-sense numbers on the dashboard.then it is every 15min
			{
				vTaskDelay(ONE_MINUTE_MS / portTICK_RATE_MS);
				start_counter = 10;
				if (xTaskGetTickCount() % (FIFTEEN_MINUTES_MS / portTICK_PERIOD_MS) == 0) {
					//addToBuffer(buffer_temperature, temperature);
					//addToBuffer(buffer_humidity, humidity);
				}
			}
			break;
		}
	}
}

void SHT3X_task_start(void)
{
	ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());
	memset(&dev, 0, sizeof(sht3x_t));

	ESP_ERROR_CHECK_WITHOUT_ABORT(sht3x_init_desc(&dev, CONFIG_ROOMSENSE_SHT3X_ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
	ESP_ERROR_CHECK_WITHOUT_ABORT(sht3x_init(&dev));

	xTaskCreatePinnedToCore(&SHT3X_task, "SHT3X_task", SHT3X_TASK_STACK_SIZE, NULL, SHT3X_TASK_PRIORITY, NULL, SHT3X_TASK_CORE_ID);
}
