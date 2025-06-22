/**
 * @file sgp40.c
 *
 * ESP-IDF driver for SGP40 Indoor Air Quality Sensor for VOC Measurements
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_err.h>
#include "esp_idf_lib_helpers.h"
#include <esp_log.h>
#include <mpm10.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ets_sys.h"
#include <inttypes.h>
#include <stdio.h>
#include <esp_system.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>
#include "tasks_common.h"

#define I2C_FREQ_HZ 400000

static const char *TAG = "mpm10";

#define CMD_SOFT_RESET  0x0006
#define CMD_FEATURESET  0x202f
#define CMD_MEASURE_RAW 0x260f
#define CMD_SELF_TEST   0x280e
#define CMD_SERIAL      0x3682
#define CMD_HEATER_OFF  0x3615

#define TIME_SOFT_RESET  10
#define TIME_FEATURESET  10
#define TIME_MEASURE_RAW 30
#define TIME_SELF_TEST   250
#define TIME_HEATER_OFF  10
#define TIME_SERIAL      10

#define SELF_TEST_OK 0xd400

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

CircularBuffer buffer_pm1;
CircularBuffer buffer_pm2_5;
CircularBuffer buffer_pm10;

esp_err_t mpm10_init_desc(mpm10_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
	CHECK_ARG(dev);

	dev->i2c_dev.port = port;
	dev->i2c_dev.addr = MPM10_ADDR;
	dev->i2c_dev.cfg.sda_io_num = sda_gpio;
	dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
	dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
	return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t measure_pm(mpm10_t *dev, uint8_t reg, uint16_t *data)
{
	uint16_t tmp;
    uint8_t  raw[2];

	CHECK(i2c_dev_read_reg(&dev->i2c_dev, reg, raw, 1)); //Read high byte
	CHECK(i2c_dev_read_reg(&dev->i2c_dev, reg + 1, raw + 1, 1)); //Read low byte
	*data = 256 * raw[0] + raw[1];

	return ESP_OK;
}

static void mpm10_task(void *pvParameter)
{
	mpm10_t mpm;

	uint16_t data;

	// setup MPM10
	memset(&mpm, 0, sizeof(mpm));
	ESP_ERROR_CHECK_WITHOUT_ABORT(mpm10_init_desc(&mpm, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

	// Wait until all set up
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	while (1)
	{
//		CHECK(read_reg(&mpm, 0x21, &data));
//		ESP_LOGI(TAG, "pm1.0=%d", data);
//		vTaskDelay(200 / portTICK_PERIOD_MS);
//
//		CHECK(read_reg(&mpm, 0x23, &data));
//		ESP_LOGI(TAG, "pm2.5=%d", data);
//		vTaskDelay(200 / portTICK_PERIOD_MS);
//
//		CHECK(read_reg(&mpm, 0x25, &data));
//		ESP_LOGI(TAG, "pm10=%d", data);
		vTaskDelay(200 / portTICK_PERIOD_MS);
	}

}

void mpm10_task_start()
{

	ESP_ERROR_CHECK(i2cdev_init());
	xTaskCreatePinnedToCore(&mpm10_task, "mpm10_task", MPM10_TASK_STACK_SIZE, NULL, MPM10_TASK_PRIORITY, NULL, MPM10_TASK_CORE_ID);

}
