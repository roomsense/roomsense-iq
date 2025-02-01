/**
 * @file sgp40.h
 * @defgroup sgp40 sgp40
 * @{
 *
 * ESP-IDF driver for SGP40 Indoor Air Quality Sensor for VOC Measurements
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MPM10_H__
#define __MPM10_H__

#include <stdbool.h>
#include <time.h>
#include <../../esp-idf-lib/components/i2cdev/i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPM10_ADDR 0x4D //!< I2C address

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    uint16_t pm_1;
    uint16_t pm_2_5;
    uint16_t pm_10;
} mpm10_t;


/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */

#define PM_ONE_REG          0x20
#define PM_TWO_AND_HALF_REG 0x22
#define PM_TEN_REG          0x24

void        mpm10_task_start();
static void mpm10_task(void *pvParameter);

esp_err_t measure_pm(mpm10_t *dev, uint8_t reg, uint16_t *data);

esp_err_t mpm10_init_desc(mpm10_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MPM10_H__ */
