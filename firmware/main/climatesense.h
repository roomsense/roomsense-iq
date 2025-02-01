/*
 * climatesense.h
 *
 *  Created on: Jan. 11, 2024
 *      Author: builder
 */

#ifndef MAIN_CLIMATESENSE_H_
#define MAIN_CLIMATESENSE_H_



static void climatesense_task(void *pvParameter);
void climatesense_task_start(void);
static const char*  voc_actionable_insight(int32_t voc_index, uint16_t co2);
static const char*  co2_actionable_insight(uint16_t co2);
static const char*  humidity_actionable_insight(float h);
static const char*  pm_actionable_insight(uint16_t pm1, uint16_t pm2_5, uint16_t pm10);
static const char*  dew_point_actionable_insight(float temperature, float dew_point);
float  calculate_dew_point(float temperature, float humidity);

#endif /* MAIN_CLIMATESENSE_H_ */
