#include "dashboard.h"
#include <esp_log.h>
#include <ld2410.h>
#include "sht30.h"
#include "scd4x.h"
#include "pir-sensor.h"
#include "freertos/event_groups.h"
#include "tasks_common.h"

static const char *TAG = "DASHBOARD";

// Define an event group handle
EventGroupHandle_t dashboard_event_group;
bool dashboard_started = false;
const int TEMPERATUR_READY_BIT_W = BIT0;
const int TEMPERATUR_READY_BIT_R = BIT1;

// g_occupancy_status: This is for the house icon. If g_occupancy_status = 0 then the icon turns blue and the below text "Vacant",
//If g_occupancy_status = 2 then the house icon turns oragne and the text below is "Occupied".


//g_ld2410_data.detected_distance: distance

//g_pir_status for the PIR icon. if g_pir_status = 1 then the icon turns red. if g_pir_status = 0 then the icon turns gray.


extern CircularBuffer buffer_temperature;
extern CircularBuffer buffer_humidity;
extern CircularBuffer buffer_light;

extern bool temprature_data_change ;

extern int page_reload_event ;
uint8_t micro_th [9] = {0};
uint8_t macro_th [9] = {0};

void track_THL_24h(void){

}
void add_macro_movement_data(void);
void add_micro_movement_data(void);

void send_config_setting(uint32_t max_macro_range, uint32_t max_micro_range, uint32_t timeout,
						 uint32_t pir_sensistivity, uint32_t bedsense_state, int8_t*set_macro_th, int8_t* set_micro_th)
{
	uint32_t settings[5] = {max_macro_range, max_micro_range, timeout, pir_sensistivity,bedsense_state};
	send_data_fame(SETTINGS,settings,5,false,false);
	for(int i=0; i<9; i++){
		macro_th[i] = set_macro_th[i];
	}

	for(int i=0; i<9; i++){
		micro_th[i] = set_micro_th[i];
	}
    add_macro_movement_data(); //Sends Macro movement / Slider thresholds, data to dashboard UI
    add_micro_movement_data(); //Sends Micro movement / Slider thresholds, data to dashboard UI
}

void sliders_changed_cb(slider_data_t ctx){
	/* Handling Micro Threshold Sliders */
    if(is_slider_name(ctx,"micro_th1")){
        printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
        roomsense_iq_shared.ld2410_config_shared.micro_threshold[0] = ctx.value;
        micro_th[0] = ctx.value;
        roomsense_iq_shared.buttons_shared.g_button_set_config = true;
        return;
    }
    else
    if(is_slider_name(ctx,"micro_th2")){
        printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
        roomsense_iq_shared.ld2410_config_shared.micro_threshold[1] = ctx.value;
        micro_th[1] = ctx.value;
        roomsense_iq_shared.buttons_shared.g_button_set_config = true;
        return;
    }
    else
    if(is_slider_name(ctx,"micro_th3")){
        printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
        roomsense_iq_shared.ld2410_config_shared.micro_threshold[2] = ctx.value;
        micro_th[2] = ctx.value;
        roomsense_iq_shared.buttons_shared.g_button_set_config = true;
        return;
    }
    else
    if(is_slider_name(ctx,"micro_th4")){
        printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
        roomsense_iq_shared.ld2410_config_shared.micro_threshold[3] = ctx.value;
        micro_th[3] = ctx.value;
        roomsense_iq_shared.buttons_shared.g_button_set_config = true;
        return;
    }
    else
	if(is_slider_name(ctx,"micro_th5")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.micro_threshold[4] = ctx.value;
		micro_th[4] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else
	if(is_slider_name(ctx,"micro_th6")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.micro_threshold[5] = ctx.value;
		micro_th[5] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
    else
	if(is_slider_name(ctx,"micro_th7")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.micro_threshold[6] = ctx.value;
		micro_th[6] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else
	if(is_slider_name(ctx,"micro_th8")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.micro_threshold[7] = ctx.	value;
		micro_th[7] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else

	if(is_slider_name(ctx,"micro_th9")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.micro_threshold[8] = ctx.value;
		micro_th[8] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else

	/* Handling Macro Threshold Sliders */
	if(is_slider_name(ctx,"macro_th1")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.macro_threshold[0] = ctx.value;
		macro_th[0] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else
	if(is_slider_name(ctx,"macro_th2")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.macro_threshold[1] = ctx.value;
		macro_th[1] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else
	if(is_slider_name(ctx,"macro_th3")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.macro_threshold[2] = ctx.value;
		macro_th[2] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else
	if(is_slider_name(ctx,"macro_th4")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.macro_threshold[3] = ctx.value;
		macro_th[3] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else
	if(is_slider_name(ctx,"macro_th5")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.macro_threshold[4] = ctx.value;
		macro_th[4] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else
	if(is_slider_name(ctx,"macro_th6")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.macro_threshold[5] = ctx.value;
		macro_th[5] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else
	if(is_slider_name(ctx,"macro_th7")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.macro_threshold[6] = ctx.value;
		macro_th[6] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else
	if(is_slider_name(ctx,"macro_th8")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.macro_threshold[7] = ctx.value;
		macro_th[7] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else

	if(is_slider_name(ctx,"macro_th9")){
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.ld2410_config_shared.macro_threshold[8] = ctx.value;
		macro_th[8] = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else

	/* Handling Setting Buttons Sliders */
    if(is_slider_name(ctx,"BedSense")){
    	roomsense_iq_shared.ha_mqtt_shared.g_bedsense_status = (bool) ctx.value;
        printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, roomsense_iq_shared.ha_mqtt_shared.g_bedsense_status);
        return;
    }
    else
    if(is_slider_name(ctx,"CalSense")){
    	roomsense_iq_shared.ha_mqtt_shared.g_calsense_status = (bool) ctx.value;
        printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, roomsense_iq_shared.ha_mqtt_shared.g_calsense_status);
        return;
    }
    else
	if(is_slider_name(ctx,"max_macro_range")){
		roomsense_iq_shared.ld2410_config_shared.max_macro_range = ctx.value;
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else
	if(is_slider_name(ctx,"max_micro_range")){
		roomsense_iq_shared.ld2410_config_shared.max_micro_range = ctx.value;
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
    if(is_slider_name(ctx,"Timeout")){
    	roomsense_iq_shared.ld2410_config_shared.absence_time_out = ctx.value;
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		roomsense_iq_shared.buttons_shared.g_button_set_config = true;
		return;
	}
	else
	if(is_slider_name(ctx,"pir_sensitivity")){
		roomsense_iq_shared.pir_sensor_shared.pir_sensitivity = ctx.value;
		roomsense_iq_shared.buttons_shared.g_button_config_to_pir = true;
		printf("[EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		return;
	}
	else
	if(is_slider_name(ctx,"get_cnfg")){
		roomsense_iq_shared.buttons_shared.g_button_get_config = true;
		printf("[EVENT] Get Config button Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		/*
		 * API usage example to send settings to web interface.
		 * */

		send_config_setting(roomsense_iq_shared.ld2410_config_shared.max_macro_range,
				            roomsense_iq_shared.ld2410_config_shared.max_micro_range,
				            roomsense_iq_shared.ld2410_config_shared.absence_time_out,
							roomsense_iq_shared.pir_sensor_shared.pir_sensitivity,
							roomsense_iq_shared.ha_mqtt_shared.g_bedsense_status,
							roomsense_iq_shared.ld2410_config_shared.macro_threshold,
							roomsense_iq_shared.ld2410_config_shared.micro_threshold);
		return;
	}
	else
	if(is_slider_name(ctx,"factory_reset")){
		roomsense_iq_shared.buttons_shared.g_button_factory_reset  =  true;

        //Give enough chance to load the values in factory_reset() function.
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

		send_config_setting(roomsense_iq_shared.ld2410_config_shared.max_macro_range,
				            roomsense_iq_shared.ld2410_config_shared.max_micro_range,
				            roomsense_iq_shared.ld2410_config_shared.absence_time_out,
				            3,0,
							roomsense_iq_shared.ld2410_config_shared.macro_threshold,
							roomsense_iq_shared.ld2410_config_shared.micro_threshold);
		return;
	}
	else
	if(is_slider_name(ctx,"baseline_calibration")){
		xSemaphoreGive(baseline_semaphore); // Signal to baseline cal task
		printf("----------------------------> [EVENT] baseline_semaphore Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		return;
	}
	else
	if(is_slider_name(ctx,"blind_spot_calibration")){
		xSemaphoreGive(blindspot_semaphore); // Signal to blindspot cal task
		printf("------------------------------------> [EVENT] blind_spot_calibration Updated %s : %d\r\n",ctx.slider_name, ctx.value);
		return;
	}
	else
	if(is_slider_name(ctx,"co2_cal")){
		    roomsense_iq_shared.forced_calibration = true;
			printf("[------>EVENT] Updated %s : %d\r\n",ctx.slider_name, ctx.value);
			return;
		}
    else printf("[UNHANDLED_SLIDERS] type : %s, value : %d\r\n",ctx.slider_name, ctx.value);  
}


/**
 * Creates Random Macromovement data;
*/

void add_macro_movement_data(void){
	uint32_t raw_data[MAX_DATA_POINTS] = {0};
    for (int i = 0; i < 9; i++) {
            raw_data[i] = roomsense_iq_shared.ld2410_data_shared.macro_bin[i];
        }

    for (int i = 0; i < 9; i++) {
        raw_data[9+i] = macro_th[i];
    }
    send_data_fame(MACRO_MOVEMENT,raw_data,MAX_DATA_POINTS, false, false);
}

/**
 * Creates Random Micromovement data;
*/

void add_micro_movement_data(void){
	uint32_t raw_data[MAX_DATA_POINTS] = {0};
    for (int i = 0; i < 9; i++) {
        raw_data[i] = roomsense_iq_shared.ld2410_data_shared.micro_bin[i];
    }

    for (int i = 0; i < 9; i++) {
           raw_data[i+9] = micro_th[i];
    }

    send_data_fame(MICRO_MOVEMENT,raw_data,MAX_DATA_POINTS, false, false);
}


void add_temprature_data(void){
	int value;
    uint32_t raw_data[96] = {0};
    uint32_t dummy2[12] =  {1,2,3,4,5,6,6,7,8,9};

	if(read_from_buffer(&buffer_temperature, &value))
	{
		printf("T_DATA %d,%d\r\n",value,buffer_temperature.writeIndex);
		raw_data[0] = (uint32_t)value;
		send_data_fame(TEMPERATURE,raw_data,1, false, false);
	}
	//else send_data_fame(TEMPERATURE,raw_data,1, false, true);

	raw_data[0] = (int)roomsense_iq_shared.scd40_shared.temperature;

    send_data_fame(TEMPERATURE,raw_data,1, true, false);
}


void add_humidity_data(void){
    uint32_t raw_data[2] = {0};
	int value;
	//Points recently added data point
	if(read_from_buffer(&buffer_humidity, &value))
	{
		printf("H_DATA %d,%d\r\n",value,buffer_humidity.writeIndex);
		raw_data[0] = (uint32_t)value;
		send_data_fame(HUMIDITY,raw_data,1, false, false);
	}
	//else send_data_fame(HUMIDITY,raw_data,1, false, true);
	raw_data[0] = (int)roomsense_iq_shared.scd40_shared.humidity;
	send_data_fame(HUMIDITY,raw_data,1, true, false);
}

void add_light_density_data(void){
    uint32_t raw_data[2] = {0};
	int value;
	if(read_from_buffer(&buffer_light, &value))
	{
		raw_data[0] = (uint32_t)value;
		send_data_fame(LIGHT_DENSITY,raw_data,1, false, false);
	}
	raw_data[0] = roomsense_iq_shared.adafruit_161_shared.light_density_raw;
    send_data_fame(LIGHT_DENSITY,raw_data,1, true, false);
}


void add_co2_data(void){
    uint32_t raw_data[2] = {0};
	int value;
	//Points recently added data point
	if(read_from_buffer(&buffer_co2, &value))
	{
		printf("CO2_DATA %d,%d\r\n",value,buffer_co2.writeIndex);
		raw_data[0] = (uint32_t)value;
		send_data_fame(CO2,raw_data,1, false, false);
	}
	//else send_data_fame(HUMIDITY,raw_data,1, false, true);
	raw_data[0] = (int)roomsense_iq_shared.scd40_shared.co2;
	send_data_fame(CO2,raw_data,1, true, false);
}
extern CircularBuffer buffer_co;
void add_co_data(){
	uint32_t raw_data[2] = {0};
	int value;
	if(read_from_buffer(&buffer_co, &value))
	{
		printf("CO_DATA %d,%d\r\n",value,buffer_co.writeIndex);
		raw_data[0] = (uint32_t)value;
		send_data_fame(CO,raw_data,1, false, false);
	}
	raw_data[0] = (int)roomsense_iq_shared.tgs5141_shared.co;
	send_data_fame(CO,raw_data,1, true, false);
}

void add_voc_data(void){
    uint32_t raw_data[2] = {0};
	int value;
	//Points recently added data point
	if(read_from_buffer(&buffer_voc, &value))
	{
		raw_data[0] = (uint32_t)value;
		send_data_fame(VOC,raw_data,1, false, false);
	}
	//else send_data_fame(HUMIDITY,raw_data,1, false, true);
	raw_data[0] = (int)roomsense_iq_shared.sgp40_shared.voc_index;
	send_data_fame(VOC,raw_data,1, true, false);
}

void add_pm1_data(void){
    uint32_t raw_data[2] = {0};
	int value;
	//Points recently added data point
	if(read_from_buffer(&buffer_pm1, &value))
	{
		printf("PM1.0_DATA %d,%d\r\n",value,buffer_pm1.writeIndex);
		raw_data[0] = (uint32_t)value;
		send_data_fame(PM1_0,raw_data,1, false, false);
	}
	//else send_data_fame(HUMIDITY,raw_data,1, false, true);
	raw_data[0] = (int)roomsense_iq_shared.mpm10_shared.pm1;
	send_data_fame(PM1_0,raw_data,1, true, false);
}

void add_pm2_5_data(void){
    uint32_t raw_data[2] = {0};
	int value;
	//Points recently added data point
	if(read_from_buffer(&buffer_pm2_5, &value))
	{
		printf("PM2.5_DATA %d,%d\r\n",value,buffer_pm2_5.writeIndex);
		raw_data[0] = (uint32_t)value;
		send_data_fame(PM2_5,raw_data,1, false, false);
	}
	//else send_data_fame(HUMIDITY,raw_data,1, false, true);
	raw_data[0] = (int)roomsense_iq_shared.mpm10_shared.pm25;
	send_data_fame(PM2_5,raw_data,1, true, false);
}

void add_pm10_data(void){
    uint32_t raw_data[2] = {0};
	int value;
	//Points recently added data point
	if(read_from_buffer(&buffer_pm10, &value))
	{
		printf("PM10_DATA %d,%d\r\n",value,buffer_pm10.writeIndex);
		raw_data[0] = (uint32_t)value;
		send_data_fame(PM10,raw_data,1, false, false);
	}
	//else send_data_fame(HUMIDITY,raw_data,1, false, true);
	raw_data[0] = (int)roomsense_iq_shared.mpm10_shared.pm10;
	send_data_fame(PM10,raw_data,1, true, false);
}


void add_distance_data(void){
	uint32_t raw_data[1] = {0};
    raw_data[0] = roomsense_iq_shared.ld2410_data_shared.detected_distance;
    send_data_fame(DISTANCE,raw_data,1, false, false);
}

/* sends Occupancy status  */
void add_occ_status(void){
	if(roomsense_iq_shared.ha_mqtt_shared.g_occupancy_status){
		update_status_bar_data(OCC_STATE,    "\"Occupied\"");
	}
	else {
		update_status_bar_data(OCC_STATE,    "\"Unoccupied\"");
	}
}

void add_pir_state(void){
	if(roomsense_iq_shared.pir_sensor_shared.g_pir_status){
		update_status_bar_data(MOTION_STATE, "true");
	}
	else {
		update_status_bar_data(MOTION_STATE, "false");
	}
}

/**
 * Creates New data points. 
*/
uint32_t dummy_data[96] = {0};
uint32_t raw_data [96]=  {10, 20 ,30, 40, 50, 60, 70, 80, 90};
void create_climatesense_dataframe(void){

	if(page_reload_event){
		page_reload_event--;
//		printf("PAGE RELOAD EVENT DETECTED!");
		vTaskDelay(2500 / portTICK_PERIOD_MS);

		if(buffer_temperature.writeIndex){
			send_data_fame(TEMPERATURE,(uint32_t*)buffer_temperature.buffer,buffer_temperature.writeIndex, false, false);
		}

		if(buffer_humidity.writeIndex){
			send_data_fame(HUMIDITY,(uint32_t*)buffer_humidity.buffer,buffer_humidity.writeIndex, false, false);
		}
		if(buffer_co2.writeIndex){
			send_data_fame(CO2,(uint32_t*)buffer_co2.buffer,buffer_co2.writeIndex,false,false);
		}
		if(buffer_co.writeIndex){
			send_data_fame(CO,(uint32_t*)buffer_co.buffer,buffer_co.writeIndex,false,false);
		}
		if(buffer_voc.writeIndex){
			send_data_fame(VOC,(uint32_t*)buffer_voc.buffer,buffer_voc.writeIndex,false,false);
		}
		if(buffer_pm1.writeIndex){
			send_data_fame(PM1_0,(uint32_t*)buffer_pm1.buffer,buffer_pm1.writeIndex,false,false);
		}
		if(buffer_pm10.writeIndex){
			send_data_fame(PM10,(uint32_t*)buffer_pm10.buffer,buffer_pm10.writeIndex,false,false);
		}
		if(buffer_pm2_5.writeIndex){
			send_data_fame(PM2_5,(uint32_t*)buffer_pm2_5.buffer,buffer_pm2_5.writeIndex,false,false);
		}

		// else printf("buffer is empty on reload\r\n");
//		vTaskDelay(1000/portTICK_PERIOD_MS);
		page_reload_event = false;


	}
	add_co_data();
	vTaskDelay(10 / portTICK_PERIOD_MS);
	add_temprature_data();     //Sends Temperature data to dashboard UI
	vTaskDelay(10 / portTICK_PERIOD_MS);
	add_humidity_data();       //Sends Humidity data to dashboard UI
	vTaskDelay(10 / portTICK_PERIOD_MS);
	add_co2_data();
	vTaskDelay(10 / portTICK_PERIOD_MS);
	add_voc_data();
	vTaskDelay(10 / portTICK_PERIOD_MS);
	add_pm1_data();
	vTaskDelay(10 / portTICK_PERIOD_MS);
	add_pm2_5_data();
	vTaskDelay(10 / portTICK_PERIOD_MS);
	add_pm10_data();
	vTaskDelay(10 / portTICK_PERIOD_MS);

	srand(xTaskGetTickCount()* xTaskGetTickCount());

	vTaskDelay(10 / portTICK_PERIOD_MS);
}

void create_main_dashboard_dataframe(void){
	add_pir_state();
	vTaskDelay(10 / portTICK_PERIOD_MS);
	add_occ_status(); 		   //Sends Occupnacy status data to dashboard UI
	vTaskDelay(10 / portTICK_PERIOD_MS);

	add_macro_movement_data(); //Sends Macro movement / Slider thresholds, data to dashboard UI
	vTaskDelay(10 / portTICK_PERIOD_MS);
	add_micro_movement_data(); //Sends Micro movement / Slider thresholds, data to dashboard UI

	vTaskDelay(10 / portTICK_PERIOD_MS);
	add_light_density_data();  //Sends Light density data to dashboard UI
	vTaskDelay(10 / portTICK_PERIOD_MS);
	add_distance_data();	   //Sends Current distance data to dashboard UI
	//ESP_LOGI(TAG, "Sender_Task");
	//vTaskDelay(20);
	dashboard_started = false;
	vTaskDelay(20 / portTICK_PERIOD_MS);
}


void Sender_Task(void){

	while (1)       
	{
        create_climatesense_dataframe();
		create_main_dashboard_dataframe();
	    vTaskDelay(20 / portTICK_PERIOD_MS);
	}
}

void dashboard_api_init(void){

    xTaskCreate(&Sender_Task,"sender", 1024*6, NULL, 2, NULL);
    dashboard_started = true;
}


bool is_slider_name(slider_data_t slider_dctx, char* name){
   return !strcmp(slider_dctx.slider_name, name);
}
