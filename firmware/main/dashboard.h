
#include "stdio.h"
#include "string.h"
#include "esp_http_server.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "cJSON.h"
#define MAX_DATA_POINTS 18

#define JSON_STRT '{'
#define JSON_CNT ','
#define JSON_END '}'
#define JSON_TERMINATE 'e'

typedef enum{
    UNDEFINED_PAGE = 0,
    INDEX_PAGE,
    MAIN_DASHBOARD_PAGE,
    CLIMATESENSE_PAGE
} cur_page_t;

typedef enum {
    MACRO_MOVEMENT = 0,
    MICRO_MOVEMENT,
    HUMIDITY,
    TEMPERATURE,
    LIGHT_DENSITY,
    DISTANCE,
    OCC_STATE,
    MOTION_STATE,
	SETTINGS,
    CO2,
    VOC,
    PM10,
    PM2_5,
    PM1_0,
    CO,
    CO2_AP,
    CO_AP,
    HUMIDITY_AP,
    VOC_AP,
    PM_AP,
    STATUS_AP
}frame_type_t;


typedef struct slider_ctx
{
    char slider_name [32];
    int16_t value;
} slider_data_t;




void update_status_bar_data(frame_type_t frame_type, char* buf);

void dashboard_api_init(void);

void sliders_changed_cb(slider_data_t ctx);

bool is_slider_name(slider_data_t slider_dctx, char* name);

/**
 * Use this as an API for abstraction. This function does not require queue handle to be passed.
 * Instead is uses frame_type_t to evaluate which Queue to push data im.
*/
void send_data_fame(frame_type_t frame_type, uint32_t* buf, uint8_t elements, int isRealtime, int addnull);

/**
 * Wrapper for queue send function
*/
int push_data_to_queue(QueueHandle_t queue,uint8_t* data);

/**
 * Wrapper for queue receive function
*/
int pop_data_from_queue(QueueHandle_t queue, uint8_t* data);


/**
 * This function maps uint8_t array into json array string and will push data
 * to the Queue specified
*/
void create_data_frame(QueueHandle_t queue, uint8_t* buf, uint8_t elements);

/**
 * This function concatenets two JSON Array. (From Micromvement, Macromovement) and
 * forms single JSON string which then will be sent to websocket client.
*/
void get_json_str(char* json_str);

/**
 * Initializes all necesarry stuff to get the dashboard up and running
*/
void start_dashboard_server_component(httpd_handle_t server_handle);

void server_task(void* pvParameters);
void server_handle_task(void* pvParameters) ;
void count_task(void* pvParameters) ;

void set_current_page(cur_page_t cur_page);
cur_page_t get_current_page(void);

int copy_to_data_buffer(char* dst_buf,char* src_data);
void update_action_point(frame_type_t ftype,char* ap_str);
