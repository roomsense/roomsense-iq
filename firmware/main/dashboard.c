#include "esp_wifi.h"
#include "/home/builder/esp/esp-idf/components/lwip/lwip/src/include/lwip/sockets.h"
#include "dashboard.h"
#include "tasks_common.h"

char temprature_buf_realtime     [8] = {'0'};
char humidity_buf_realtime       [8] = {'0'};
char light_density_buf_realtime  [8] = {'0'};
char micro_movement_buf [456] = {0};
char macro_movement_buf [456] = {0};
char temprature_buf     [512] = {0};
char humidity_buf       [512] = {0};
char light_density_buf  [256] = {0};
char distance_buf       [256] = {0};
char motion_status_buf  [256] = {0};
char occ_status_buf     [16] = {0};
char settings_sync_buf  [256] = {0};

char CO2_buf   [256] = {0};
char CO_buf    [256] = {0};
char PM10_buf  [256] = {0};
char PM1_0_buf [256] = {0};
char PM2_5_buf [256] = {0};
char voc_buf   [256] = {0};

char CO2_buf_realtime   [8] = {0};
char CO_buf_realtime    [8] = {0};
char PM10_buf_realtime  [8] = {0};
char PM1_0_buf_realtime [8] = {0};
char PM2_5_buf_realtime [8] = {0};
char voc_buf_realtime   [8] = {0};

char co2_ap_buf     [100] = {0};
char co_ap_buf      [100] = {0};
char voc_ap_buf     [100] = {0};
char pm_ap_buf      [100] = {0};
char status_ap_buf  [100] = {0};
char humidity_ap_buf  [100] = {0};

int page_reload_event = false;
cur_page_t current_page = UNDEFINED_PAGE;

cur_page_t get_current_page(void){
    return current_page;
}

void set_current_page(cur_page_t cur_page){
    current_page = cur_page;
}

#define TAG "dashboard"
struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
};


extern httpd_handle_t http_server_handle ;
extern const uint8_t dashboard_html_start[]			        asm("_binary_dashboard_html_start");
extern const uint8_t dashboard_html_end[]		            asm("_binary_dashboard_html_end");
extern const uint8_t chart_js_start[]			            asm("_binary_chart_js_start");
extern const uint8_t chart_js_end[]		                    asm("_binary_chart_js_end");

extern const uint8_t climatesense_html_start []	               asm("_binary_climatesense_html_start");
extern const uint8_t climatesense_html_end  []	                asm("_binary_climatesense_html_end");

extern const uint8_t climatesense_js_start  []	                asm("_binary_climatesense_js_start");
extern const uint8_t climatesense_js_end  []	                  asm("_binary_climatesense_js_end");

extern const uint8_t climatesense_css_start []	                asm("_binary_climatesense_css_start");
extern const uint8_t climatesense_css_end  []	                 asm("_binary_climatesense_css_end");

extern const uint8_t navbar_css_start    []	                   asm("_binary_navbar_css_start");
extern const uint8_t navbar_css_end     []	                    asm("_binary_navbar_css_end");

extern const uint8_t slideswitch_css_start []	                 asm("_binary_slideswitch_css_start");
extern const uint8_t slideswitch_css_end  []	                  asm("_binary_slideswitch_css_end");

extern const uint8_t dashboard_charts_js_start[]	        asm("_binary_dashboard_charts_js_start");
extern const uint8_t dashboard_charts_js_end[]		        asm("_binary_dashboard_charts_js_end");

extern const uint8_t dashboard_charts_sliders_js_start[]	asm("_binary_dashboard_sliders_js_start");
extern const uint8_t dashboard_charts_sliders_js_end[]		asm("_binary_dashboard_sliders_js_end");

extern const uint8_t dashboard_css_start[]	                asm("_binary_dashboard_css_start");
extern const uint8_t dashboard_css_end[]		            asm("_binary_dashboard_css_end");

extern const uint8_t dashboard_statusbar_js_start[]         asm("_binary_dashboard_statusbar_js_start");
extern const uint8_t dashboard_statusbar_js_end[]           asm("_binary_dashboard_statusbar_js_end");

extern const uint8_t home_occupid_png_start[]               asm("_binary_home_occupid_png_start");
extern const uint8_t home_occupid_png_end[]                 asm("_binary_home_occupid_png_end");

extern const uint8_t home_unoccupid_png_start[]             asm("_binary_home_unoccupid_png_start");
extern const uint8_t home_unoccupid_png_end[]               asm("_binary_home_unoccupid_png_end");

extern const uint8_t motion_true_png_start[]                asm("_binary_motion_true_png_start");
extern const uint8_t motion_true_png_end[]                  asm("_binary_motion_true_png_end");

extern const uint8_t motion_false_png_start[]               asm("_binary_motion_false_png_start");
extern const uint8_t motion_false_png_end[]                 asm("_binary_motion_false_png_end");

extern const uint8_t RoomSense_Logo_png_start[]             asm("_binary_RoomSense_Logo_png_start");
extern const uint8_t RoomSense_Logo_png_end[]               asm("_binary_RoomSense_Logo_png_end");

extern const uint8_t dashboard_html_start[]			        asm("_binary_dashboard_html_start");
extern const uint8_t dashboard_html_end[]		            asm("_binary_dashboard_html_end");

/**
 * Wrapper for queue send function
*/
int copy_to_data_buffer(char* dst_buf,char* src_data){
	sprintf(dst_buf,"%s", src_data);
    return 0;
    //return xQueueSend(queue, data, 0);
}

void clear_th_buffers(void){
	uint32_t raw_data[2] = {0};
    send_data_fame(TEMPERATURE,raw_data,1, false, true);
    send_data_fame(HUMIDITY,raw_data,1, false, true);
    send_data_fame(SETTINGS,raw_data,1, false, true);
    send_data_fame(VOC,raw_data,1, false, true);
    send_data_fame(CO,raw_data,1, false, true);
    send_data_fame(CO2,raw_data,1, false, true);
    send_data_fame(PM10,raw_data,1, false, true);
    send_data_fame(PM1_0,raw_data,1, false, true);
    send_data_fame(PM2_5,raw_data,1, false, true);
}

/**
 * Wrapper for queue receive function
*/
int pop_data_from_queue(QueueHandle_t queue, uint8_t* data){
	return xQueueReceive(queue,data,0);
}

void update_status_bar_data(frame_type_t frame_type, char* buf){
    switch (frame_type)
    {
    case MOTION_STATE:
        copy_to_data_buffer(motion_status_buf,buf);
        break;
    case OCC_STATE :
        copy_to_data_buffer(occ_status_buf,buf);
        break;
    case HUMIDITY_AP:
        copy_to_data_buffer(humidity_ap_buf,buf);
        break;
    case CO2_AP:
        copy_to_data_buffer(co2_ap_buf,buf);
        break;

    case CO_AP:
        copy_to_data_buffer(co_ap_buf,buf);
        break;

    case VOC_AP:
        copy_to_data_buffer(voc_ap_buf,buf);
        break;

    case PM_AP:
        copy_to_data_buffer(pm_ap_buf,buf);
        break;

    case STATUS_AP:
        copy_to_data_buffer(status_ap_buf,buf);
        break;
    default:
        break;
    }
}
void update_action_point(frame_type_t ftype,char* ap_str){
	char str[256];
	sprintf(str,"\"%s\"",ap_str);
	update_status_bar_data(ftype,str);
}


void send_data_fame(frame_type_t frame_type, uint32_t* buf, uint8_t elements, int isRealtime,int addnull){
	char data[2024] = {0};
	if(addnull){
		sprintf(data,"%s", "\"NULL\"");
	}
	else {
		for (uint8_t i = 0; i < elements; i++)
		{
			if(i == (elements-1))sprintf(data + strlen(data),"%d", buf[i]);
			else sprintf(data + strlen(data),"%d,", buf[i]);
		}
	}

    switch (frame_type)
    {
    case MICRO_MOVEMENT:
        copy_to_data_buffer(micro_movement_buf, data);
        break;

    case MACRO_MOVEMENT:
        copy_to_data_buffer(macro_movement_buf,data);
        break;

    case HUMIDITY :
        if(isRealtime)
            copy_to_data_buffer(humidity_buf_realtime,data);
        else
            copy_to_data_buffer(humidity_buf,data);

        break;

    case TEMPERATURE :
        if(isRealtime)
            copy_to_data_buffer(temprature_buf_realtime,data);
        else
            copy_to_data_buffer(temprature_buf,data);
        break;

    case LIGHT_DENSITY :
        if(isRealtime)
            copy_to_data_buffer(light_density_buf_realtime,data);
        else
            copy_to_data_buffer(light_density_buf,data);;
        break;

    case DISTANCE :
        copy_to_data_buffer(distance_buf,data);
        break;

    case SETTINGS :
    	copy_to_data_buffer(settings_sync_buf,data);
    	break;

    case VOC :
        if(isRealtime)
        {
            copy_to_data_buffer(voc_buf_realtime,data);
        }
        else
        {
            copy_to_data_buffer(voc_buf,data);
        }
        break;

    case CO2 :
        if(isRealtime)
            copy_to_data_buffer(CO2_buf_realtime,data);
        else
            copy_to_data_buffer(CO2_buf,data);

        break;

    case CO :
        if(isRealtime)
            copy_to_data_buffer(CO_buf_realtime,data);
        else
            copy_to_data_buffer(CO_buf,data);

        break;

    case PM10 :
        if(isRealtime)
            copy_to_data_buffer(PM10_buf_realtime,data);
        else
            copy_to_data_buffer(PM10_buf,data);

        break;

    case PM1_0:
        if(isRealtime)
            copy_to_data_buffer(PM1_0_buf_realtime,data);
        else
            copy_to_data_buffer(PM1_0_buf,data);
        break;

    case PM2_5:
        if(isRealtime)
            copy_to_data_buffer(PM2_5_buf_realtime,data);
        else
            copy_to_data_buffer(PM2_5_buf,data);

        break;

    default:
        break;
    }
}


/**
 * This function concatenets two JSON Array. (From Micromvement, Macromovement) and
 * forms single JSON string which then will be sent to websocket client.
*/
void add_json_value(char* json_str, char* name, char* value, char json_char){
        switch (json_char)
        {
            case JSON_STRT :
                sprintf(json_str+strlen(json_str),"%c \"%s\" : [%s],",JSON_STRT,name,value);
                break;
            case JSON_CNT :
                sprintf(json_str+strlen(json_str)," \"%s\":[%s]%c", name, value, JSON_CNT);
                break;
            case JSON_END :
                sprintf(json_str+strlen(json_str)," \"%s\":[%s]%c",name, value,JSON_END);
                break;
            case JSON_TERMINATE :
                if(strlen(json_str) > 1){
                    sprintf(json_str+(strlen(json_str)-1)," %c",JSON_END);
                }
                break;
            default:
                break;
        }
}

void get_main_dashboard_json_str(char* json_str){
    add_json_value(json_str,"macromovement",macro_movement_buf,JSON_STRT);
    add_json_value(json_str,"micromovement",micro_movement_buf,JSON_CNT);

    add_json_value(json_str,"light_density",light_density_buf,JSON_CNT);
    add_json_value(json_str,"motion_status",motion_status_buf,JSON_CNT);
    add_json_value(json_str, "occ_status", occ_status_buf,JSON_CNT);
    add_json_value(json_str, "distance", distance_buf,JSON_CNT);

    add_json_value(json_str, "light_density_realtime", light_density_buf_realtime,JSON_CNT);
    add_json_value(json_str,"CO2_AP"      ,co2_ap_buf,JSON_CNT);
    add_json_value(json_str,"CO_AP"       ,co_ap_buf,JSON_CNT);
    add_json_value(json_str,"HUMIDITY_AP" ,humidity_ap_buf,JSON_CNT);
    add_json_value(json_str,"VOC_AP"      ,voc_ap_buf,JSON_CNT);
    add_json_value(json_str,"PM_AP"       ,pm_ap_buf,JSON_CNT);
    add_json_value(json_str,"STATUS_AP"   ,status_ap_buf,JSON_CNT);

	add_json_value(json_str,"temperature",temprature_buf,JSON_CNT);
	add_json_value(json_str,"humidity",humidity_buf,JSON_CNT);
    add_json_value(json_str, "VOC",   voc_buf,   JSON_CNT);
    add_json_value(json_str, "CO2",   CO2_buf,   JSON_CNT);
    add_json_value(json_str, "CO",    CO_buf,    JSON_CNT);
    add_json_value(json_str, "PM10",  PM10_buf,  JSON_CNT);
    add_json_value(json_str, "PM2_5", PM2_5_buf, JSON_CNT);
    add_json_value(json_str, "PM1_0", PM1_0_buf, JSON_CNT);
	add_json_value(json_str, "temperature_realtime", temprature_buf_realtime,JSON_CNT);
	add_json_value(json_str, "humidity_realtime", humidity_buf_realtime,JSON_CNT);

    add_json_value(json_str, "voc_realtime", voc_buf_realtime,JSON_CNT);
    add_json_value(json_str, "co2_realtime", CO2_buf_realtime,JSON_CNT);
    add_json_value(json_str, "co_realtime", CO_buf_realtime,JSON_CNT);
    add_json_value(json_str, "pm10_realtime", PM10_buf_realtime,JSON_CNT);
    add_json_value(json_str, "pm2_5_realtime", PM2_5_buf_realtime,JSON_CNT);
    add_json_value(json_str, "pm1_0_realtime", PM1_0_buf_realtime,JSON_CNT);



    add_json_value(json_str, "settings", settings_sync_buf,JSON_END);
//    printf("\r\n JSON : %s\r\n",json_str);
}

void get_climatesense_json_str(char* json_str){
    add_json_value(json_str,"type","\"climatesense\"",JSON_STRT);
	add_json_value(json_str,"temperature",temprature_buf,JSON_CNT);
	add_json_value(json_str,"humidity",humidity_buf,JSON_CNT);
    add_json_value(json_str, "VOC",   voc_buf,   JSON_CNT);
    add_json_value(json_str, "CO2",   CO2_buf,   JSON_CNT);
    add_json_value(json_str, "CO",    CO_buf,    JSON_CNT);
    add_json_value(json_str, "PM10",  PM10_buf,  JSON_CNT);
    add_json_value(json_str, "PM2_5", PM2_5_buf, JSON_CNT);
    add_json_value(json_str, "PM1_0", PM1_0_buf, JSON_CNT);
	add_json_value(json_str, "temperature_realtime", temprature_buf_realtime,JSON_CNT);
	add_json_value(json_str, "humidity_realtime", humidity_buf_realtime,JSON_END);
}

void concate_json(char* json_str){

}

static esp_err_t serve_roomsense_interface(httpd_req_t *req){
	httpd_resp_set_type(req, "text/html");
	httpd_resp_send(req, (const char *)dashboard_html_start, dashboard_html_end - dashboard_html_start);
	return ESP_OK;
}

static esp_err_t http_server_dashboard_statusbar_handler(httpd_req_t *req){
	ESP_LOGI(TAG, "dashboard_statusbar.js requested");
	httpd_resp_set_type(req, "application/javascript");
	httpd_resp_send(req, (const char *)dashboard_statusbar_js_start, dashboard_statusbar_js_end - dashboard_statusbar_js_start);
	return ESP_OK;
}


static esp_err_t serve_home_occupid_png(httpd_req_t *req){
    httpd_resp_set_type(req, "image/jpeg");
	httpd_resp_send(req, (const char *)home_occupid_png_start, home_occupid_png_end - home_occupid_png_start);

	return ESP_OK;
}

static esp_err_t serve_home_unoccupid_png(httpd_req_t *req){
  httpd_resp_set_type(req, "image/jpeg");
	httpd_resp_send(req, (const char *)home_unoccupid_png_start, home_unoccupid_png_end - home_unoccupid_png_start);

	return ESP_OK;
}

static esp_err_t serve_motion_sense_true_png(httpd_req_t *req){
    httpd_resp_set_type(req, "image/jpeg");
	httpd_resp_send(req, (const char *)motion_true_png_start, motion_true_png_end - motion_true_png_start);

	return ESP_OK;
}

static esp_err_t serve_motion_sense_false_png(httpd_req_t *req){
   httpd_resp_set_type(req, "image/jpeg");
	httpd_resp_send(req, (const char *)motion_false_png_start, motion_false_png_end - motion_false_png_start);

	return ESP_OK;
}

static esp_err_t serve_RoomSense_Logo_png(httpd_req_t *req){
   httpd_resp_set_type(req, "image/jpeg");
	httpd_resp_send(req, (const char *)RoomSense_Logo_png_start, RoomSense_Logo_png_end - RoomSense_Logo_png_start);

	return ESP_OK;
}


/**
 * dashboard.css get handler is requested when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_dashboard_css_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "dashboard_css requested");
	httpd_resp_set_type(req, "text/css");
	httpd_resp_send(req, (const char *)dashboard_css_start, dashboard_css_end - dashboard_css_start);

	return ESP_OK;
}


/**
 * chart.js get handler is requested when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_chart_js_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "chart.js requested");
	httpd_resp_set_type(req, "application/javascript");
	httpd_resp_send(req, (const char *)chart_js_start, chart_js_end - chart_js_start);
    page_reload_event = 2;
	return ESP_OK;
}


/**
 * dashboard_chart.js get handler is requested when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_dashboard_charts_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "dasboard_charts.js requested");

	httpd_resp_set_type(req, "application/javascript");
	httpd_resp_send(req, (const char *)dashboard_charts_js_start, dashboard_charts_js_end - dashboard_charts_js_start);
    current_page = MAIN_DASHBOARD_PAGE;
	return ESP_OK;
}


static esp_err_t http_server_climatesense_html_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "dashboard_css requested");
	httpd_resp_set_type(req, "text/html");
	httpd_resp_send(req, (const char *)climatesense_html_start, climatesense_html_end - climatesense_html_start);
	return ESP_OK;
}

static esp_err_t http_server_climatesense_js_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "dashboard_css requested");
	httpd_resp_set_type(req, "application/javascript");
	httpd_resp_send(req, (const char *)climatesense_js_start, climatesense_js_end - climatesense_js_start);
    current_page = CLIMATESENSE_PAGE;
	return ESP_OK;
}


static esp_err_t http_server_climatesense_css_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "climatesense_cssrequested");
	httpd_resp_set_type(req, "text/css");
	httpd_resp_send(req, (const char *)climatesense_css_start,climatesense_css_end - climatesense_css_start);
	return ESP_OK;
}

static esp_err_t http_server_navbar_css_handler(httpd_req_t *req)
{
	httpd_resp_set_type(req, "text/css");
		httpd_resp_send(req, (const char *)dashboard_css_start, dashboard_css_end - dashboard_css_start);

	ESP_LOGI(TAG, "navbar_css equested");
	httpd_resp_set_type(req, "text/css");
	httpd_resp_send(req, (const char *)navbar_css_start,navbar_css_end - navbar_css_start);
	return ESP_OK;
}

static esp_err_t http_server_slideswitch_css_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "slideswitch_css_ requested");
	httpd_resp_set_type(req, "text/css");
	httpd_resp_send(req, (const char *)slideswitch_css_start,slideswitch_css_end - slideswitch_css_start);
	return ESP_OK;
}

/**
 * In ESP-IDF websocket send invoked on reception of data from client.
 * here we can send data to be sent.
*/
#define WS_DBUF_SZ 1024*6

static char data[WS_DBUF_SZ]={0};
static void ws_async_send(void *arg)
{
	memset(data,0,WS_DBUF_SZ);
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;
	get_main_dashboard_json_str(data);
	clear_th_buffers();

	if(strlen((char*)data)< 10){
		sprintf(data,"%s","{ \"macromovement\" : [\"error\"],\"micromovement\" : [\"error\"]}");
	}
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)data;
    ws_pkt.len = strlen(data);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);

    //printf("finished transaciton\r\n");
    free(resp_arg);
}

/**
 * This method will send data over WS protocal asynchronously.
*/
static esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req)
{
    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);

    return httpd_queue_work(handle, ws_async_send, resp_arg);
}

void parse_sliders_values(char* json_str,slider_data_t* slider_data){
     /* Parse the JSON data */
        cJSON *json = cJSON_Parse(json_str);


        /* Extract the value and name from JSON */
        cJSON *valueJson = cJSON_GetObjectItemCaseSensitive(json, "value");
        cJSON *nameJson = cJSON_GetObjectItemCaseSensitive(json, "name");

        if (!cJSON_IsString(valueJson) || !cJSON_IsString(nameJson)) {
            cJSON_Delete(json);
            return;
        }

        slider_data->value       = atoi(valueJson->valuestring);
        sprintf(slider_data->slider_name, "%s", nameJson->valuestring);

        /* Handle the data as per your application needs */
        // Example: Print the received data
        printf("Received data: Name: %s, Value: %d\n", slider_data->slider_name, slider_data->value );

        cJSON_Delete(json);
}
/**
 * In ESP-IDF Websocke are Request-respose based.
 * Upon calling of  socket.send() from client side, this function will be handled.
 * When Received string is contains "data_request" we'll invoke void ws_async_send(void *arg)
*/
static esp_err_t handle_ws_req(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }

    if (ws_pkt.len)
    {
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }

    }

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char *)ws_pkt.payload, "data_request") == 0)
    {

        free(buf);
        return trigger_async_send(req->handle, req);
    }
    else {
        ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
       static slider_data_t slider_data ;
       parse_sliders_values((char*)ws_pkt.payload,&slider_data);
       sliders_changed_cb(slider_data);
       free(buf);
    }
    return ESP_OK;
}
/**
 * Initializes all necesarry stuff to get the dashboard up and running
*/
void start_dashboard_server_component(httpd_handle_t server_handle){
    dashboard_api_init();

    httpd_uri_t ws = {
				.uri        = "/ws",
				.method     = HTTP_GET,
				.handler    = handle_ws_req,
				.user_ctx   = NULL,
				.is_websocket = true
		};
    httpd_register_uri_handler(server_handle, &ws);

    httpd_uri_t room_sense_interface_uri = {
            .uri = "/dashboard.html",
            .method = HTTP_GET,
            .handler = serve_roomsense_interface,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &room_sense_interface_uri);

    httpd_uri_t climate_sense_html_uri = {
            .uri = "/climatesense.html",
            .method = HTTP_GET,
            .handler = http_server_climatesense_html_handler,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &climate_sense_html_uri);

    httpd_uri_t climate_sense_js_uri = {
                .uri = "/climatesense.js",
                .method = HTTP_GET,
                .handler = http_server_climatesense_js_handler,
                .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &climate_sense_js_uri);


    httpd_uri_t climate_sense_css_uri = {
            .uri = "/climatesense.css",
            .method = HTTP_GET,
            .handler = http_server_climatesense_css_handler,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &climate_sense_css_uri);

    httpd_uri_t nav_bar_css_uri = {
            .uri = "/navbar.css",
            .method = HTTP_GET,
            .handler = http_server_navbar_css_handler,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &nav_bar_css_uri);

    httpd_uri_t slide_switch_css_uri = {
            .uri = "/slideswitch.css",
            .method = HTTP_GET,
            .handler = http_server_slideswitch_css_handler,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &slide_switch_css_uri);

    // register chart.js handler
    httpd_uri_t chart_js = {
            .uri = "/chart.js",
            .method = HTTP_GET,
            .handler = http_server_chart_js_handler,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &chart_js);


    httpd_uri_t dashboard_charts_js = {
            .uri = "/dashboard_charts.js",
            .method = HTTP_GET,
            .handler = http_server_dashboard_charts_handler,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &dashboard_charts_js);

    httpd_uri_t dashboard_css = {
            .uri = "/dashboard.css",
            .method = HTTP_GET,
            .handler = http_server_dashboard_css_handler,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &dashboard_css);



    httpd_uri_t dashboard_statusbar = {
            .uri = "/dashboard_statusbar.js",
            .method = HTTP_GET,
            .handler = http_server_dashboard_statusbar_handler,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &dashboard_statusbar);


    httpd_uri_t serve_home_occupid = {
            .uri = "/home_occupid.png",
            .method = HTTP_GET,
            .handler = serve_home_occupid_png,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &serve_home_occupid);


    httpd_uri_t serve_home_unoccupid = {
            .uri = "/home_unoccupid.png",
            .method = HTTP_GET,
            .handler = serve_home_unoccupid_png,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &serve_home_unoccupid);

    httpd_uri_t serve_motion_sense_true = {
            .uri = "/motion_true.png",
            .method = HTTP_GET,
            .handler = serve_motion_sense_true_png,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &serve_motion_sense_true);

    httpd_uri_t serve_motion_sense_false = {
            .uri = "/motion_false.png",
            .method = HTTP_GET,
            .handler = serve_motion_sense_false_png,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &serve_motion_sense_false);

    httpd_uri_t serve_roomsense_logo_uri = {
            .uri = "/RoomSense_Logo.png",
            .method = HTTP_GET,
            .handler = serve_RoomSense_Logo_png,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(server_handle, &serve_roomsense_logo_uri);
}
