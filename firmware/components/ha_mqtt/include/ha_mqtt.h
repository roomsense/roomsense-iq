
#define MAX_MQTT_HOST_LENGTH     		64
#define MAX_MQTT_PORT_LENGTH            16
#define MAX_USERNAME_LENGTH				32
#define MAX_MQTT_PASSWORD_LENGTH		64


typedef struct {
    char mqtt_host[MAX_MQTT_HOST_LENGTH];
    char mqtt_port[MAX_MQTT_PORT_LENGTH];
    char mqtt_username[MAX_USERNAME_LENGTH];
    char mqtt_password[MAX_MQTT_PASSWORD_LENGTH];
} mqtt_credentials_t;


/**
 * Message IDs for the MQTT application task
 * @note Expand this based on your application requirements.
 */
typedef enum mqtt_app_message
{
	   MQTT_APP_MSG_START_HTTP_SERVER = 0,
	   MQTT_APP_MSG_START_MQTT_CLIENT,
	   MQTT_APP_MSG_CONNECTING_FROM_HTTP_SERVER,
	   MQTT_APP_MSG_LOAD_SAVED_CREDENTIALS,
       MQTT_APP_MSG_CONNECTED,
       MQTT_APP_MSG_NEW_CREDENTIALS,
       MQTT_APP_MSG_DISCONNECTED,
	   MQTT_APP_MSG_USER_REQUESTED_DISCONNECT,
	   MQTT_APP_MSG_RESTART_CPU,
} mqtt_app_message_e;

/**
 * Structure for the message queue
 * @note Expand this based on application requirements e.g. add another type and parameter as required
 */
typedef struct mqtt_app_queue_message
{
       mqtt_app_message_e msgID;
} mqtt_app_queue_message_t;


typedef enum occupancy_state
{
	VACANT= 0,
	OCCUPIED,
} occupancy_state_e;

/**
 * Sends a message to the queue
 * @param msgID message ID from the mqtt_app_message_e enum.
 * @return pdTRUE if an item was successfully sent to the queue, otherwise pdFALSE.
 * @note Expand the parameter list based on your requirements e.g. how you've expanded the mqtt_app_queue_message_t.
 */
BaseType_t mqtt_app_send_message(mqtt_app_message_e msgID);

void ha_mqtt_start(void);
mqtt_credentials_t* get_mqtt_credentials(void);
bool toggle_value(bool trigger);
