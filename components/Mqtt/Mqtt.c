#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"


#include "mqtt_client.h"
#include "Mqtt.h"
#include "Json_parse.h"
#include "E2prom.h"
#include "Led.h"
#include "libGSM.h"
#include "PMS7003.h"
#include "hmac_sha1.h"
#include "sleep.h"

#define MQTT_JSON_TYPE  0X03

static const char *TAG = "MQTT";
extern const int PPP_CONNECTED_BIT;
extern const int PM25_COMPLETE_BIT;

esp_mqtt_client_handle_t client;
EventGroupHandle_t mqtt_event_group;
static const int MQTT_CONNECTED_BIT = BIT0;

char Topic_Set[256];
char Topic_Post[256];

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;

    switch (event->event_id)
    {
        case MQTT_EVENT_BEFORE_CONNECT:
            break;
            
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

            
           
            msg_id = esp_mqtt_client_subscribe(client, Topic_Set, 0);//订阅
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            xEventGroupClearBits(mqtt_event_group, MQTT_CONNECTED_BIT);
            goto_sleep(SHORT_SLEEP_TIME);
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);  
            xEventGroupSetBits(mqtt_event_group, MQTT_CONNECTED_BIT);    
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGW(TAG, "MQTT_EVENT_DATA=%s",event->data);
            //ESP_LOGW(TAG, "MQTT_EVENT_user_context=%s",event->user_context);
            ESP_LOGW(TAG, "MQTT_EVENT_topic=%s",event->topic);            

            parse_objects_mqtt(event->data);//收到平台MQTT数据并解析
            bzero(event->data,strlen(event->data));
            Mqtt_Send_Msg(Topic_Post);
            
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
    }
    return ESP_OK;
}

void Mqtt_Send_Msg(char* topic)
{
    int msg_id;
    creat_json *pCreat_json=malloc(sizeof(creat_json));
    create_mqtt_json(pCreat_json);

    msg_id = esp_mqtt_client_publish(client, topic, pCreat_json->creat_json_b, pCreat_json->creat_json_c, 0, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    Led_Status=LED_STA_SENDDATA;

    free(pCreat_json);
}




static void MqttSend_Task(void* arg)
{
    static uint8_t send_count=0;
    while(1)
    {
        xEventGroupWaitBits(ppp_event_group, PPP_CONNECTED_BIT , false, true, portMAX_DELAY); 
        xEventGroupWaitBits(mqtt_event_group, MQTT_CONNECTED_BIT , false, true, portMAX_DELAY); 
        xEventGroupWaitBits(PM25_event_group, PM25_COMPLETE_BIT , false, true, portMAX_DELAY); 
        Mqtt_Send_Msg(Topic_Post);
        send_count++;
        if(send_count>=2)
        {
            send_count=0;
            vTaskDelay(3000 / portTICK_RATE_MS);
            goto_sleep(update_time*60);
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    
}

void initialise_mqtt(void)
{
    char mqtt_url[100]="\0";
    char mqtt_usrname[100]="\0";
    char mqtt_clientid[100]="\0";
    char mqtt_content[100]="\0";
    char mqtt_passwd[100]="\0";

    sprintf(mqtt_url,"mqtt://mqtt.iot-as-mqtt.cn-shanghai.aliyuncs.com");
    printf("mqtt_url=%s\n",mqtt_url);

    sprintf(mqtt_usrname,"%s&%s",DeviceName,ProductKey);
    printf("mqtt_usrname=%s\n",mqtt_usrname);

    sprintf(mqtt_clientid,"%s|securemode=3,signmethod=hmacsha1|",DeviceName);
    printf("mqtt_clientid=%s\n",mqtt_clientid);

    sprintf(mqtt_content,"clientId%sdeviceName%sproductKey%s",DeviceName,DeviceName,ProductKey);
    printf("mqtt_content=%s\n",mqtt_content);
    printf("DeviceSecret=%s\n",DeviceSecret);
    aliyun_iot_common_hmac_sha1(mqtt_content, strlen(mqtt_content), mqtt_passwd, DeviceSecret, strlen(DeviceSecret));
    printf("mqtt_passwd=%s\n",mqtt_passwd);

    //Topic_Set = "/sys/ProductKey/DeviceName/thing/service/property/set"           
    sprintf(Topic_Set,"/sys/%s/%s/thing/service/property/set",ProductKey,DeviceName);
    printf("Topic_Set=%s\n",Topic_Set);
    //Topic_Post=/sys/ProductKey/DeviceName/thing/event/property/post
    sprintf(Topic_Post,"/sys/%s/%s/thing/event/property/post",ProductKey,DeviceName);
    printf("Topic_Post=%s\n",Topic_Post);

    const esp_mqtt_client_config_t mqtt_cfg = {

        .uri = mqtt_url,
        .port = 1883,
        .event_handle = mqtt_event_handler,
        .username = mqtt_usrname,
        .password = mqtt_passwd,
        .client_id = mqtt_clientid,
        .keepalive = 300
    };

    mqtt_event_group = xEventGroupCreate();

    xEventGroupWaitBits(ppp_event_group, PPP_CONNECTED_BIT , false, true, portMAX_DELAY);
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
    xTaskCreate(MqttSend_Task, "MqttSend_Task", 8192, NULL, 4, NULL);
}
