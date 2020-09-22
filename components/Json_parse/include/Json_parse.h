#ifndef __Json_parse
#define __Json_parse
#include <stdio.h>
#include "esp_err.h"

typedef struct
{
    char creat_json_b[512];
    int creat_json_c;
} creat_json;


esp_err_t parse_objects_http_active(char *http_json_data);
esp_err_t parse_objects_mqtt(char *json_data);
esp_err_t parse_Uart0(char *json_data);
void create_mqtt_json(creat_json *pCreat_json);

int year,month,day,hour,min,sec;

#endif