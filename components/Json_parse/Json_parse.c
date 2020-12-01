#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdlib.h>

#include <cJSON.h>
#include "esp_system.h"
#include "Json_parse.h"
#include "Nvs.h"
#include "esp_log.h"

#include "E2prom.h"
#include "sht31.h"
#include "sleep.h"
#include "PMS7003.h"
#include "libGSM.h"

extern void write_flash_updatetime(uint16_t time);
extern void write_flash_TVOC_PWR(uint16_t flag);

esp_err_t parse_Uart0(char *json_data)
{
    cJSON *json_data_parse = NULL;
    cJSON *json_data_parse_Command = NULL;
    cJSON *json_data_parse_ProductKey = NULL;
    cJSON *json_data_parse_DeviceName = NULL;
    cJSON *json_data_parse_DeviceSecret = NULL;
    cJSON *json_data_parse_UpdateTime = NULL;

    if(json_data[0]!='{')
    {
        printf("uart0 Json Formatting error1\n");
        return 0;
    }

    json_data_parse = cJSON_Parse(json_data);
    if (json_data_parse == NULL) //如果数据包不为JSON则退出
    {
        printf("uart0 Json Formatting error\n");
        cJSON_Delete(json_data_parse);
        return 0;
    }
    else
    {
        json_data_parse_Command = cJSON_GetObjectItem(json_data_parse, "Command"); 
        /*
        {
            "Command": "SetupALi",
            "ProductKey": "a18hJfuxArE",
            "DeviceName": "AIR2V001",
            "DeviceSecret": "kc0tfij2RbXdbOmiHSXnwmaZgR3CDE85"
        }
        */
        if(strcmp(json_data_parse_Command->valuestring,"SetupALi")==0)//烧录阿里云三元组
        {
            char zero_data[256];
            bzero(zero_data,sizeof(zero_data));
            
            json_data_parse_ProductKey = cJSON_GetObjectItem(json_data_parse, "ProductKey");
            if(json_data_parse_ProductKey!=NULL) 
            {
                E2prom_Write(PRODUCTKEY_ADDR, (uint8_t *)zero_data, PRODUCTKEY_LEN);            
                sprintf(ProductKey,"%s%c",json_data_parse_ProductKey->valuestring,'\0');
                E2prom_Write(PRODUCTKEY_ADDR, (uint8_t *)ProductKey, strlen(ProductKey));            
                printf("ProductKey= %s\n", json_data_parse_ProductKey->valuestring);
            }

            json_data_parse_DeviceName = cJSON_GetObjectItem(json_data_parse, "DeviceName"); 
            if(json_data_parse_DeviceName!=NULL) 
            {
                E2prom_Write(DEVICENAME_ADDR, (uint8_t *)zero_data, DEVICENAME_LEN);   
                sprintf(DeviceName,"%s%c",json_data_parse_DeviceName->valuestring,'\0');
                E2prom_Write(DEVICENAME_ADDR, (uint8_t *)DeviceName, strlen(DeviceName)); 
                printf("DeviceName= %s\n", json_data_parse_DeviceName->valuestring);
            }

            json_data_parse_DeviceSecret = cJSON_GetObjectItem(json_data_parse, "DeviceSecret"); 
            if(json_data_parse_DeviceSecret!=NULL) 
            {
                E2prom_Write(DEVICESECRET_ADDR, (uint8_t *)zero_data, DEVICESECRET_LEN);   
                sprintf(DeviceSecret,"%s%c",json_data_parse_DeviceSecret->valuestring,'\0');
                E2prom_Write(DEVICESECRET_ADDR, (uint8_t *)DeviceSecret, strlen(DeviceSecret));
                printf("DeviceSecret= %s\n", json_data_parse_DeviceSecret->valuestring);
            }  

       }
        /*
        {
            "Command": "UpdateTime",
            "value": 10
        }
        */
        else if(strcmp(json_data_parse_Command->valuestring,"UpdateTime")==0)//设置更新时间
        {
            json_data_parse_UpdateTime = cJSON_GetObjectItem(json_data_parse, "value"); 
            write_flash_updatetime(json_data_parse_UpdateTime->valueint);
        }
        /*
        {
            "Command": "TVOC_ON",
            "value": 0
        }
        */
        else if(strcmp(json_data_parse_Command->valuestring,"TVOC_ON")==0)//设置TVOC在休眠时是否打开电源，0=关，1=开
        {
            json_data_parse_UpdateTime = cJSON_GetObjectItem(json_data_parse, "value"); 
            write_flash_TVOC_PWR(json_data_parse_UpdateTime->valueint);
        }
        
        printf("{\"status\":\"success\",\"err_code\": 0}");
        cJSON_Delete(json_data_parse);
        fflush(stdout);//使stdout清空，就会立刻输出所有在缓冲区的内容。
        esp_restart();//芯片复位 函数位于esp_system.h
        return 1;
    }
}


/*
{
	"method": "thing.event.property.post",
	"params": {
		"Temperature": 24.3,
		"Humidity": 34.9

	}
}
*/

void create_mqtt_json(creat_json *pCreat_json)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *params = cJSON_CreateObject();
    

    cJSON_AddItemToObject(root, "method", cJSON_CreateString("thing.event.property.post"));
    cJSON_AddItemToObject(root, "params", params);

    if (sht31_readTempHum()) 
    {		
        double Temperature = sht31_readTemperature();
        double Humidity = sht31_readHumidity();

        cJSON_AddItemToObject(params, "Temperature", cJSON_CreateNumber(Temperature));
        cJSON_AddItemToObject(params, "Humidity", cJSON_CreateNumber(Humidity)); 
        ESP_LOGI("SHT30", "Temperature=%.1f, Humidity=%.1f", Temperature, Humidity);
	} 

    cJSON_AddItemToObject(params, "CO2", cJSON_CreateNumber(CO2));
    cJSON_AddItemToObject(params, "TVOC", cJSON_CreateNumber(TVOC));
    cJSON_AddItemToObject(params, "HCHO", cJSON_CreateNumber(HCHO));
    cJSON_AddItemToObject(params, "Battery", cJSON_CreateNumber(pwr_int));
    cJSON_AddItemToObject(params, "PM25", cJSON_CreateNumber(PM2_5));
    cJSON_AddItemToObject(params, "RSSI", cJSON_CreateNumber(csq_int));
    
    char *cjson_printunformat;
    cjson_printunformat=cJSON_PrintUnformatted(root);
    pCreat_json->creat_json_c=strlen(cjson_printunformat);
    bzero(pCreat_json->creat_json_b,sizeof(pCreat_json->creat_json_b));
    memcpy(pCreat_json->creat_json_b,cjson_printunformat,pCreat_json->creat_json_c);
    printf("len=%d,mqtt_json=%s\n",pCreat_json->creat_json_c,pCreat_json->creat_json_b);
    free(cjson_printunformat);
    cJSON_Delete(root);
}


/*
{
	"method": "thing.service.property.set",
	"id": "216373306",
	"params": {
		"Switch_JDQ": 0
	},
	"version": "1.0.0"
}
*/


esp_err_t parse_objects_mqtt(char *json_data)
{
    cJSON *json_data_parse = NULL;
    cJSON *json_data_parse_method = NULL;
    cJSON *json_data_parse_params = NULL;

    cJSON *iot_params_parse = NULL;
    cJSON *iot_params_parse_Switch_JDQ = NULL;

    json_data_parse = cJSON_Parse(json_data);

    if(json_data[0]!='{')
    {
        printf("mqtt Json Formatting error\n");

        return 0;       
    }

    if (json_data_parse == NULL) //如果数据包不为JSON则退出
    {

        printf("Json Formatting error1\n");
        cJSON_Delete(json_data_parse);
        return 0;
    }
    else
    {
        json_data_parse_method = cJSON_GetObjectItem(json_data_parse, "method"); 
        printf("method= %s\n", json_data_parse_method->valuestring);

        json_data_parse_params = cJSON_GetObjectItem(json_data_parse, "params"); 
        char *iot_params;
        iot_params=cJSON_PrintUnformatted(json_data_parse_params);
        printf("cJSON_Print= %s\n",iot_params);
      
        free(iot_params);
        cJSON_Delete(iot_params_parse);
    }
    
    cJSON_Delete(json_data_parse);
    

    return 1;
}
