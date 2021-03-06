#include <stdio.h>
#include <string.h>
#include <time.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "Nvs.h"
#include "Mqtt.h"
#include "Json_parse.h"
#include "Uart0.h"

#include "Led.h"
#include "E2prom.h"
#include "sleep.h"
#include "sht31.h"
#include "libGSM.h"
#include "PMS7003.h"


extern const int PPP_CONNECTED_BIT;
int32_t restart_counter = 0;
extern int32_t calibration_flag;
extern int32_t TVOC_sleep_pwrON_flag;
time_t sys_time;

void timer_periodic_cb(void *arg); 
esp_timer_handle_t timer_periodic_handle = 0; //定时器句柄

esp_timer_create_args_t timer_periodic_arg = {
    .callback =
        &timer_periodic_cb, 
    .arg = NULL,            
    .name = "PeriodicTimer" 
};

void timer_periodic_cb(void *arg) //1ms中断一次
{
  static int64_t timer_count = 0;
  timer_count++;
  if(calibration_flag==1)//开机需要重新校准CO2传感器
  {
    if (timer_count >= 1200000) //校准时最大20min
    {
        timer_count = 0;
        printf("[APP] Free memory: %d bytes\n", esp_get_free_heap_size());
        goto_sleep(update_time*60);
    }
  }
  else
  {
    if (timer_count >= 240000) //非校准时最大4min
    {
        timer_count = 0;
        printf("[APP] Free memory: %d bytes\n", esp_get_free_heap_size());
        goto_sleep(update_time*60);
    }
  }
}

void read_flash_usr(void)
{
  esp_err_t err;
  // Open
  printf("Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) 
  {
      printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } 
  else 
  {
      printf("Done\n");
      printf("Reading restart counter from NVS ... ");
      err = nvs_get_i32(my_handle, "TVOC_PWR", &TVOC_sleep_pwrON_flag);
      switch (err) 
      {
          case ESP_OK:
            printf("Done\n");
            printf("TVOC_PWR = %d\n", TVOC_sleep_pwrON_flag);
            break;
          case ESP_ERR_NVS_NOT_FOUND://烧写程序后第一次开机,则清空eeprom，重新烧写序列号
            printf("The first time start after flash! default 0(OFF)\n");
            TVOC_sleep_pwrON_flag=0;
            err = nvs_set_i32(my_handle, "TVOC_PWR", TVOC_sleep_pwrON_flag);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
          default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
      }

      printf("Reading update time from NVS ... ");
      err = nvs_get_i32(my_handle, "update_time", &update_time);
      switch (err) 
      {
          case ESP_OK:
            printf("Done\n");
            printf("update_time = %d\n", update_time);
            break;
          case ESP_ERR_NVS_NOT_FOUND://未配置，写入10分钟默认
            printf("The first time start after flash! default 10 min\n");
            update_time=10;
            err = nvs_set_i32(my_handle, "update_time", update_time);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
          default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
      }
      nvs_close(my_handle);
  }
}

void write_flash_updatetime(uint16_t time)
{
  esp_err_t err;
  // Open
  printf("Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) 
  {
      printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } 
  else 
  {
      printf("Done\n");
      // Write
      err = nvs_set_i32(my_handle, "update_time", time);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      printf("Committing updates in NVS ... ");
      err = nvs_commit(my_handle);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      nvs_close(my_handle);
  }
}

void write_flash_TVOC_PWR(uint16_t flag)
{
  esp_err_t err;
  // Open
  printf("Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) 
  {
      printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } 
  else 
  {
      printf("Done\n");
      // Write
      err = nvs_set_i32(my_handle, "TVOC_PWR", flag);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      printf("Committing updates in NVS ... ");
      err = nvs_commit(my_handle);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      nvs_close(my_handle);
  }
}


static void Uart0_Task(void* arg)
{
    while(1)
    {
        Uart0_read();
        vTaskDelay(50 / portTICK_RATE_MS);
    }  
}


void app_main(void)
{
  ESP_ERROR_CHECK(nvs_flash_init());
  read_flash_usr();

  time(&sys_time);
  printf("timestamps=%ld\n",sys_time);
  if(sys_time<3)//从没电到上电，先进行TVOC 15min预热校准
  {
    calibration_flag=1;
  }
  if(sys_time<=14400)//上电开始前4H，休眠时TVOC电源一直开启
  {
    TVOC_sleep_pwrON_flag=1;
  }

  Led_Init();
  PM25_PWR_GPIO_Init();
  PM25_PWR_Off();
  vTaskDelay(1000 / portTICK_RATE_MS);
  ESP_LOGI("MAIN", "[APP] IDF version: %s", esp_get_idf_version());
  i2c_init();
  Uart0_Init();
  xTaskCreate(Uart0_Task, "Uart0_Task", 4096, NULL, 10, NULL);
  
  
  /*******************************timer 1s init**********************************************/
  esp_err_t err = esp_timer_create(&timer_periodic_arg, &timer_periodic_handle);
  err = esp_timer_start_periodic(timer_periodic_handle, 1000); //创建定时器，单位us，定时1ms
  if (err != ESP_OK)
  {
    printf("timer periodic create err code:%d\n", err);
  }

  /*step1 判断是否有ProductKey/DeviceName/DeviceSecret****/
  E2prom_Read(PRODUCTKEY_ADDR,(uint8_t *)ProductKey,PRODUCTKEY_LEN);
  printf("ProductKey=%s\n", ProductKey);

  E2prom_Read(DEVICENAME_ADDR,(uint8_t *)DeviceName,DEVICENAME_LEN);
  printf("DeviceName=%s\n", DeviceName);

  E2prom_Read(DEVICESECRET_ADDR,(uint8_t *)DeviceSecret,DEVICESECRET_LEN);
  printf("DeviceSecret=%s\n", DeviceSecret); 


  if(((strlen(DeviceName)==0)||(strlen(DeviceName)==0)||(strlen(DeviceSecret)==0)) || ((ProductKey[0]==0xff)&&(ProductKey[1]==0xff)&&(ProductKey[2]==0xff)&&(DeviceSecret[2]==0xff)))
  {
    printf("no ProductKey/DeviceName/DeviceSecret!\n");
    Led_Status=LED_STA_NOSER;

    while(1)
    {
      //故障
      vTaskDelay(500 / portTICK_RATE_MS);
    }
  }

  PM25_Init();
  if(calibration_flag==1)
  {
    ESP_LOGW("MAIN", "wait calibration then pppos\n");
  }
  //阻塞等待TVOC传感器测量结束
  xEventGroupWaitBits(PM25_event_group, PM25_COMPLETE_BIT , false, true, portMAX_DELAY); 
  ppposInit(); 
  //阻塞等待ppp连接成功
  xEventGroupWaitBits(ppp_event_group, PPP_CONNECTED_BIT , false, true, portMAX_DELAY); 
  vTaskDelay(500 / portTICK_RATE_MS);
  initialise_mqtt();

}
