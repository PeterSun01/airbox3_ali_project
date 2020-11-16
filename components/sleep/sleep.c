#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sleep.h"
#include "Led.h"
#include "libGSM.h"
#include "PMS7003.h"
#include "E2prom.h"
#include "sht31.h"


//static const char *TAG = "SLEEP";


void goto_sleep(int time)
{
    GSM_PWR_Off();
    PM25_PWR_Off();
    //TVOC_PWR_Off();
    TVOC_PWR_On();
    IIC_PWR_Off();
    Led_B_Off();
    Led_R_Off();

    //const int wakeup_time_sec = 600;
    //const int wakeup_time_sec = 10;
    printf("Enabling timer wakeup, %ds\n", time);
    esp_sleep_enable_timer_wakeup(time * 1000000);
    printf("Entering deep sleep\n");
    esp_deep_sleep_start();
   
    esp_restart();//芯片复位 函数位于esp_system.h
}