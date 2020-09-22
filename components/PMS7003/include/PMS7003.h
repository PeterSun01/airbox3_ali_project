
#ifndef _PMS7003_H_
#define _PMS7003_H_
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

uint16_t PM2_5;
uint16_t CO2;
uint16_t TVOC;
uint16_t HCHO;

extern void PM25_Init(void);
extern void PM25_PWR_Off(void);
extern void TVOC_PWR_On(void);
extern void TVOC_PWR_GPIO_Init(void);


EventGroupHandle_t PM25_event_group;
static const int PM25_COMPLETE_BIT = BIT0;


#endif

