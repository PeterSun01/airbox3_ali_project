
#ifndef _SLEEP_H_
#define _SLEEP_H_

#include "freertos/FreeRTOS.h"

#define LONG_SLEEP_TIME 300
#define SHORT_SLEEP_TIME 5

extern void goto_sleep(int time);

extern int32_t update_time;

#endif

