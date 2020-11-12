#ifndef __NV_S
#define __NV_S
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#define CO2_CAL_COUNT 70//每70次校准一次co2
void nvs_write(char *write_nvs, char *write_data);
esp_err_t nvs_read(char *read_nvs);

extern int32_t restart_counter;
#endif