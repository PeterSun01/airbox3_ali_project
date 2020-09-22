#ifndef _E2PROM_H_
#define _E2PROM_H_

#include "freertos/FreeRTOS.h"

  /*
  EEPROM PAGE0 
    0x00 ProductKey      (16byte)   -------烧录获得，鉴权使用
    0x10 DeviceName    (32byte)   --------烧录获得，鉴权使用
    0x30 DeviceSecret   (32byte)   -------烧录获得，鉴权使用

  EEPROM PAGE1
    0X00 none
  */
#define PRODUCTKEY_ADDR       0x00
#define DEVICENAME_ADDR       0x10
#define DEVICESECRET_ADDR     0x30



#define PRODUCTKEY_LEN       16
#define DEVICENAME_LEN       32
#define DEVICESECRET_LEN     32


char ProductKey[PRODUCTKEY_LEN+1];
char DeviceName[DEVICENAME_LEN+1];
char DeviceSecret[DEVICESECRET_LEN+1];





extern void E2prom_Init(void);
extern int E2prom_Write(uint8_t addr,uint8_t*data_write,int len);
extern int E2prom_Read(uint8_t addr,uint8_t*data_read,int len);
extern int E2prom_BluWrite(uint8_t addr,uint8_t*data_write,int len);
extern int E2prom_BluRead(uint8_t addr,uint8_t*data_read,int len);
#endif

