#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sht31.h"
#include "PMS7003.h"
#include "Nvs.h"

#define GPIO_TVOC_PWR	14
#define UART2_TXD  (GPIO_NUM_26)
#define UART2_RXD  (GPIO_NUM_25)
#define UART2_RTS  (UART_PIN_NO_CHANGE)
#define UART2_CTS  (UART_PIN_NO_CHANGE)
#define BUF_SIZE    100

static const char *TAG = "PM25_TVOC";
void UART2_Read_Task(void* arg);
extern void write_flash_calibration(void);
int32_t calibration_flag=0;


void TVOC_PWR_GPIO_Init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 <<GPIO_TVOC_PWR);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);  
}

void TVOC_PWR_On(void)
{
    gpio_set_level(GPIO_TVOC_PWR, 1);
}

void TVOC_PWR_Off(void)
{
    gpio_set_level(GPIO_TVOC_PWR, 0);
}

#define GPIO_PM25_PWR	27

void PM25_PWR_GPIO_Init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << GPIO_PM25_PWR);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);  
}

void PM25_PWR_On(void)
{
    gpio_set_level(GPIO_PM25_PWR, 1);
}

void PM25_PWR_Off(void)
{
    gpio_set_level(GPIO_PM25_PWR, 0);
}

void PM25_Init(void)
{
    PM25_PWR_GPIO_Init();
    //TVOC_PWR_GPIO_Init();
    TVOC_PWR_Off();

    //PM25_PWR_On();
    //vTaskDelay(1000 / portTICK_RATE_MS);
    //PM25_PWR_Off();
    vTaskDelay(500 / portTICK_RATE_MS);
    PM25_PWR_On();

    //配置GPIO
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask =  1 << UART2_RXD;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, UART2_TXD, UART2_RXD, UART2_RTS, UART2_CTS);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);

    //const char Send_TVOC_passive[]={0x42,0x78,0x04,0x00,0x00,0x00,0x00,0x00,0xff};
    //const char Send_TVOC_passive[]={0X42,0X4d,0XE1,0X00,0X00,0X01,0X70};
    //uart_write_bytes(UART_NUM_2, Send_TVOC_passive, sizeof(Send_TVOC_passive));

    xTaskCreate(&UART2_Read_Task, "UART2_Read_Task", 2048, NULL, 10, NULL);

    PM25_event_group = xEventGroupCreate();
}


static uint8_t	Check_PMSensor_DataValid(uint8_t* PM_Sensor_RxBuffer)
{
	uint16_t 	Cal_CheckSum;
	uint16_t 	Buffer_CheckSum;
	uint16_t 	Buffer_Len;
	uint8_t 	i;
	uint8_t     Result = 0;

	if((PM_Sensor_RxBuffer[0] == 'B')&&(PM_Sensor_RxBuffer[1] == 'M'))
	{
		Buffer_Len = (uint16_t)((PM_Sensor_RxBuffer[2] << 8) | PM_Sensor_RxBuffer[3]);

		Buffer_CheckSum = (uint16_t)((PM_Sensor_RxBuffer[Buffer_Len + 2] << 8) | PM_Sensor_RxBuffer[Buffer_Len + 3]);

		Cal_CheckSum = 0;
		for(i=0;i<(Buffer_Len + 2);i++)
		{
			Cal_CheckSum += PM_Sensor_RxBuffer[i];
		}
		if(Cal_CheckSum == Buffer_CheckSum)
			Result = 1;
	}
	return Result;
}


uint16_t CRC_Compute ( uint8_t *arr_buff, uint8_t len)
{
    uint16_t crc=0xFFFF;
    uint8_t i, j;
    for ( j=0; j <len;j++)
    {
        crc=crc ^*arr_buff++;
        for ( i=0; i<8; i++)
        {
            if((crc&0x0001)>0)
            {
                crc=crc>>1;
                crc=crc^ 0xa001;
            }
            else
                crc=crc>>1;
        }
    }

    //printf("crc=%04x\r\n",crc);
    return ( crc);
}


void UART2_Read_Task(void* arg)
{
    uint8_t data_u2[BUF_SIZE];
    static uint16_t data_count=0;
    static uint32_t CO2_aver=0;
    static uint64_t count=1;
    while(1)
    {
        //printf("conut=%lld",count);
        if(count>1000)
        {
            count=0;
            data_count=0;
            PM25_PWR_Off();
            vTaskDelay(200 / portTICK_PERIOD_MS);
            TVOC_PWR_On();
            vTaskDelay(200 / portTICK_PERIOD_MS);
            sht31_reset();
            vTaskDelay(10 / portTICK_PERIOD_MS);

        }
        int len1 = uart_read_bytes(UART_NUM_2, data_u2, BUF_SIZE, 20 / portTICK_RATE_MS);
        if(len1!=0)  //读取到传感器数据
        {
            for(int i=0;i<len1;i++)
            {
                printf("0x%02x ",data_u2[i]);
            }
            puts("\n");
            
            if(len1==17)//TVOC数据
            {
                if(CRC_Compute(data_u2+2,13)==(uint16_t)data_u2[16]*256+(uint16_t)data_u2[15])//数据校验成功
                {
                    //puts("crc ok\n");
                    data_count++;
                    CO2 = (uint16_t)((data_u2[9]<<8) | data_u2[10]);
                    TVOC = (uint16_t)((data_u2[11]<<8) | data_u2[12]);
                    HCHO = (uint16_t)((data_u2[13]<<8) | data_u2[14]);
                    printf("CO2=%dppm,TVOC=%dppb,HCHO=%dppb,data_count=%d,CO2_aver=%d\n", CO2,TVOC,HCHO,data_count,CO2_aver);
                    //if((restart_counter%CO2_CAL_COUNT==0)|(calibration_flag==0))//每70次CO2连续开启10分钟，重新校准CO2传感器
                    // if(calibration_flag==0)//每70次CO2连续开启10分钟，重新校准CO2传感器
                    // {
                    //     if((data_count>=590)&&(data_count<600))
                    //     {
                    //         CO2_aver=CO2_aver+CO2;
                    //     }
                    //     if(data_count==600)//结束测量，进行上传
                    //     {
                    //         CO2=CO2_aver/10;
                    //         ESP_LOGI(TAG, "final CO2=%dppm,TVOC=%dppb,HCHO=%dppb,data_count=%d,CO2_aver=%d", CO2,TVOC,HCHO,data_count,CO2_aver);
                    //         xEventGroupSetBits(PM25_event_group, PM25_COMPLETE_BIT);
                    //         write_flash_calibration();
                    //         vTaskDelete( NULL );  //任务删除
                    //     }
                    // }
                    // else
                    {
                        if((data_count>=20)&&(data_count<30))
                        {
                            CO2_aver=CO2_aver+CO2;
                        }
                        if(data_count==30)//结束测量，进行上传
                        {
                            CO2=CO2_aver/10;
                            ESP_LOGI(TAG, "final CO2=%dppm,TVOC=%dppb,HCHO=%dppb,data_count=%d,CO2_aver=%d", CO2,TVOC,HCHO,data_count,CO2_aver);
                            xEventGroupSetBits(PM25_event_group, PM25_COMPLETE_BIT);
                            vTaskDelete( NULL );  //任务删除
                        }
                    }
                }
            }

            else if(len1==32)//PM25数据
            {
                if(Check_PMSensor_DataValid(data_u2)==1)//数据校验成功
                {
                    PM2_5  = (uint16_t)((data_u2[12]<<8) | data_u2[13]);
                    ESP_LOGI(TAG, "PM2_5=%d,data_count=%d", PM2_5, data_count);
                    data_count++;
                    if(data_count>=25)
                    {
                        data_count=0;
                        count=0;
                        PM25_PWR_Off();
                        vTaskDelay(200 / portTICK_PERIOD_MS);
                        TVOC_PWR_On();
                        vTaskDelay(200 / portTICK_PERIOD_MS);
                        sht31_reset();
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                    }
                }
            }
            len1=0;
            bzero(data_u2,sizeof(data_u2));                 
        } 
        if(count>0)
        { 
            count++;
        }
        vTaskDelay(5 / portTICK_RATE_MS);
    }   
}


