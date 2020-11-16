
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "math.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sht31.h"
#include "PMS7003.h"

static char tag[] = "SHT31";

esp_err_t sht31_reset() 
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SHT31_ADDRESS << 1) | I2C_MASTER_WRITE,
	SHT31_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x30, SHT31_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0xA2, SHT31_ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd,1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret == ESP_FAIL) 
	{
		ESP_LOGE("SHT30", "SHT31_RESET : failed");
		return ESP_FAIL;
	}
	return ESP_OK;
}

#define GPIO_IIC_PWR	4

void IIC_PWR_GPIO_Init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 <<GPIO_IIC_PWR);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);  
}

void IIC_PWR_On(void)
{
    gpio_set_level(GPIO_IIC_PWR, 1);
}

void IIC_PWR_Off(void)
{
	gpio_set_level(GPIO_IIC_PWR, 0);
}

void i2c_init() 
{
	IIC_PWR_GPIO_Init();
	TVOC_PWR_GPIO_Init();//开启SHT30电源
	TVOC_PWR_On();
	IIC_PWR_On();

	vTaskDelay(100 / portTICK_PERIOD_MS);
	int i2c_master_port = I2C_NUM;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_SDA_IO;
	conf.scl_io_num = I2C_SCL_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_FREQ_HZ;
	ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, 
									   I2C_RX_BUF_DISABLE, 
									   I2C_TX_BUF_DISABLE, 0));
	vTaskDelay(500 / portTICK_PERIOD_MS);

	//sht31_reset();
	//vTaskDelay(10 / portTICK_PERIOD_MS);
}


double sht31_readTemperature() 
{

	return temp;
}

double sht31_readHumidity() 
{

	return humidity;
}

bool sht31_readTempHum() 
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SHT31_ADDRESS << 1) | I2C_MASTER_WRITE,
	SHT31_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x24, SHT31_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x00, SHT31_ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd,
			1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret == ESP_FAIL) {
		ESP_LOGI(tag, "0x2400 Failed");
		return false;
	}

	vTaskDelay(500 / portTICK_PERIOD_MS);

	uint8_t readbuffer[6];

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SHT31_ADDRESS << 1) | I2C_MASTER_READ,
	SHT31_ACK_CHECK_EN);

	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 1, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 2, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 3, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 4, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 5, SHT31_NACK_VAL));

	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	if (ret == ESP_FAIL) {
		ESP_LOGI(tag, "reading Failed");
		return false;
	}

	//ESP_LOGI(tag, "HEX : %x %x %x %x %x %x", readbuffer[0], readbuffer[1],
	//		readbuffer[2], readbuffer[3], readbuffer[4], readbuffer[5]);

	uint16_t ST, SRH;
	ST = readbuffer[0];
	ST <<= 8;
	ST |= readbuffer[1];

	if (readbuffer[2] != sht31_crc8(readbuffer, 2)) {
		ESP_LOGI(tag, "crc8 : 0 Failed");
		return false;
	}

	SRH = readbuffer[3];
	SRH <<= 8;
	SRH |= readbuffer[4];

	if (readbuffer[5] != sht31_crc8(readbuffer + 3, 2)) {
		ESP_LOGI(tag, "crc8 : 1 Failed");
		return false;
	}

	double stemp = ST;
	stemp *= 175;
	stemp /= 0xffff;
	stemp = -45 + stemp;
	int temp_int=stemp*10;
	temp = (double)temp_int/10;	
	//ESP_LOGW("SHT30", "stemp=%f", stemp);
	//ESP_LOGE("SHT30", "temp=%f", temp);

	double shum = SRH;
	shum *= 100;
	shum /= 0xFFFF;
	int humidity_int=shum*10;
	humidity = (double)humidity_int/10;
	//ESP_LOGW("SHT30", "shum=%f", shum);
	//ESP_LOGE("SHT30", "humidity=%f", humidity);

	return true;
}

uint8_t sht31_crc8(const uint8_t *data, int len) {
	/*
	 *
	 * CRC-8 formula from page 14 of SHT spec pdf
	 *
	 * Test data 0xBE, 0xEF should yield 0x92
	 *
	 * Initialization data 0xFF
	 * Polynomial 0x31 (x8 + x5 +x4 +1)
	 * Final XOR 0x00
	 */

	const uint8_t POLYNOMIAL = 0x31;
	uint8_t crc = 0xFF;

	for (int j = len; j; --j) {
		crc ^= *data++;

		for (int i = 8; i; --i) {
			crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
		}
	}
	return crc;
}
