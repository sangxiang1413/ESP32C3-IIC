/************************************************************************************
*  
*           
* File name: AHT10.c
* Project  : OLED
* Processor: ESP32C3
* Compiler :xsten gcc
* 
* Author : Yingxiang Li
* Version: 1.00
* Date   : 2022.12.22
* Email  : feng13114@gmail.com
* Modification: none


*************************************************************************************/
#include <stdio.h>
#include "driver/i2c.h"
#include "hal/i2c_types.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

extern esp_err_t I2C_Write(uint8_t slaveAddr, uint8_t *TxBuf, size_t dataLen);
extern esp_err_t I2C_Read(uint8_t slaveAddr, uint8_t *RxBuf, size_t dataLen);
extern volatile uint8_t g_u8RxData[];
extern volatile uint8_t g_u8TxData[];

float temperature;
float  humidness;	


#define AHT10_ADDR						0x38

#define CMD_AHT10_INIT					0xe1
#define CMD_TRIGGER_MEASUREMENTS 		0xac
#define CMD_SOFT_RESET					0xba


#define BUSY_INDICATION_Pos				(7)
#define BUSY_INDICATION_Msk				(1 << BUSY_INDICATION_Pos)

#define MODE_STATUS_MSK_Pos				(5)
#define MODE_STATUS_MSK					(3 << MODE_STATUS_MSK_Pos)

#define CAL_ENABLE_MSK_Pos				(3)
#define CAL_ENABLE_MSK					(1 << CAL_ENABLE_MSK_Pos)

#define DEVICE_BUSY						1
#define	DEVICE_IDEL						0
#define	WORK_IN_NOR_MODE				0
#define	WORK_IN_CYC_MODE				1
#define	WORK_IN_CMD_MODE				2		//both of 2 and 3, all is cmd mode  
#define HAS_CAL							1
#define	NO_CAL							0

#define	CMD_TRIG_MEASUR_WAIT_TIME		75		//ms min 75
#define	CMD_SOFT_RESET_WAIT_TIME		20		//ms max 20

#define DEBUG_AHT10

void AHT10_Init(void)
{
	vTaskDelay((40) / portTICK_RATE_MS);
	g_u8TxData[0] = CMD_SOFT_RESET;
	I2C_Write(AHT10_ADDR, (uint8_t *)&g_u8TxData, 1);
	vTaskDelay((CMD_SOFT_RESET_WAIT_TIME) / portTICK_RATE_MS);
}

void get_temp_hum(void)
{
	uint8_t status;
	int32_t temp1, temp2;
	g_u8TxData[0] = CMD_TRIGGER_MEASUREMENTS;
	g_u8TxData[1] = 0x33;
	g_u8TxData[2] = 0x00;
	I2C_Write(AHT10_ADDR, (uint8_t *)&g_u8TxData, 3);
	vTaskDelay((CMD_TRIG_MEASUR_WAIT_TIME + 20) / portTICK_RATE_MS);
	I2C_Read(AHT10_ADDR, (uint8_t *)&g_u8RxData, 1);
	status = g_u8RxData[0];
	
	if(status >> MODE_STATUS_MSK_Pos !=  DEVICE_IDEL)
	{
#ifdef	DEBUG_AHT10
		printf("The data is not ready!\r\n");	
#endif
		I2C_Read(AHT10_ADDR, (uint8_t *)&g_u8RxData, 1);
		status = g_u8RxData[0];
		
	}

	I2C_Read(AHT10_ADDR, (uint8_t *)&g_u8RxData, 6);
	temp1 = (((g_u8RxData[3] & (~(0xf0))) << 16) | (g_u8RxData[4] << 8) | g_u8RxData[5]);
	temp2 = ((g_u8RxData[1] << 12) | (g_u8RxData[2] << 4) | (g_u8RxData[3] >> 4));
	
 	
 	temperature = (float)((temp1 / pow(2, 20)) * 200 - 50);
	humidness = (float)((temp2 /  pow(2, 20)) * 100)/100;

	
#ifdef	DEBUG_AHT10
	printf("\r\ntemp1: 0x%x temp2: 0x%x \r\n", temp1,  temp2);
	printf("\r\ntemperature: %.2f humidness: %.2f \r\n", temperature,  humidness);
#endif
 	
}



/*
typedef struct _BASIC_CMD{
	
}BASIC_CMD;
*/
