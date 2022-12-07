/************************************************************************************
*  
*           
* File name: main.c
* Project  : OLED
* Processor: ESP32C3
* Compiler :xsten gcc
* 
* Author : Yingxiang Li
* Version: 1.00
* Date   : 2011.12.21
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




#define I2C_MASTER_SCL_IO    19    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    18   /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ   400000
//#define portMAX_DELAY		500
#define Rx_BUF_LEN 			64
#define Tx_BUF_LEN			64
#define NACK				0
#define ACK				1


int i2c_master_port = 0;
#define I2C_MASTER_NUM 0

i2c_config_t I2C_CFG;
SemaphoreHandle_t  print_mux = NULL;
SemaphoreHandle_t  i2c_mux = NULL;

uint8_t   g_str_buf[256];
uint8_t   g_dbg_buf[256];

volatile uint8_t g_u8RxData[Rx_BUF_LEN];
volatile uint8_t g_u8TxData[Tx_BUF_LEN];

extern void OLED_Init(void);
extern void OLED_P16x16Ch(unsigned char x, unsigned char y, unsigned char N);
extern void OLED_P8x16Str(unsigned char x, unsigned char y, unsigned char ch[]);
extern void OLED_CLS(void);
extern void Draw_BMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[]);
extern void OLED_P6x8Str(unsigned char x, unsigned char y, unsigned char ch[]);
extern	void oled_run(void);


#define  unlock_i2c()     do{\
         xSemaphoreGive((xSemaphoreHandle)i2c_mux);\
         }while(0)

#define  lock_i2c()       do{\
         xSemaphoreTake((xSemaphoreHandle)i2c_mux, portMAX_DELAY);\
         }while(0)



#define  lock_uart()       do{\
         xSemaphoreTake((xSemaphoreHandle)print_mux, portMAX_DELAY);\
         }while(0)

#define  unlock_uart()       do{\
         xSemaphoreGive((xSemaphoreHandle)print_mux);\
         }while(0)

static void  my_debug(char *fmt, ...)
{
      va_list v_args;
	  
	  xSemaphoreTake((xSemaphoreHandle)print_mux, portMAX_DELAY);
	  va_start(v_args, fmt);
	  (void) vsnprintf( (char *) &g_dbg_buf[0],
	  	                (size_t) sizeof(g_dbg_buf),
	  	                (char const *) fmt,
	  	                              v_args);
	  va_end(v_args);
      printf("%s", g_dbg_buf);
	  xSemaphoreGive((xSemaphoreHandle)print_mux);	  
}



static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

/*
void Init_I2C(void)
{
	I2C_CFG = i2c_master_init();
	//ESP_ERROR_CHECK(I2C_CFG);
	i2c_driver_install(I2C_NUM_0, I2C_CFG.mode, 0, 0, 0);
	i2c_param_config(I2C_NUM_0, &I2C_CFG);
	
}

*/
esp_err_t I2C_Write(uint8_t slaveAddr, uint8_t *TxBuf, size_t dataLen)
{
	esp_err_t err;
    i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | I2C_MASTER_WRITE, ACK);
    i2c_master_write(cmd, TxBuf, dataLen, ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

esp_err_t I2C_Read(uint8_t slaveAddr, uint8_t *RxBuf, size_t dataLen)
{
	esp_err_t err;
	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (slaveAddr << 1) | I2C_MASTER_READ, ACK);
	i2c_master_read(cmd, RxBuf, dataLen - 1, ACK);
	i2c_master_read(cmd, RxBuf + dataLen - 1, 1, NACK);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);

	i2c_cmd_link_delete(cmd);
	
	return err;
	
}
static void print_task(void *arg)
{
    //int ret;
    //uint32_t task_idx = (uint32_t)arg;

    while (1) 
	{
    	my_debug("Hello world!\r\n"); 
		vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}
static void I2C_TASK(void *arg)
{
	my_debug("I2C WRITE TASK IS RUNNING\r\n");
	i2c_master_init();
	OLED_Init(); //OLED³õÊ¼»¯
	oled_run();

	vTaskDelete(NULL);
}


void app_main(void)
{
	//Init_I2C();
	print_mux = xSemaphoreCreateMutex();
	i2c_mux  = xSemaphoreCreateMutex();
	
	if(print_mux != NULL && i2c_mux != NULL)
	{
		xTaskCreate(print_task, "print_task", 1024 * 2, (void *)0, 10, NULL);
		xTaskCreate(I2C_TASK, "I2C1_TASK", 1024 * 2, (void *)0, 10, NULL);
	}

	
}
