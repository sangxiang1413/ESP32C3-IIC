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
* Date   : 2011.12.22
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


