#ifndef  _MY_DEBUG_H_
#define _MY_DEBUG_H_
#include <stdio.h>
#include "driver/i2c.h"
#include "hal/i2c_types.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

SemaphoreHandle_t  print_mux = NULL;
SemaphoreHandle_t  i2c_mux = NULL;

uint8_t   g_str_buf[256];
uint8_t   g_dbg_buf[256];

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

#endif

