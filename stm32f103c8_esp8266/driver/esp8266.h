#ifndef __ESP8266_H
#define __ESP8266_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef GPIO_TypeDef*	ESP8266GPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	ESP8266GPIOPort_t;

#else
    #error esp8266.h: No processor defined!
#endif

/* 调试接口 */
#if 1
	#include "uart.h"
	extern UARTDev_t debug;
	#define ESP8266_DEBUG(fmt, ...)	debug.printf(fmt, ##__VA_ARGS__)
#else
	#define ESP8266_DEBUG(str)
#endif

/* WiFi信息 */
#define ESP8266_WIFI_INFO	"AT+CWJAP=\"shouji\",\"thxd156369\"\r\n"

typedef struct ESP8266Dev {
	bool init_flag;							// 初始化标志
	void *priv_data;						// 私有数据指针
	int8_t (*clear)(struct ESP8266Dev *dev);
    int8_t (*send_cmd)(struct ESP8266Dev *dev, char *cmd, char *res);
    int8_t (*send_data)(struct ESP8266Dev *dev, unsigned char *data, unsigned short len);
    uint8_t *(*get_ipd)(struct ESP8266Dev *dev, unsigned short timeout);
	int8_t (*get_beijing_time)(struct ESP8266Dev *dev, uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second);
	int8_t (*get_weather)(struct ESP8266Dev *dev);
	int8_t (*deinit)(struct ESP8266Dev *dev);  // 去初始化
}ESP8266Dev_t;

int8_t esp8266_init(ESP8266Dev_t *dev);
void esp8266_uart_irq_callback(void);

#endif
