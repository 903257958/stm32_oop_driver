#ifndef __ESP8266_H
#define __ESP8266_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "usart.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef USART_TypeDef*			USARTx;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef USART_TypeDef*			USARTx;
	
#else
	#error esp8266.h: No processor defined!
#endif

#ifndef FREERTOS
	#define FREERTOS	0
#endif

#if FREERTOS
	#include "FreeRTOS.h"
	#include "task.h"
#endif

#ifndef ESP8266_Log
	#define ESP8266_Log(x) 
#endif

#define BUFFER_LENGTH 50

typedef struct {
	USARTDev_t *pUSART;			// 通信串口
	USARTDev_t *pDebugUSART;	// 调试串口
	const char *ssid;			// WiFi名称
	const char *password;		// WiFi密码
	const char *ip;				// IP
	uint16_t port;				// 端口
}ESP8266Info_t;

typedef struct ESP8266Dev {
	ESP8266Info_t info;
	bool initFlag;																//初始化标志
	int (*send_cmd)(struct ESP8266Dev *pDev, const char *cmd, const char *ack, uint16_t timeout);
	int (*restore)(struct ESP8266Dev *pDev);
	int (*rst)(struct ESP8266Dev *pDev);
	int (*set_mode)(struct ESP8266Dev *pDev, uint8_t mode);
	int (*connect_to_wifi)(struct ESP8266Dev *pDev);
	int (*set_connection_mode)(struct ESP8266Dev *pDev, uint8_t mode);
	int (*start_tcp)(struct ESP8266Dev *pDev);
	int (*enable_transparent_mode)(struct ESP8266Dev *pDev);
	int (*debug)(struct ESP8266Dev *pDev, const char *data);
	int (*send_data)(struct ESP8266Dev *pDev, const char *data);
	bool (*recv_data_flag)(struct ESP8266Dev *pDev);
	int (*recv_data)(struct ESP8266Dev *pDev, char *recvStr);
	int (*config_tcp_passthrough)(struct ESP8266Dev *pDev);
	int (*deinit)(struct ESP8266Dev *pDev);										//去初始化
}ESP8266Dev_t;

int esp8266_init(ESP8266Dev_t *pDev);

#endif
