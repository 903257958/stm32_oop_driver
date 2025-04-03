#ifndef __ESP_H
#define __ESP_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "uart.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	
#else
	#error esp.h: No processor defined!
#endif

/* AT指令函数返回值 */
#define AT_RX_OK		0
#define AT_RX_ERROR		-1
#define AT_RX_TIMEOUT	-2

extern UARTDev_t esp_debug;	// ESP8266调试串口
extern UARTDev_t esp_usart;	// ESP8266通信串口，需配置为DMA+空闲中断

typedef struct ESPDev {
	bool init_flag;							// 初始化标志
	int (*send_cmd)(struct ESPDev *dev, const char *cmd, const char *ack, char *recv, uint16_t timeout);
	int (*reset)(struct ESPDev *dev);
	int (*connect_to_wifi)(struct ESPDev *dev, const char *ssid, const char *password);
	int (*deinit)(struct ESPDev *dev);		// 去初始化
}ESPDev_t;

int esp_init(ESPDev_t *dev);

#endif
