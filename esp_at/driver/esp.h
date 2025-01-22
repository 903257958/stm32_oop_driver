#ifndef __ESP_H
#define __ESP_H

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef USART_TypeDef*			USARTx;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef USART_TypeDef*			USARTx;
	
#else
	#error esp.h: No processor defined!
#endif

#ifndef FREERTOS
	#define FREERTOS	0
#endif

#ifndef esp_log
	#define esp_log(x) 
#endif

/* AT指令函数返回值 */
#define AT_RX_OK		0
#define AT_RX_ERROR		-1
#define AT_RX_TIMEOUT	-2

extern USARTDev_t esp_debug;	// ESP8266调试串口
extern USARTDev_t esp_usart;	// ESP8266通信串口，需配置为DMA+空闲中断

typedef struct ESPDev {
	bool initFlag;							// 初始化标志
	int (*send_cmd)(struct ESPDev *pDev, const char *cmd, const char *ack, char *recv, uint16_t timeout);
	int (*reset)(struct ESPDev *pDev);
	int (*connect_to_wifi)(struct ESPDev *pDev, const char *ssid, const char *password);
	int (*deinit)(struct ESPDev *pDev);		// 去初始化
}ESPDev_t;

int esp_init(ESPDev_t *pDev);

#endif
