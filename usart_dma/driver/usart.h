#ifndef __USART_H
#define __USART_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef USART_TypeDef*			USARTx;
	typedef GPIO_TypeDef*			USART_GPIO_Port;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef USART_TypeDef*			USARTx;
	typedef GPIO_TypeDef*			USART_GPIO_Port;
	
#else
	#error usart.h: No processor defined!
#endif

#ifndef usart_log
	#define usart_log(x)
#endif

typedef struct {
	USARTx usartx;																// 串口外设
	uint32_t baud;																// 波特率
}USARTInfo_t;

typedef struct USARTDev {
	USARTInfo_t info;
	bool initFlag;																// 初始化标志
	bool DMAFlag;																// 使用DMA标志
	void *pPrivData;															// 私有数据指针
	int (*dma_recv_enable)(struct USARTDev *pDev);								// 开启DMA接收
	int (*send_byte)(struct USARTDev *pDev, uint8_t byte);						// 发送一个字节
	int (*send_array)(struct USARTDev *pDev, uint8_t *array, uint16_t length);	// 发送一个数组
	int (*send_string)(struct USARTDev *pDev, char *str);						// 发送一个字符串
	int (*send_number)(struct USARTDev *pDev, uint32_t num, uint8_t length);	// 发送一个数字
	int (*send_hex_packet)(struct USARTDev *pDev, uint8_t *packet, uint8_t length, uint8_t head, uint8_t end);// 发送HEX数据包
	int (*printf)(struct USARTDev *pDev, char *format, ...);					// 通用printf函数
	uint8_t (*recv_byte)(struct USARTDev *pDev);								// 接收一个字节
	uint8_t (*recv_byte_flag)(struct USARTDev *pDev);							// 接收一个字节标志位
	char *(*recv_string)(struct USARTDev *pDev);								// 接收文本数据包
	uint8_t (*recv_string_flag)(struct USARTDev *pDev);							// 接收文本数据包标志位
	uint8_t *(*recv_hex_packet)(struct USARTDev *pDev);							// 接收HEX数据包
	uint8_t (*recv_hex_packet_flag)(struct USARTDev *pDev);						// 接收HEX数据包标志位
	int (*deinit)(struct USARTDev *pDev);										// 去初始化
}USARTDev_t;

int usart_init(USARTDev_t *pDev);
int usart_dma_init(USARTDev_t *pDev);

#endif
