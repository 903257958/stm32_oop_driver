#ifndef __UART_H
#define __UART_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef USART_TypeDef*			UARTPER_t;
	typedef GPIO_TypeDef*			UARTGPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef USART_TypeDef*			UARTPER_t;
	typedef GPIO_TypeDef*			UARTGPIOPort_t;
	
#else
	#error uart.h: No processor defined!
#endif

typedef struct {
	UARTPER_t uartx;				// 串口外设
	uint32_t baud;					// 波特率
	UARTGPIOPort_t tx_port;			// 发送端口
	uint32_t tx_pin;				// 发送引脚
	UARTGPIOPort_t rx_port;			// 接收端口
	uint32_t rx_pin;				// 接收引脚
}UARTConfig_t;

typedef struct UARTDev {
	UARTConfig_t config;
	bool init_flag;																// 初始化标志
	bool dma_Flag;																// 使用DMA标志
	void *priv_data;															// 私有数据指针
	int (*dma_recv_enable)(struct UARTDev *dev);								// 开启DMA接收
	int (*send_byte)(struct UARTDev *dev, uint8_t byte);						// 发送一个字节
	int (*send_array)(struct UARTDev *dev, uint8_t *array, uint16_t length);	// 发送一个数组
	int (*send_string)(struct UARTDev *dev, char *str);							// 发送一个字符串
	int (*send_number)(struct UARTDev *dev, uint32_t num, uint8_t length);		// 发送一个数字
	int (*send_hex_packet)(struct UARTDev *dev, uint8_t *packet, uint8_t length, uint8_t head, uint8_t end);// 发送HEX数据包
	int (*printf)(struct UARTDev *dev, char *format, ...);						// 通用printf函数
	char *(*recv_string)(struct UARTDev *dev);									// 接收文本数据包
	uint8_t (*recv_string_flag)(struct UARTDev *dev);							// 接收文本数据包标志位
	int (*deinit)(struct UARTDev *dev);											// 去初始化
}UARTDev_t;

int uart_init(UARTDev_t *dev);
int uart_dma_init(UARTDev_t *dev);

#endif
