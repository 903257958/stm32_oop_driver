#ifndef UART_DRV_H
#define UART_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef USART_TypeDef*	uart_periph_t;
	typedef GPIO_TypeDef*	uart_gpio_port_t;
	typedef uint32_t		uart_gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef USART_TypeDef*	uart_periph_t;
	typedef GPIO_TypeDef*	uart_gpio_port_t;
	typedef uint32_t		uart_gpio_pin_t;
	
#else
	#error uart.h: No processor defined!
#endif

/* 索引结构体数组长度 */
#define INDEX_BUF_NUM	10

/* 串口接收索引结构体 */
typedef struct {
    uint8_t *start;	// 数据段起始地址
    uint8_t *end;	// 数据段结束地址
} uart_rx_index_t;

/* 串口接收控制块结构体 */
typedef struct {
    uint16_t data_cnt;							// 已接收数据计数
    uart_rx_index_t index_buf[INDEX_BUF_NUM];	// 索引数组，用于管理多段数据
    uart_rx_index_t *index_in;					// 写入指针
    uart_rx_index_t *index_out;					// 读出指针
    uart_rx_index_t *index_end;					// 索引数组结束位置
} uart_rx_cb_t;

/* 串口配置结构体 */
typedef struct {
	uart_periph_t uartx;		// 串口外设
	uint32_t baud;				// 波特率
	uart_gpio_port_t tx_port;	// 发送端口
	uart_gpio_pin_t tx_pin;		// 发送引脚
	uart_gpio_port_t rx_port;	// 接收端口
	uart_gpio_pin_t rx_pin;		// 接收引脚
    uint8_t *tx_buf;            // 发送数据缓冲区
	uint8_t *rx_buf;            // 接收数据缓冲区
	uint16_t tx_buf_size;		// 发送数据缓冲区大小
	uint16_t rx_buf_size;		// 接收数据缓冲区大小
	uint16_t rx_single_max;		// 单次接收最大数据量
} uart_config_t;

/* 串口设备结构体 */
typedef struct uart_dev {
	uart_config_t config;
	bool init_flag;								// 初始化标志
	uart_rx_cb_t rx_cb;							// 接收数据控制块
	void (*printf)(char *format, ...);			// 格式化打印
	void (*send)(uint8_t *data, uint32_t len);	// 发送数据
	char *(*recv)(void);						// 接收数据
	int8_t (*deinit)(struct uart_dev *dev);		// 去初始化
} uart_dev_t;

int8_t uart_init(uart_dev_t *dev);

#endif
