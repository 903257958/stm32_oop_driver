#ifndef UART_H
#define UART_H

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

/* 串口使能配置 */
#define UART1_ENABLE	1
#define UART2_ENABLE	1
#define UART3_ENABLE	1
#define UART4_ENABLE	0
#define UART5_ENABLE	0
#define UART6_ENABLE	0

/* 发送与接收缓冲区长度 */
#define UART1_TX_SIZE	2048
#define UART1_RX_SIZE	2048
#define UART2_TX_SIZE	2048
#define UART2_RX_SIZE	2048
#define UART3_TX_SIZE	2048
#define UART3_RX_SIZE	2048
#define UART4_TX_SIZE	2048
#define UART4_RX_SIZE	2048
#define UART5_TX_SIZE	2048
#define UART5_RX_SIZE	2048
#define UART6_TX_SIZE	2048
#define UART6_RX_SIZE	2048

/* 单次接收最大长度 */
#define UART1_RX_MAX	512
#define UART2_RX_MAX	512
#define UART3_RX_MAX	512
#define UART4_RX_MAX	512
#define UART5_RX_MAX	512
#define UART6_RX_MAX	512

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
} uart_config_t;

/* 串口设备结构体 */
typedef struct uart_dev {
	uart_config_t config;
	bool init_flag;								// 初始化标志
	void (*printf)(char *format, ...);			// 格式化打印
	void (*send)(uint8_t *data, uint32_t len);	// 发送数据
	char *(*recv)(void);						// 接收数据
	int8_t (*deinit)(struct uart_dev *dev);		// 去初始化
} uart_dev_t;

/* 缓冲区与接收控制块定义 */
#if UART1_ENABLE
extern uint8_t uart1_tx_buf[UART1_TX_SIZE];
extern uint8_t uart1_rx_buf[UART1_RX_SIZE];
extern uart_rx_cb_t uart1_rx_cb;
#endif
#if UART2_ENABLE
extern uint8_t uart2_tx_buf[UART2_TX_SIZE];
extern uint8_t uart2_rx_buf[UART2_RX_SIZE];
extern uart_rx_cb_t uart2_rx_cb;
#endif
#if UART3_ENABLE
extern uint8_t uart3_tx_buf[UART3_TX_SIZE];
extern uint8_t uart3_rx_buf[UART3_RX_SIZE];
extern uart_rx_cb_t uart3_rx_cb;
#endif
#if UART4_ENABLE
extern uint8_t uart4_tx_buf[UART4_TX_SIZE];
extern uint8_t uart4_rx_buf[UART4_RX_SIZE];
extern uart_rx_cb_t uart4_rx_cb;
#endif
#if UART5_ENABLE
extern uint8_t uart5_tx_buf[UART5_TX_SIZE];
extern uint8_t uart5_rx_buf[UART5_RX_SIZE];
extern uart_rx_cb_t uart5_rx_cb;
#endif
#if UART6_ENABLE
extern uint8_t uart6_tx_buf[UART6_TX_SIZE];
extern uint8_t uart6_rx_buf[UART6_RX_SIZE];
extern uart_rx_cb_t uart6_rx_cb;
#endif

int8_t uart_init(uart_dev_t *dev);

#endif
