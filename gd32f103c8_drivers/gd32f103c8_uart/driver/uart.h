#ifndef UART_DRV_H
#define UART_DRV_H

#include <stdint.h>
#include <stdbool.h>

#ifdef USE_STDPERIPH_DRIVER
	
#if defined (GD32F10X_MD)
    #include "gd32f10x.h"
    typedef uint32_t	uart_periph_t;
    typedef uint32_t	gpio_port_t;
	typedef uint32_t	gpio_pin_t;
	
#else
	#error uart.h: No processor defined!
#endif

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
	uart_periph_t uartx;
	uint32_t baud;
	gpio_port_t tx_port;
	gpio_pin_t tx_pin;
	gpio_port_t rx_port;
	gpio_pin_t rx_pin;
    uint8_t *tx_buf;
	uint8_t *rx_buf;
	uint16_t tx_buf_size;
	uint16_t rx_buf_size;
	uint16_t rx_single_max; // 单次接收最大数据量
} uart_config_t;

/* 串口设备结构体 */
typedef struct uart_dev {
	uart_config_t config;
	bool init_flag;
	uart_rx_cb_t rx_cb;
	void (*printf)(char *format, ...);
	void (*send)(uint8_t *data, uint32_t len);
	char *(*recv)(void);
	int (*deinit)(struct uart_dev *dev);
} uart_dev_t;

int uart_drv_init(uart_dev_t *dev);

#endif
