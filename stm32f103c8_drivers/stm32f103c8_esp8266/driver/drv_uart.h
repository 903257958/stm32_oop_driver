#ifndef DRV_UART_H
#define DRV_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
#define DRV_UART_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef USART_TypeDef*			uart_periph_t;
typedef DMA_Channel_TypeDef*	dma_channel_t;
typedef GPIO_TypeDef*			gpio_port_t;
typedef uint32_t				gpio_pin_t;
typedef IRQn_Type				iqrn_type_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_UART_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef USART_TypeDef*      uart_periph_t;
typedef uint32_t	        dma_channel_t;
typedef DMA_Stream_TypeDef* dma_stream_t;
typedef GPIO_TypeDef*       gpio_port_t;
typedef uint32_t            gpio_pin_t;
typedef IRQn_Type           iqrn_type_t;

#elif defined(GD32F10X_MD)
#define DRV_UART_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t			uart_periph_t;
typedef dma_channel_enum	dma_channel_t;
typedef uint32_t			gpio_port_t;
typedef uint32_t			gpio_pin_t;
typedef IRQn_Type			iqrn_type_t;

#else
#error drv_uart.h: No processor defined!
#endif

#ifndef EAGAIN
#define EAGAIN	11
#endif

/* 配置结构体 */
typedef struct {
	uart_periph_t uart_periph;
	uint32_t 	  baudrate;
	gpio_port_t   tx_port;
	gpio_pin_t 	  tx_pin;
	gpio_port_t   rx_port;
	gpio_pin_t    rx_pin;
    uint8_t 	 *tx_buf;
	uint8_t 	 *rx_buf;
	uint16_t 	  tx_buf_size;
	uint16_t 	  rx_buf_size;	// 必须 >= rx_single_max + 1
	uint16_t      rx_single_max;
	uint8_t 	  rx_pre_priority;
	uint8_t 	  rx_sub_priority;
} uart_cfg_t;

typedef struct uart_dev uart_dev_t;

/* 操作接口结构体 */
typedef struct {
	void (*vprintf)(uart_dev_t *dev, const char *format, va_list args);
	void (*printf)(uart_dev_t *dev, const char *format, ...);
	int (*send_data)(uart_dev_t *dev, uint8_t *data, uint32_t len);
	int (*recv_str)(uart_dev_t *dev, char *str);
	int (*recv_data)(uart_dev_t *dev, uint8_t **data, uint32_t *len);
	int (*deinit)(uart_dev_t *dev);
} uart_ops_t;

/* 设备结构体 */
struct uart_dev {
	void *priv;
	uart_cfg_t cfg;
	const uart_ops_t *ops;
};

/**
 * @brief   初始化串口设备驱动
 * @details 只能配置支持 DMA 的串口，配置为 DMA 接收
 * @param[out] dev uart_dev_t 结构体指针
 * @param[in]  cfg uart_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
int drv_uart_init(uart_dev_t *dev, const uart_cfg_t *cfg);

#endif
