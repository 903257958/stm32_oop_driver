#ifndef ESP8266_DRV_H
#define ESP8266_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef USART_TypeDef*	uart_periph_t;
	typedef GPIO_TypeDef*	esp8266_gpio_port_t;
	typedef uint32_t		esp8266_gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef USART_TypeDef*	uart_periph_t;
	typedef GPIO_TypeDef*	esp8266_gpio_port_t;
	typedef uint32_t		esp8266_gpio_pin_t;

#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
   #include "gd32f10x.h"
	typedef uint32_t	uart_periph_t;
	typedef uint32_t	esp8266_gpio_port_t;
	typedef uint32_t	esp8266_gpio_pin_t;

#else
    #error esp8266.h: No processor defined!
#endif

#include "uart.h"
#include "delay.h"

#ifndef ESP8266_DELAY_MS
	#define ESP8266_DELAY_MS(ms)	delay_ms(ms)
#endif

/* ESP8266调试接口 */
#ifndef	ESP8266_DEBUG
	#if 1
	extern uart_dev_t debug;
	#define ESP8266_DEBUG(fmt, ...)	debug.printf(fmt, ##__VA_ARGS__)
	#else
	#define ESP8266_DEBUG(fmt, ...)
	#endif
#endif
#ifndef	ESP8266_SEND_DEBUG
	#if 0
	#define ESP8266_SEND_DEBUG(send)	ESP8266_DEBUG("ESP8266 send: %s\r\n", send)
	#else
	#define ESP8266_SEND_DEBUG(send)
	#endif
#endif
#ifndef	ESP8266_RECV_DEBUG
	#if 0
	#define ESP8266_RECV_DEBUG(recv)	ESP8266_DEBUG("ESP8266 recv: %s\r\n", recv)
	#else
	#define ESP8266_RECV_DEBUG(recv)
	#endif
#endif

typedef struct {
	uart_periph_t uartx;			// 串口外设
	esp8266_gpio_port_t rx_port;	// 接收端口（串口发送端口）
	esp8266_gpio_pin_t rx_pin;		// 接收引脚（串口发送引脚）
	esp8266_gpio_port_t tx_port;	// 发送端口（串口接收端口）
	esp8266_gpio_pin_t tx_pin;		// 发送引脚（串口接收引脚）
	char *wifi_inf0;				// WiFi信息
} esp8266_config_t;

typedef struct esp8266_dev {
	esp8266_config_t config;
	bool init_flag;								// 初始化标志
	void *priv_data;							// 私有数据指针
    int8_t (*send_cmd)(struct esp8266_dev *dev, char *cmd, char *res, char *recv_data);
	int8_t (*set_tcp_transparent)(struct esp8266_dev *dev, const char *ip, uint16_t port);
	int8_t (*exit_tcp_transparent)(struct esp8266_dev *dev);
    int8_t (*tcp_send_data)(struct esp8266_dev *dev, char *send_data);
    int16_t (*tcp_recv_data)(struct esp8266_dev *dev, char *recv_data, uint16_t len);
	int8_t (*deinit)(struct esp8266_dev *dev);  // 去初始化
} esp8266_dev_t;

int8_t esp8266_init(esp8266_dev_t *dev);

#endif
