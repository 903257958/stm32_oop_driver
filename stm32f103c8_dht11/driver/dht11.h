#ifndef DHT11_H
#define DHT11_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
    typedef GPIO_TypeDef*	dht11_gpio_port_t;
    typedef uint32_t	    dht11_gpio_pin_t;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	dht11_gpio_port_t;
    typedef uint32_t	    dht11_gpio_pin_t;
	
#else
    #error dht11.h: No processor defined!
#endif

#include "delay.h"

#ifndef DHT11_DELAY_US
	#define DHT11_DELAY_US(us)	delay_us(us)
#endif
#ifndef DHT11_DELAY_MS
	#define DHT11_DELAY_MS(ms)	delay_ms(ms)
#endif

typedef struct {
	dht11_gpio_port_t port;	// 端口
    dht11_gpio_pin_t pin;	// 引脚
} dht11_config_t;

typedef struct {
	uint8_t temperature;    // 温度
    uint8_t humidity;       // 湿度
} dht11_data_t;

typedef struct dht11_dev {
    dht11_config_t config;
    dht11_data_t data;
	bool init_flag;                             // 初始化标志
	int8_t (*get_data)(struct dht11_dev *dev);  // 获取数据
	int8_t (*deinit)(struct dht11_dev *dev);    // 去初始化
} dht11_dev_t;

int8_t dht11_init(dht11_dev_t *dev);

#endif
