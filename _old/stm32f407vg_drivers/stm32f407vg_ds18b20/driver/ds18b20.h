#ifndef DS18B20_DRV_H
#define DS18B20_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
    typedef GPIO_TypeDef*	gpio_port_t;
    typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	gpio_port_t;
    typedef uint32_t		gpio_pin_t;
	
#else
    #error ds18b20.h: No processor defined!
#endif

#endif

#include "delay.h"

#ifndef DS18B20_DELAY_US
	#define DS18B20_DELAY_US(us)	delay_us(us)
#endif
#ifndef DS18B20_DELAY_MS
	#define DS18B20_DELAY_MS(ms)	delay_ms(ms)
#endif

typedef struct {
	gpio_port_t port;	// 端口
    gpio_pin_t pin;		// 引脚
} ds18b20_config_t;

typedef struct ds18b20_dev {
    ds18b20_config_t config;
	float temperature;
	bool init_flag;										// 初始化标志
	int8_t (*get_temperature)(struct ds18b20_dev *dev);	// 获取温度
	int8_t (*deinit)(struct ds18b20_dev *dev);			// 去初始化
} ds18b20_dev_t;

int8_t ds18b20_init(ds18b20_dev_t *dev);

#endif
