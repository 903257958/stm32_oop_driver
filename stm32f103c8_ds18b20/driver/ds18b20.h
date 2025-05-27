#ifndef __DS18B20_H
#define __DS18B20_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
    typedef GPIO_TypeDef*	DS18B20GPIOPort_t;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	DS18B20GPIOPort_t;
	
#else
    #error ds18b20.h: No processor defined!
#endif

typedef struct {
	DS18B20GPIOPort_t port;							// 端口
    uint32_t pin;									// 引脚
}DS18B20Config_t;

typedef struct DS18B20Dev {
    DS18B20Config_t config;
	float temperature;
	bool init_flag;										// 初始化标志
	int8_t (*get_temperature)(struct DS18B20Dev *dev);	// 获取温度
	int8_t (*deinit)(struct DS18B20Dev *dev);			// 去初始化
}DS18B20Dev_t;

int8_t ds18b20_init(DS18B20Dev_t *dev);

#endif
