#ifndef __DHT11_H
#define __DHT11_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
    typedef GPIO_TypeDef*	DHT11GPIOPort_t;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	DHT11GPIOPort_t;
	
#else
    #error dht11.h: No processor defined!
#endif

typedef struct {
	DHT11GPIOPort_t port;	// 端口
    uint32_t pin;			// 引脚
}DHT11Config_t;

typedef struct {
	uint8_t temperature;    // 温度
    uint8_t humidity;       // 湿度
}DHT11Data_t;

typedef struct DHT11Dev {
    DHT11Config_t config;
    DHT11Data_t data;
	bool init_flag;                             // 初始化标志
	int8_t (*get_data)(struct DHT11Dev *dev);	// 获取数据
	int8_t (*deinit)(struct DHT11Dev *dev);     // 去初始化
}DHT11Dev_t;

int8_t dht11_init(DHT11Dev_t *dev);

#endif
