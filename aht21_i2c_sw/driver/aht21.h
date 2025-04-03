#ifndef __AHT21_H
#define __AHT21_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "i2c.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	AHT21GPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	AHT21GPIOPort_t;
	
#else
	#error aht21.h: No processor defined!
#endif

/* AHT21的I2C从机地址 */
#define AHT21_ADDRESS		0x38

/* AHT21寄存器 */
#define AHT21_GET_STATUS	0x71
#define AHT21_INIT			0xBE
#define AHT21_MEASURE		0xAC

typedef struct {
    AHT21GPIOPort_t scl_port;		// SCL端口
	uint32_t scl_pin;				// SCL引脚
	AHT21GPIOPort_t sda_port;		// SDA端口
	uint32_t sda_pin;				// SDA引脚
}AHT21Config_t;

typedef struct AHT21Dev {
	AHT21Config_t config;
	bool init_flag;		// 初始化标志
    void *priv_data;	// 私有数据指针
	float temp;
	float humi;
	int (*get_data)(struct AHT21Dev *dev);
	int (*deinit)(struct AHT21Dev *dev); // 去初始化
}AHT21Dev_t;

int aht21_init(AHT21Dev_t *dev);

#endif
