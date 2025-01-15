#ifndef __I2C_H
#define __I2C_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef GPIO_TypeDef*	I2C_GPIO_Port;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	I2C_GPIO_Port;
	
#else
	#error i2c.h: No processor defined!
#endif

#ifndef FREERTOS
	#define FREERTOS	0
#endif

#if FREERTOS
	#include "timer.h"
	#include "FreeRTOS.h"
	#include "task.h"

	extern TimerDev_t timerDelay;
#endif

#ifndef i2c_log
	#define i2c_log(x) 
#endif

typedef struct {
	I2C_GPIO_Port SCLPort;			// SCL端口
	uint32_t SCLPin;				// SCL引脚
	I2C_GPIO_Port SDAPort;			// SDA端口
	uint32_t SDAPin;				// SDA引脚
}I2CInfo_t;

typedef struct I2CDev {
	I2CInfo_t info;
	bool initFlag;												// 初始化标志
	int (*start)(struct I2CDev *pDev);							// 软件I2C起始
	int (*stop)(struct I2CDev *pDev);							// 软件I2C停止
	int (*send_byte)(struct I2CDev *pDev, uint8_t byte);		// 软件I2C发送一个字节
	uint8_t (*recv_byte)(struct I2CDev *pDev);					// 软件I2C接收一个字节
	int (*send_ack)(struct I2CDev *pDev, uint8_t ack);			// 软件I2C发送应答位
	uint8_t (*recv_ack)(struct I2CDev *pDev);					// 软件I2C接收应答位
	uint8_t (*check_dev)(struct I2CDev *pDev, uint8_t addr);	// 软件I2C检查已连接的设备
	int (*deinit)(struct I2CDev *pDev);							// 去初始化
}I2CDev_t;

int i2c_init(I2CDev_t *pDev);
#endif
