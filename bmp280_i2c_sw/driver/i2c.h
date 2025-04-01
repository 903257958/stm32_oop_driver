#ifndef __I2C_H
#define __I2C_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	I2CGPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	I2CGPIOPort_t;
	
#else
	#error i2c.h: No processor defined!
#endif

#ifndef i2c_log
	#define i2c_log(x) 
#endif

typedef struct {
	I2CGPIOPort_t scl_port;			// SCL端口
	uint32_t scl_pin;				// SCL引脚
	I2CGPIOPort_t sda_port;			// SDA端口
	uint32_t sda_pin;				// SDA引脚
}I2CInfo_t;

typedef struct I2CDev {
	I2CInfo_t info;
	bool init_flag;											// 初始化标志
	int (*start)(struct I2CDev *dev);						// 软件I2C起始
	int (*stop)(struct I2CDev *dev);						// 软件I2C停止
	int (*send_byte)(struct I2CDev *dev, uint8_t byte);		// 软件I2C发送一个字节
	uint8_t (*recv_byte)(struct I2CDev *dev);				// 软件I2C接收一个字节
	int (*send_ack)(struct I2CDev *dev, uint8_t ack);		// 软件I2C发送应答位
	uint8_t (*recv_ack)(struct I2CDev *dev);				// 软件I2C接收应答位
	int (*read_reg)(struct I2CDev *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);					// 软件I2C读寄存器
	int (*read_regs)(struct I2CDev *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t num, uint8_t data[]);	// 软件I2C读多个寄存器
	int (*write_reg)(struct I2CDev *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);					// 软件I2C写寄存器
	int (*write_regs)(struct I2CDev *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t num, uint8_t data[]);	// 软件I2C写多个寄存器
	int (*deinit)(struct I2CDev *dev);						// 去初始化
}I2CDev_t;

int i2c_init(I2CDev_t *dev);

#endif
