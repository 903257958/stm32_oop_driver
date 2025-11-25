#ifndef I2C_SOFT_DRV_H
#define I2C_SOFT_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
    
#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
    #include "gd32f10x.h"
    typedef uint32_t	gpio_port_t;
	typedef uint32_t	gpio_pin_t;
	
#else
	#error i2c.h: No processor defined!
#endif

#endif

#include "delay.h"

#ifndef I2C_SOFT_DELAY_US
	#define I2C_SOFT_DELAY_US(us)	delay_us(us)
#endif

typedef struct {
	gpio_port_t scl_port;	// SCL端口
	gpio_pin_t scl_pin;	    // SCL引脚
	gpio_port_t sda_port;	// SDA端口
	gpio_pin_t sda_pin;	    // SDA引脚
} i2c_soft_config_t;

typedef struct i2c_soft_dev {
	i2c_soft_config_t config;
	bool init_flag;													// 初始化标志
	int8_t (*start)(struct i2c_soft_dev *dev);						// 起始
	int8_t (*stop)(struct i2c_soft_dev *dev);						// 停止
	int8_t (*send_byte)(struct i2c_soft_dev *dev, uint8_t byte);	// 发送一个字节
	uint8_t (*recv_byte)(struct i2c_soft_dev *dev);					// 接收一个字节
	int8_t (*send_ack)(struct i2c_soft_dev *dev, uint8_t ack);		// 发送应答位
	uint8_t (*recv_ack)(struct i2c_soft_dev *dev);					// 接收应答位
	int8_t (*read_reg)(struct i2c_soft_dev *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);					// 读寄存器
	int8_t (*read_regs)(struct i2c_soft_dev *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[]);	// 读多个寄存器
	int8_t (*write_reg)(struct i2c_soft_dev *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);					// 写寄存器
	int8_t (*write_regs)(struct i2c_soft_dev *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[]);	// 写多个寄存器
	int8_t (*deinit)(struct i2c_soft_dev *dev);						// 去初始化
} i2c_soft_dev_t;

int8_t i2c_soft_init(i2c_soft_dev_t *dev);

#endif
