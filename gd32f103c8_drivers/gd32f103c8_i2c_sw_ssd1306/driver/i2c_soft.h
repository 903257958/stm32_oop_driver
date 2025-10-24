#ifndef I2C_SOFT_DRV_H
#define I2C_SOFT_DRV_H

#include <stdint.h>
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

#ifndef i2c_soft_delay_us
	#define i2c_soft_delay_us(us)	delay_us(us)
#endif

typedef struct {
	gpio_port_t scl_port;
	gpio_pin_t scl_pin;
	gpio_port_t sda_port;
	gpio_pin_t sda_pin;
} i2c_soft_config_t;

typedef struct i2c_soft_dev {
	i2c_soft_config_t config;
	bool init_flag;
	int (*start)(struct i2c_soft_dev *dev);
	int (*stop)(struct i2c_soft_dev *dev);
	int (*send_byte)(struct i2c_soft_dev *dev, uint8_t byte);
	int (*recv_byte)(struct i2c_soft_dev *dev, uint8_t *byte);
	int (*send_ack)(struct i2c_soft_dev *dev, uint8_t ack);
	int (*recv_ack)(struct i2c_soft_dev *dev, uint8_t *ack);
	int (*read_reg)(struct i2c_soft_dev *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
	int (*read_regs)(struct i2c_soft_dev *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[]);
	int (*write_reg)(struct i2c_soft_dev *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
	int (*write_regs)(struct i2c_soft_dev *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[]);
	int (*deinit)(struct i2c_soft_dev *dev);
} i2c_soft_dev_t;

int i2c_soft_drv_init(i2c_soft_dev_t *dev);

#endif
