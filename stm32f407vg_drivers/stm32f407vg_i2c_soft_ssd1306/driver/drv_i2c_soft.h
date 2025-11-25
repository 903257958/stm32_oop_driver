#ifndef DRV_I2C_SOFT_H
#define DRV_I2C_SOFT_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_I2C_SOFT_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_I2C_SOFT_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
#define DRV_I2C_SOFT_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t	gpio_port_t;
typedef uint32_t	gpio_pin_t;

#else
#error drv_i2c_soft.h: No processor defined!
#endif

/* 配置结构体 */
typedef struct {
	gpio_port_t scl_port;
	gpio_pin_t  scl_pin;
	gpio_port_t sda_port;
	gpio_pin_t  sda_pin;
	uint32_t    bit_delay_us; // 电平切换延时，防止高频 MCU 反转过快导致时序问题
	void (*delay_us)(uint32_t us);
} i2c_soft_cfg_t;

typedef struct i2c_soft_dev i2c_soft_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*start)(i2c_soft_dev_t *dev);
	int (*stop)(i2c_soft_dev_t *dev);
	int (*send_byte)(i2c_soft_dev_t *dev, uint8_t byte);
	int (*recv_byte)(i2c_soft_dev_t *dev, uint8_t *byte);
	int (*send_ack)(i2c_soft_dev_t *dev, uint8_t ack);
	int (*recv_ack)(i2c_soft_dev_t *dev, uint8_t *ack);
	int (*read_reg)(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
	int (*read_regs)(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[]);
	int (*write_reg)(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
	int (*write_regs)(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[]);
	int (*deinit)(i2c_soft_dev_t *dev);
} i2c_soft_ops_t;

/* 设备结构体 */
struct i2c_soft_dev {
	i2c_soft_cfg_t cfg;
	const i2c_soft_ops_t *ops;
};

/**
 * @brief   初始化软件 I2C 驱动
 * @param[out] dev i2c_soft_dev_t 结构体指针
 * @param[in]  cfg i2c_soft_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_i2c_soft_init(i2c_soft_dev_t *dev, const i2c_soft_cfg_t *cfg);

#endif
