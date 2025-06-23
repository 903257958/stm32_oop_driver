#ifndef SPI_SOFT_DRV_H
#define SPI_SOFT_DRV_H

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
	#error spi_soft.h: No processor defined!
#endif

#endif

typedef enum {
    SPI_MODE_0,
    SPI_MODE_1,
    SPI_MODE_2,
    SPI_MODE_3
} spi_mode_t;

typedef struct {
	gpio_port_t sck_port;	// SCK端口
	gpio_pin_t sck_pin;	    // SCK引脚
	gpio_port_t mosi_port;	// MOSI端口
	gpio_pin_t mosi_pin;	// MOSI引脚
	gpio_port_t miso_port;	// MISO端口
	gpio_pin_t miso_pin;	// MISO引脚
	gpio_port_t cs_port;	// CS端口
	gpio_pin_t cs_pin;		// CS引脚
	spi_mode_t mode;        // SPI模式
} spi_soft_config_t;

typedef struct spi_soft_dev {
	spi_soft_config_t config;
	bool init_flag;														// 初始化标志
	void (*cs_write)(struct spi_soft_dev *dev, uint8_t bit_val);		// 写CS引脚电平
	void (*start)(struct spi_soft_dev *dev);							// 起始
	void (*stop)(struct spi_soft_dev *dev);								// 停止
	uint8_t (*swap_byte)(struct spi_soft_dev *dev, uint8_t send_byte);	// 交换一个字节
	int8_t (*deinit)(struct spi_soft_dev *dev);							// 去初始化
} spi_soft_dev_t;

int8_t spi_soft_init(spi_soft_dev_t *dev);

#endif
