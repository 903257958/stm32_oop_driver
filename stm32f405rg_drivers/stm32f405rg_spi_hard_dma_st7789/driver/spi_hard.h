#ifndef SPI_HARD_DRV_H
#define SPI_HARD_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef SPI_TypeDef*	spi_periph_t;
	typedef GPIO_TypeDef*	spi_hard_gpio_port_t;
	typedef uint32_t		spi_hard_gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef SPI_TypeDef*	spi_periph_t;
	typedef GPIO_TypeDef*	spi_hard_gpio_port_t;
	typedef uint32_t		spi_hard_gpio_pin_t;

#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
    #include "gd32f10x.h"
    typedef uint32_t	spi_periph_t; 
    typedef uint32_t	spi_hard_gpio_port_t;
	typedef uint32_t	spi_hard_gpio_pin_t;
	
#else
	#error spi.h: No processor defined!
#endif

typedef enum {
    SPI_MODE_0,
    SPI_MODE_1,
    SPI_MODE_2,
    SPI_MODE_3
} spi_mode_t;

typedef struct {
	spi_periph_t spix;				// SPI外设
	spi_hard_gpio_port_t sck_port;	// SCK端口
	spi_hard_gpio_pin_t sck_pin;	// SCK引脚
	spi_hard_gpio_port_t mosi_port;	// MOSI端口
	spi_hard_gpio_pin_t mosi_pin;	// MOSI引脚
	spi_hard_gpio_port_t miso_port;	// MISO端口
	spi_hard_gpio_pin_t miso_pin;	// MISO引脚
	spi_hard_gpio_port_t cs_port;	// CS端口
	spi_hard_gpio_pin_t cs_pin;		// CS引脚
	uint16_t prescaler;				// 预分频系数
	spi_mode_t mode;				// SPI模式
} spi_hard_config_t;

typedef struct spi_hard_dev {
	spi_hard_config_t config;
	bool init_flag;														// 初始化标志
	void (*cs_write)(struct spi_hard_dev *dev, uint8_t bit_val);		// 写CS引脚电平
	void (*start)(struct spi_hard_dev *dev);							// 起始
	void (*stop)(struct spi_hard_dev *dev);								// 停止
	uint8_t (*swap_byte)(struct spi_hard_dev *dev, uint8_t send_byte);	// 交换一个字节
	int8_t (*deinit)(struct spi_hard_dev *dev);							// 去初始化
} spi_hard_dev_t;

int8_t spi_hard_init(spi_hard_dev_t *dev);

#endif
