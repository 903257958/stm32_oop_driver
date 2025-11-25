#ifndef DRV_SPI_H
#define DRV_SPI_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_SPI_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef SPI_TypeDef*	spi_periph_t;
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
#define DRV_SPI_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef SPI_TypeDef*	spi_periph_t;
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
#define DRV_SPI_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t	spi_periph_t; 
typedef uint32_t	gpio_port_t;
typedef uint32_t	gpio_pin_t;

#else
#error drv_spi.h: No processor defined!
#endif

/* SPI 模式 */
typedef enum {
    SPI_MODE_0,
    SPI_MODE_1,
    SPI_MODE_2,
    SPI_MODE_3
} spi_mode_t;

/* 配置结构体 */
typedef struct {
	spi_periph_t spi_periph;
	gpio_port_t  sck_port;
	gpio_pin_t   sck_pin;
	gpio_port_t  miso_port;
	gpio_pin_t   miso_pin;
	gpio_port_t  mosi_port;
	gpio_pin_t   mosi_pin;
	uint16_t 	 prescaler;
	spi_mode_t   mode;
} spi_cfg_t;

typedef struct spi_dev spi_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*start)(spi_dev_t *dev, gpio_port_t cs_port, gpio_pin_t cs_pin);
	int (*stop)(spi_dev_t *dev, gpio_port_t cs_port, gpio_pin_t cs_pin);
	int (*swap_byte)(spi_dev_t *dev, uint8_t send, uint8_t *recv);
	int (*deinit)(spi_dev_t *dev);
} spi_ops_t;

/* 设备结构体 */
struct spi_dev {
	spi_cfg_t cfg;
	const spi_ops_t *ops;
};

/**
 * @brief   初始化 SPI 驱动
 * @param[out] dev spi_dev_t 结构体指针
 * @param[in]  cfg spi_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_spi_init(spi_dev_t *dev, const spi_cfg_t *cfg);

#endif
