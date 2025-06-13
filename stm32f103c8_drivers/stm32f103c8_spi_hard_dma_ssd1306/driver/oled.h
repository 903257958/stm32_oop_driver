#ifndef OLED_SSD1306_SPI_DRV_H
#define OLED_SSD1306_SPI_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
	typedef SPI_TypeDef*	spi_periph_t;
    typedef GPIO_TypeDef*	oled_gpio_port_t;
    typedef uint32_t		oled_gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef SPI_TypeDef*	spi_periph_t;
	typedef GPIO_TypeDef*	oled_gpio_port_t;
    typedef uint32_t		oled_gpio_pin_t;
	
#else
    #error oled.h: No processor defined!
#endif

#include "spi_hard.h"

/* 屏幕方向选择，0为正向，1为反向 */
#define OLED_DIRECTION		0

/* fontSize参数取值 */
/* 此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度 */
#define OLED_8X16			8
#define OLED_6X8			6

/* isFilled参数数值 */
#define OLED_UNFILLED		0
#define OLED_FILLED			1

typedef struct {
	spi_periph_t spix;				// SPI外设
	oled_gpio_port_t sck_port;		// SCK端口
	oled_gpio_pin_t sck_pin;		// SCK引脚
	oled_gpio_port_t mosi_port;		// MOSI端口
	oled_gpio_pin_t mosi_pin;		// MOSI引脚
	oled_gpio_port_t res_port;		// RES端口
	oled_gpio_pin_t res_pin;		// RES引脚
	oled_gpio_port_t dc_port;		// DC端口
	oled_gpio_pin_t dc_pin;			// DC引脚
	oled_gpio_port_t cs_port;		// CS端口
	oled_gpio_pin_t cs_pin;			// CS引脚
	uint16_t prescaler;				// 预分频系数
	spi_mode_t mode;				// SPI模式
}oled_config_t;

typedef struct oled_dev {
	oled_config_t config;
	bool init_flag;								// 初始化标志
	void *priv_data;							// 私有数据指针
	int (*update)(struct oled_dev *dev);
	int (*update_area)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*clear)(struct oled_dev *dev);
	int (*clear_area)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*reverse)(struct oled_dev *dev);
	int (*reverse_area)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*show_image)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *image);
	int (*show_char)(struct oled_dev *dev, uint8_t x, uint8_t y, char chr, uint8_t size);
	int (*show_string)(struct oled_dev *dev, uint8_t x, uint8_t y, char *string, uint8_t size);
	int (*show_num)(struct oled_dev *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
	int (*show_signed_num)(struct oled_dev *dev, uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size);
	int (*show_hex_num)(struct oled_dev *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
	int (*show_bin_num)(struct oled_dev *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
	int (*show_float_num)(struct oled_dev *dev, uint8_t x, uint8_t y, double num, uint8_t int_len, uint8_t fra_len, uint8_t size);
	int (*show_chinese)(struct oled_dev *dev, uint8_t x, uint8_t y, char *chinese);
	int (*printf)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t size, char *format, ...);
	int (*draw_point)(struct oled_dev *dev, uint8_t x, uint8_t y);
	uint8_t (*get_point)(struct oled_dev *dev, uint8_t x, uint8_t y);
	int (*draw_line)(struct oled_dev *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
	int (*draw_rectangle)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t is_filled);
	int (*draw_triangle)(struct oled_dev *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t is_filled);
	int (*draw_circle)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t radius, uint8_t is_filled);
	int (*draw_ellipse)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t a, uint8_t b, uint8_t is_filled);
	int (*draw_arc)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t radius, int16_t start_angle, int16_t end_angle, uint8_t is_filled);
	int (*deinit)(struct oled_dev *dev);
}oled_dev_t;

int oled_init(oled_dev_t *dev);

#endif
