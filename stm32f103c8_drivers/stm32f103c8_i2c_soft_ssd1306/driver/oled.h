#ifndef OLED_SSD1306_I2C_DRV_H
#define OLED_SSD1306_I2C_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
    typedef GPIO_TypeDef*	gpio_port_t;
    typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	gpio_port_t;
    typedef uint32_t		gpio_pin_t;
	
#else
    #error oled.h: No processor defined!
#endif

#endif

#include "i2c_soft.h"

/* 屏幕方向选择，0为正向，1为反向 */
#define OLED_DIRECTION		0

/* size参数取值 */
/* 此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度 */
#define OLED_8X16			8
#define OLED_6X8			6

/* is_filled参数数值 */
#define OLED_UNFILLED		0
#define OLED_FILLED			1

typedef struct {
	gpio_port_t scl_port;	// SCL端口
	gpio_pin_t scl_pin;		// SCL引脚
	gpio_port_t sda_port;	// SDA端口
	gpio_pin_t sda_pin;		// SDA引脚
} oled_config_t;

typedef struct oled_dev {
	oled_config_t config;
	bool init_flag;								// 初始化标志
	void *priv_data;							// 私有数据指针
	int8_t (*update)(struct oled_dev *dev);
	int8_t (*update_area)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int8_t (*clear)(struct oled_dev *dev);
	int8_t (*clear_area)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int8_t (*reverse)(struct oled_dev *dev);
	int8_t (*reverse_area)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int8_t (*show_image)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *image);
	int8_t (*show_char)(struct oled_dev *dev, uint8_t x, uint8_t y, char chr, uint8_t size);
	int8_t (*show_string)(struct oled_dev *dev, uint8_t x, uint8_t y, char *string, uint8_t size);
	int8_t (*show_num)(struct oled_dev *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
	int8_t (*show_signed_num)(struct oled_dev *dev, uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size);
	int8_t (*show_hex_num)(struct oled_dev *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
	int8_t (*show_bin_num)(struct oled_dev *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
	int8_t (*show_float_num)(struct oled_dev *dev, uint8_t x, uint8_t y, double num, uint8_t int_len, uint8_t fra_len, uint8_t size);
	int8_t (*show_chinese)(struct oled_dev *dev, uint8_t x, uint8_t y, char *chinese);
	int8_t (*printf)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t size, char *format, ...);
	int8_t (*draw_point)(struct oled_dev *dev, uint8_t x, uint8_t y);
	uint8_t (*get_point)(struct oled_dev *dev, uint8_t x, uint8_t y);
	int8_t (*draw_line)(struct oled_dev *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
	int8_t (*draw_rectangle)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t is_filled);
	int8_t (*draw_triangle)(struct oled_dev *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t is_filled);
	int8_t (*draw_circle)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t radius, uint8_t is_filled);
	int8_t (*draw_ellipse)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t a, uint8_t b, uint8_t is_filled);
	int8_t (*draw_arc)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t radius, int16_t start_angle, int16_t end_angle, uint8_t is_filled);
	int8_t (*deinit)(struct oled_dev *dev);
} oled_dev_t;

int8_t oled_init(oled_dev_t *dev);

#endif
