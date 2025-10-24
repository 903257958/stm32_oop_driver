#ifndef OLED_SSD1306_I2C_SOFT_DRV_H
#define OLED_SSD1306_I2C_SOFT_DRV_H

#include <stdint.h>
#include <stdbool.h>
#include "i2c_soft.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
    typedef GPIO_TypeDef*	gpio_port_t;
    typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	gpio_port_t;
    typedef uint32_t		gpio_pin_t;
	
#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
    #include "gd32f10x.h"
    typedef uint32_t    gpio_port_t;
    typedef uint32_t	gpio_pin_t;
    
#else
    #error oled.h: No processor defined!
#endif

#endif

/* 屏幕方向选择，0为正向，1为反向 */
#define OLED_DIRECTION		0

/* 
 * size参数取值
 * 此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度 
 */
#define OLED_8X16			8
#define OLED_6X8			6

/* is_filled参数数值 */
#define OLED_UNFILLED		0
#define OLED_FILLED			1

typedef struct {
	gpio_port_t scl_port;
	gpio_pin_t scl_pin;
	gpio_port_t sda_port;
	gpio_pin_t sda_pin;
} oled_config_t;

typedef struct oled_dev {
	oled_config_t config;
	bool init_flag;
	void *priv_data;
	int (*update)(struct oled_dev *dev);
	int (*update_area)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*clear)(struct oled_dev *dev);
	int (*clear_area)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*reverse)(struct oled_dev *dev);
	int (*reverse_area)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*show_image)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *img);
	int (*show_char)(struct oled_dev *dev, uint8_t x, uint8_t y, char chr, uint8_t size);
	int (*show_string)(struct oled_dev *dev, uint8_t x, uint8_t y, char *str, uint8_t size);
	int (*show_num)(struct oled_dev *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
	int (*show_signed_num)(struct oled_dev *dev, uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size);
	int (*show_hex_num)(struct oled_dev *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
	int (*show_bin_num)(struct oled_dev *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
	int (*show_float_num)(struct oled_dev *dev, uint8_t x, uint8_t y, double num, uint8_t int_len, uint8_t fra_len, uint8_t size);
	int (*show_chinese)(struct oled_dev *dev, uint8_t x, uint8_t y, char *chinese);
	int (*printf)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t size, char *format, ...);
	int (*draw_point)(struct oled_dev *dev, uint8_t x, uint8_t y);
	int (*get_point)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t *point);
	int (*draw_line)(struct oled_dev *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
	int (*draw_rectangle)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t is_filled);
	int (*draw_triangle)(struct oled_dev *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t is_filled);
	int (*draw_circle)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t radius, uint8_t is_filled);
	int (*draw_ellipse)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t a, uint8_t b, uint8_t is_filled);
	int (*draw_arc)(struct oled_dev *dev, uint8_t x, uint8_t y, uint8_t radius, int16_t start_angle, int16_t end_angle, uint8_t is_filled);
	int (*deinit)(struct oled_dev *dev);
} oled_dev_t;

int oled_drv_init(oled_dev_t *dev);

#endif
