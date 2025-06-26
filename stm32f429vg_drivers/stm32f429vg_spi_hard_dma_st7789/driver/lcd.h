#ifndef LCD_ST7789_DRV_H
#define LCD_ST7789_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
    typedef GPIO_TypeDef*	gpio_port_t;
    typedef uint32_t		gpio_pin_t;
	typedef TIM_TypeDef*	timer_periph_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	gpio_port_t;
    typedef uint32_t		gpio_pin_t;
    typedef TIM_TypeDef*	timer_periph_t;
	
#else
    #error lcd.h: No processor defined!
#endif

#endif

#include "lcd_data.h"
#include "spi_hard.h"
#include "delay.h"
#include "pwm.h"

#ifndef LCD_DELAY_MS
	#define LCD_DELAY_MS(ms)	delay_ms(ms)
#endif

/* LCD屏幕方向选择 */
#define LCD_H				280		// 屏幕高度
#define LCD_W				240		// 屏幕宽度
#define VERTICAL_FORWARD	0		// 竖屏正向
#define VERTICAL_REVERSE	1		// 竖屏反向
#define HORIZONTAL_FORWARD	2		// 横屏正向
#define HORIZONTAL_REVERSE	3		// 横屏反向

/* LCD画笔颜色 */
typedef enum {
	WHITE		=	0xFFFF,
	BLACK		=	0x0000,
	BLUE		=	0x001F,
	BRED		=	0XF81F,
	GRED		=	0XFFE0,
	GBLUE		=	0X07FF,
	RED			=	0xF800,
	MAGENTA		=	0xF81F,
	GREEN		=	0x07E0,
	CYAN		=	0x7FFF,
	YELLOW		=	0xFFE0,
	BROWN		=	0XBC40,		// 棕色
	BRRED		=	0XFC07,		// 棕红色
	GRAY		=	0X8430,		// 灰色
	DARKBLUE	=	0X01CF,		// 深蓝色
	LIGHTBLUE	=	0X7D7C,		// 浅蓝色
	GRAYBLUE	=	0X5458,		// 灰蓝色
	LIGHTGREEN	=	0X841F,		// 浅绿色
	LGRAY		=	0XC618,		// 浅灰色(PANNEL),窗体背景色
	LGRAYBLUE	=	0XA651,		// 浅灰蓝色(中间层颜色)
	LBBLUE		=	0X2B12		// 浅棕蓝色(选择条目的反色)
} lcd_color_t;

/* LCD显示字体大小 */
typedef enum {
	LCD_16X32 = 32,
	LCD_12X24 = 24,
	LCD_8X16 = 16,
	LCD_6X12 = 12,
	LCD_32X32 = 32,
	LCD_24X24 = 24,
	LCD_16X16 = 16,
	LCD_12X12 = 12
} lcd_font_size_t;

/* isFilled参数数值 */
typedef enum {
	LCD_UNFILLED = 0,
	LCD_FILLED = 1,
} lcd_is_filled_t;

typedef struct {
	spi_periph_t spix;		// SPI外设
	gpio_port_t sck_port;	// SCK端口
	gpio_pin_t sck_pin;		// SCK引脚
	gpio_port_t mosi_port;	// MOSI端口
	gpio_pin_t mosi_pin;	// MOSI引脚
	gpio_port_t res_port;	// RES端口
	gpio_pin_t res_pin;		// RES引脚
	gpio_port_t dc_port;	// DC端口
	gpio_pin_t dc_pin;		// DC引脚
	gpio_port_t cs_port;	// CS端口
	gpio_pin_t cs_pin;		// CS引脚
	gpio_port_t bl_port;	// BL端口
	gpio_pin_t bl_pin;		// BL引脚
	timer_periph_t timx;	// 背光PWM定时器
	uint8_t oc_channel;		// 背光PWM输出比较通道
	uint8_t dir;			// 显示方向
} lcd_config_t;

typedef struct lcd_dev {
	lcd_config_t config;
	bool init_flag;					// 初始化标志
	void *priv_data;				// 私有数据指针
	uint16_t width;
	uint16_t height;
	void (*backlight_ctrl)(struct lcd_dev *dev, uint16_t val);
	void (*clear)(struct lcd_dev *dev, uint16_t color);
	void (*fill)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
	void (*color_fill)(struct lcd_dev *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color);
	void (*color_fill_dma)(struct lcd_dev *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t size);
	void (*show_char)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_string)(struct lcd_dev *dev, uint16_t x, uint16_t y, char *str, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_num)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_hex_num)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_float_num)(struct lcd_dev *dev, uint16_t x, uint16_t y, float num, uint8_t int_len, uint8_t fra_len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_chinese)(struct lcd_dev *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_image)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);
	void (*draw_point)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint16_t color);
	void (*draw_line)(struct lcd_dev *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
	void (*draw_rectangle)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
	void (*draw_circle)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint8_t radius, uint16_t color);
	int8_t (*deinit)(struct lcd_dev *dev);
} lcd_dev_t;

int8_t lcd_init(lcd_dev_t *dev);
void lcd_dma_init(lcd_dev_t *dev, uint32_t mem_base_addr);

#endif
