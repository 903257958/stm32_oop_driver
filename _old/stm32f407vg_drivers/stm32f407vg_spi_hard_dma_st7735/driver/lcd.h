#ifndef LCD_DRV_H
#define LCD_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
	typedef SPI_TypeDef*	spi_periph_t;
    typedef GPIO_TypeDef*	gpio_port_t;
    typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef SPI_TypeDef*	spi_periph_t;
	typedef GPIO_TypeDef*	gpio_port_t;
    typedef uint32_t		gpio_pin_t;
	
#else
    #error lcd.h: No processor defined!
#endif

#endif

#include "spi_hard.h"
#include "delay.h"

#ifndef LCD_DELAY_MS
	#define LCD_DELAY_MS(ms)	delay_ms(ms)
#endif

/* LCD屏幕方向选择：0为竖屏正向，1为竖屏反向，2为横屏正向，3为横屏反向 */
#define LCD_DIRECTION	3

#if (LCD_DIRECTION == 0) || (LCD_DIRECTION == 1)
#define LCD_W 128
#define LCD_H 160

#else
#define LCD_W 160
#define LCD_H 128

#endif

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

/* is_filled参数数值 */
typedef enum {
	LCD_UNFILLED = 0,
	LCD_FILLED = 1,
} lcd_is_filled_t;

/* 是否使用DMA */
typedef enum {
	LCD_NONUSE_DMA = 0,
	LCD_USE_DMA = 1
} lcd_use_dma_t;

typedef struct {
	spi_periph_t spix;			// SPI外设
	gpio_port_t sck_port;		// SCK端口
	gpio_pin_t sck_pin;			// SCK引脚
	gpio_port_t mosi_port;		// MOSI端口
	gpio_pin_t mosi_pin;		// MOSI引脚
	gpio_port_t res_port;		// RES端口
	gpio_pin_t res_pin;			// RES引脚
	gpio_port_t dc_port;		// DC端口
	gpio_pin_t dc_pin;			// DC引脚
	gpio_port_t cs_port;		// CS端口
	gpio_pin_t cs_pin;			// CS引脚
	gpio_port_t bl_port;		// BL端口
	gpio_pin_t bl_pin;			// BL引脚
	uint16_t prescaler;			// 预分频系数
	spi_mode_t mode;			// SPI模式
	lcd_use_dma_t use_dma;		// 是否使用DMA
} lcd_config_t;

typedef struct lcd_dev {
	lcd_config_t config;
	bool init_flag;													// 初始化标志
	void *priv_data;												// 私有数据指针
	void (*update)(struct lcd_dev *dev);							// DMA传输更新显存函数
	void (*fill)(struct lcd_dev *dev, uint16_t color);				// LCD填充颜色
	void (*fill_area)(struct lcd_dev *dev, uint16_t x1, uint16_t y1, uint16_t width, uint16_t height, uint16_t color);						//LCD指定区域填充颜色
	void (*show_char)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t size);					//LCD显示字符
	void (*show_string)(struct lcd_dev *dev, uint16_t x, uint16_t y, char *string, uint16_t fc, uint16_t bc, uint8_t size);					//LCD显示字符串
	void (*show_num)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc , uint8_t size);		//LCD显示整型数字
	void (*show_float_num)(struct lcd_dev *dev, uint16_t x, uint16_t y, float num, uint8_t int_len, uint8_t fra_len, uint16_t fc, uint16_t bc, uint8_t size);	//LCD显示两位浮点数
	void (*show_chinese)(struct lcd_dev *dev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t size);				//LCD显示汉字串
	void (*show_image)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);					//LCD显示图片
	void (*draw_point)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint16_t color);														//LCD画点
	void (*draw_line)(struct lcd_dev *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);								//LCD画线
	void (*draw_rectangle)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t is_filled);//LCD画矩形
	void (*draw_circle)(struct lcd_dev *dev, uint16_t x, uint16_t y, uint8_t radius, uint16_t color);										//LCD画圆
	int (*deinit)(struct lcd_dev *dev);																										//去初始化
} lcd_dev_t;

int lcd_init(lcd_dev_t *dev);

#endif
