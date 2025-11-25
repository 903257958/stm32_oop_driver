#ifndef DRV_ST7789V_H
#define DRV_ST7789V_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_ST7789V_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef SPI_TypeDef*	spi_periph_t;
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;
typedef TIM_TypeDef*	timer_periph_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
#define DRV_ST7789V_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef SPI_TypeDef*		spi_periph_t;
typedef GPIO_TypeDef*		gpio_port_t;
typedef uint32_t			gpio_pin_t;
typedef TIM_TypeDef*		timer_periph_t;
typedef uint32_t	        dma_channel_t;
typedef DMA_Stream_TypeDef* dma_stream_t;
	
#else
#error drv_st7789v.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
#define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW  0
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
} st7789v_color_t;

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
} st7789v_font_size_t;

/* SPI 操作接口结构体 */
typedef struct {
	int (*start)(gpio_port_t cs_port, gpio_pin_t cs_pin);
	int (*swap_byte)(uint8_t send, uint8_t *recv);
	int (*stop)(gpio_port_t cs_port, gpio_pin_t cs_pin);
} st7789v_spi_ops_t;

/* PWM 操作接口结构体 */
typedef struct {
	int (*set_psc)(uint16_t psc);
	int (*set_arr)(uint16_t arr);
	int (*set_compare)(uint16_t compare);
} st7789v_pwm_ops_t;

/* 配置结构体 */
typedef struct {
	const st7789v_spi_ops_t *spi_ops;
	const st7789v_pwm_ops_t *pwm_ops;
	void (*delay_ms)(uint32_t ms);
	spi_periph_t spi_periph;
	gpio_port_t  cs_port;
	gpio_pin_t   cs_pin;
	gpio_port_t  res_port;
	gpio_pin_t   res_pin;
	gpio_port_t  dc_port;
	gpio_pin_t   dc_pin;
	uint16_t     x_max;
    uint16_t     y_max;
    bool         is_vertical;	// true: 竖屏；false: 横屏
	bool         is_forward;	// true: 正向；false: 反向
} st7789v_cfg_t;

typedef struct st7789v_dev st7789v_dev_t;

/* 操作接口结构体 */
typedef struct {
    int (*set_backlight)(st7789v_dev_t *dev, uint16_t val);
	int (*clear)(st7789v_dev_t *dev, st7789v_color_t color);
	int (*fill)(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, st7789v_color_t color);
	int (*flush_area)(st7789v_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color);
	int (*flush_area_dma)(st7789v_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t size);
	int (*show_char)(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint8_t chr, st7789v_color_t fc, 
					 st7789v_color_t bc, st7789v_font_size_t size, bool overlay);
	int (*show_str)(st7789v_dev_t *dev, uint16_t x, uint16_t y, char *str, st7789v_color_t fc, st7789v_color_t bc, 
					st7789v_font_size_t size, bool overlay);
	int (*show_num)(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, st7789v_color_t fc, 
					st7789v_color_t bc, st7789v_font_size_t size, bool overlay);
	int (*show_hex_num)(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, st7789v_color_t fc, 
						st7789v_color_t bc, st7789v_font_size_t size, bool overlay);
	int (*show_float_num)(st7789v_dev_t *dev, uint16_t x, uint16_t y, float num, uint8_t int_len, uint8_t fra_len, 
						  st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay);
	int (*show_chinese)(st7789v_dev_t *dev, uint16_t x, uint16_t y, char *chinese, st7789v_color_t fc, 
						st7789v_color_t bc, st7789v_font_size_t size, bool overlay);
	int (*show_image)(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);
	int (*draw_point)(st7789v_dev_t *dev, uint16_t x, uint16_t y, st7789v_color_t color);
	int (*draw_line)(st7789v_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, st7789v_color_t color);
	int (*draw_rectangle)(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, 
						  st7789v_color_t color);
	int (*draw_circle)(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint8_t radius, st7789v_color_t color);
	int (*deinit)(st7789v_dev_t *dev);
} st7789v_ops_t;

/* 设备结构体 */
struct st7789v_dev {
	st7789v_cfg_t cfg;
	const st7789v_ops_t *ops;
};

/**
 * @brief   初始化 ST7789V 驱动
 * @param[out] dev 			 st7789v_dev_t 结构体指针
 * @param[in]  cfg 			 st7789v_cfg_t 结构体指针
 * @param[in]  disp_buf_addr 显示缓冲区地址，不使用 LVGL 等图形库时传入 NULL
 * @return	0 表示成功，其他值表示失败
 */
int drv_st7789v_init(st7789v_dev_t *dev, const st7789v_cfg_t *cfg, uint32_t disp_buf_addr);

#endif
