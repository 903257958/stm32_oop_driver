#ifndef WS2812B_DRV_H
#define WS2812B_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef*			timer_periph_t;
	typedef GPIO_TypeDef*			gpio_port_t;
	typedef uint32_t				gpio_pin_t;
	typedef DMA_Channel_TypeDef*	dma_channel_t;

#else
	#error ws2812b.h: No processor defined!
#endif

#endif

#include "pwm.h"

/* WS2812B的最大设备数量 */
#define WS2812B_MAX_DEVICE_NUM	1

/* 每个WS2812B设备的最大LED数量 */
#define WS2812B_MAX_LED_NUM		60

/* WS2812B配置结构体 */
typedef struct {
	timer_periph_t timx;	// 定时器外设
	uint8_t oc_channel;		// 输出比较通道
	gpio_port_t port;		// 端口
	gpio_pin_t pin;			// 引脚
	uint16_t led_num;		// LED灯数量
} ws2812b_config_t;

/* WS2812B设备结构体 */
typedef struct ws2812b_dev {
	ws2812b_config_t config;
	bool init_flag;									// 初始化标志
	void *priv_data;								// 私有数据指针
	int8_t (*set_color)(struct ws2812b_dev *dev, uint32_t *color_rgb_buf);		// 设置颜色
	int8_t (*set_single_color)(struct ws2812b_dev *dev, uint32_t color_rgb);	// 设置单种颜色
	int8_t (*off)(struct ws2812b_dev *dev);			// 关闭
	int8_t (*deinit)(struct ws2812b_dev *dev);		// 去初始化
} ws2812b_dev_t;

/* 函数声明 */
int8_t ws2812b_init(ws2812b_dev_t *dev);

#endif
