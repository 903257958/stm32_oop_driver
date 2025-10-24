#ifndef SR04_DRV_H
#define SR04_DRV_H

#include <stdint.h>
#include <stdbool.h>

#ifdef USE_STDPERIPH_DRIVER
	
#if defined (GD32F10X_MD)
    #include "gd32f10x.h"
    typedef uint32_t	timer_periph_t;
    typedef uint32_t	gpio_port_t;
    typedef uint32_t	gpio_pin_t;
#else
	#error sr04.h: No processor defined!
#endif

#endif

#include "delay.h"

#ifndef sr04_delay_us
	#define sr04_delay_us(us)	delay_us(us)
#endif

#ifndef sr04_delay_ms
	#define sr04_delay_ms(ms)	delay_ms(ms)
#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

/* 最大SR04设备数 */
#define MAX_SR04_NUM	1

/* 为避免中断处理函数重复定义，初始化时需根据配置启用对应的中断处理函数 */
#define SR04_TIM1_IRQ_HANDLER_ENABLE	1
#define SR04_TIM2_IRQ_HANDLER_ENABLE	0
#define SR04_TIM3_IRQ_HANDLER_ENABLE	0

/* SR04配置结构体 */
typedef struct {
	timer_periph_t timx;	// 定时器外设，每个SR04设备需要使用不同的定时器
	uint8_t ic_channel;		// 输入捕获通道: 0~3
	gpio_port_t trig_port;
	gpio_pin_t trig_pin;
	gpio_port_t echo_port;
	gpio_pin_t echo_pin;
} sr04_config_t;

/* SR04设备结构体 */
typedef struct sr04_dev {
	sr04_config_t config;
	bool init_flag;
	void *priv_data;
	float distance_cm;
	int (*get_distance)(struct sr04_dev *dev);
	int (*deinit)(struct sr04_dev *dev);
} sr04_dev_t;

/* 函数声明 */
int sr04_drv_init(sr04_dev_t *dev);

#endif
