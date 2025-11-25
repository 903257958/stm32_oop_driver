#ifndef DRV_SR04_H
#define DRV_SR04_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_SR04_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef TIM_TypeDef*	timer_periph_t;
typedef IRQn_Type		iqrn_type_t;
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)
#define DRV_SR04_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef TIM_TypeDef*	timer_periph_t;
typedef IRQn_Type		iqrn_type_t;
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined (GD32F10X_MD)
#define DRV_SR04_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t	timer_periph_t;
typedef IRQn_Type	iqrn_type_t;
typedef uint32_t	gpio_port_t;
typedef uint32_t	gpio_pin_t;

#else
#error drv_sr04.h: No processor defined!
#endif

/* ----------------------- 用户配置，可根据实际硬件修改 ----------------------- */

/* 仅启用需要使用的中断，避免重复定义或无效中断 */
#define SR04_TIMER1_IRQ_HANDLER_ENABLE	0
#define SR04_TIMER2_IRQ_HANDLER_ENABLE	1
#define SR04_TIMER3_IRQ_HANDLER_ENABLE	0
#define SR04_TIMER4_IRQ_HANDLER_ENABLE	0
#define SR04_TIMER5_IRQ_HANDLER_ENABLE	0
/* -------------------------------------------------------------------------- */

#ifndef ETIMEDOUT 
#define ETIMEDOUT	7
#endif

/* 配置结构体 */
typedef struct {
	timer_periph_t timer_periph;	// 定时器外设，每个SR04设备需要使用不同的定时器
	uint8_t 	   ic_channel;
	gpio_port_t    trig_port;
	gpio_pin_t 	   trig_pin;
	gpio_port_t    echo_port;
	gpio_pin_t     echo_pin;
	uint8_t		   pre_priority;	// 抢占优先级
	uint8_t		   sub_priority;	// 响应优先级
	void (*delay_us)(uint32_t us);
	void (*delay_ms)(uint32_t ms);
} sr04_cfg_t;

typedef struct sr04_dev sr04_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*get_distance)(sr04_dev_t *dev, float *distance_cm);
	int (*deinit)(sr04_dev_t *dev);
} sr04_ops_t;

/* 设备结构体 */
typedef struct sr04_dev {
	void *priv;
	sr04_cfg_t cfg;
	const sr04_ops_t *ops;
} sr04_dev_t;

/**
 * @brief   初始化 SR04 设备驱动
 * @param[out] dev sr04_dev_t 结构体指针
 * @param[in]  cfg sr04_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_sr04_init(sr04_dev_t *dev, const sr04_cfg_t *cfg);

#endif
