#ifndef DRV_TIMER_H
#define DRV_TIMER_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_TIMER_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef TIM_TypeDef*	timer_periph_t;
typedef IRQn_Type		iqrn_type_t;

#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)
#define DRV_TIMER_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef TIM_TypeDef*	timer_periph_t;
typedef IRQn_Type		iqrn_type_t;

#elif defined (GD32F10X_MD)
#define DRV_TIMER_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t	timer_periph_t;
typedef IRQn_Type	iqrn_type_t;

#else
#error drv_timer.h: No processor defined!
#endif

/* ----------------------- 用户配置，可根据实际硬件修改 ----------------------- */

/* 仅启用需要使用的中断，避免重复定义或无效中断 */
#define TIMER1_IRQ_HANDLER_ENABLE	0
#define TIMER2_IRQ_HANDLER_ENABLE	1
#define TIMER3_IRQ_HANDLER_ENABLE	0
#define TIMER4_IRQ_HANDLER_ENABLE	0
#define TIMER5_IRQ_HANDLER_ENABLE	0
#define TIMER6_IRQ_HANDLER_ENABLE	0
#define TIMER7_IRQ_HANDLER_ENABLE	0
/* -------------------------------------------------------------------------- */

typedef void (*timer_irq_callback_t)(void *param);

/* 配置结构体 */
typedef struct {
	timer_periph_t 		 timer_periph;
	uint16_t 			 psc;
	uint16_t 			 arr;
	uint8_t 			 pre_priority;	// 抢占优先级（仅use_irq=true时有效）
	uint8_t 			 sub_priority;	// 响应优先级（仅use_irq=true时有效）
    bool                 use_irq;
} timer_cfg_t;

typedef struct timer_dev timer_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*delay_us)(timer_dev_t *dev, uint32_t us);
	int (*delay_ms)(timer_dev_t *dev, uint32_t ms);
	int (*register_irq_callback)(timer_dev_t *dev,
								 timer_irq_callback_t callback, 
								 void *param);
	int (*deinit)(timer_dev_t *dev);
} timer_ops_t;

/* 设备结构体 */
struct timer_dev {
	void 				*priv;
	timer_cfg_t 	     cfg;
	const timer_ops_t   *ops;
	timer_irq_callback_t irq_callback;
	void 				*irq_callback_param;
};

/**
 * @brief   初始化定时器设备驱动（不包括高级定时器）
 * @details 以 STM32F1 为例，定时器时钟为 72 MHz，计数频率由分频系数 PSC 决定：
 *          计数频率 = 72 MHz / (PSC + 1)，其倒数为计数周期。
 *          - 当用于微秒级延时时，需配置 PSC = (TIMER_FREQ / 1000000 - 1)，
 *            此时计数周期为 1 µs
 *          - 当用于毫秒级延时时，需配置 ARR >= 1000，
 *            定时周期 = (ARR + 1) µs，最大约 65.5 ms。
 *          - 若 PSC 或 ARR 未正确配置，
 *            则该定时器仅可用于中断，无法提供延时功能。
 * @param[out] dev timer_dev_t 结构体指针
 * @param[in]  cfg timer_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
int drv_timer_init(timer_dev_t *dev, const timer_cfg_t *cfg);

#endif
