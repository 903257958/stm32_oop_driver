#ifndef DRV_PWM_H
#define DRV_PWM_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_PWM_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef TIM_TypeDef*	timer_periph_t;
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_PWM_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef TIM_TypeDef*	timer_periph_t;
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#else
#error drv_pwm.h: No processor defined!
#endif

/* 配置结构体 */
typedef struct {
	timer_periph_t timer_periph;
	uint8_t 	   oc_channel;
	uint16_t 	   psc;
	uint16_t 	   arr;
	gpio_port_t    port;
	gpio_pin_t     pin;
} pwm_cfg_t;

typedef struct pwm_dev pwm_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*set_psc)(pwm_dev_t *dev, uint16_t psc);
	int (*set_arr)(pwm_dev_t *dev, uint16_t arr);
	int (*set_compare)(pwm_dev_t *dev, uint16_t compare);
	int (*deinit)(pwm_dev_t *dev);
} pwm_ops_t;

/* 设备结构体 */
struct pwm_dev {
	pwm_cfg_t cfg;
	const pwm_ops_t *ops;
};

/**
 * @brief   初始化 PWM 驱动（不包括高级定时器）
 * @param[out] dev pwm_dev_t 结构体指针
 * @param[in]  cfg pwm_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
int drv_pwm_init(pwm_dev_t *dev, const pwm_cfg_t *cfg);

#endif
