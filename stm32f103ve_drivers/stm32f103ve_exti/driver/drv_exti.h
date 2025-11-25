#ifndef DRV_EXTI_H
#define DRV_EXTI_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
#define DRV_EXTI_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef GPIO_TypeDef*		gpio_port_t;
typedef uint32_t			gpio_pin_t;
typedef IRQn_Type           iqrn_type_t;
typedef uint32_t			exti_line_t;
typedef EXTITrigger_TypeDef	exti_trigger_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_EXTI_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef GPIO_TypeDef*       gpio_port_t;
typedef uint32_t            gpio_pin_t;
typedef IRQn_Type           iqrn_type_t;
typedef uint32_t			exti_line_t;
typedef EXTITrigger_TypeDef	exti_trigger_t;

#elif defined (GD32F10X_MD) || defined(GD32F10X_HD)
#define DRV_EXTI_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t			gpio_port_t;
typedef uint32_t			gpio_pin_t;
typedef IRQn_Type			iqrn_type_t;
typedef exti_line_enum		exti_line_t;
typedef exti_trig_type_enum	exti_trigger_t;
	
#else
#error drv_exti.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
#define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW 0
#endif

typedef void (*exti_irq_callback_t)(void *param);

/* 配置结构体 */
typedef struct {
	gpio_port_t    port;
	gpio_pin_t 	   pin;
	exti_trigger_t trigger;			// 外部中断触发方式
	uint8_t 	   pre_priority;	// 抢占优先级
	uint8_t 	   sub_priority;	// 响应优先级
} exti_cfg_t;

typedef struct exti_dev exti_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*register_falling_irq_callback)(exti_dev_t *dev,
										 exti_irq_callback_t callback, 
										 void *param);
	int (*register_rising_irq_callback)(exti_dev_t *dev, 
									    exti_irq_callback_t callback, 
									    void *param);
	int (*deinit)(exti_dev_t *dev);
} exti_ops_t;

/* EXTI设备结构体 */
struct exti_dev {
	void 			   *priv;
	exti_cfg_t  	    cfg;
	const exti_ops_t   *ops;
	exti_irq_callback_t falling_irq_callback;
	void 		 	   *falling_irq_callback_param;
	exti_irq_callback_t rising_irq_callback;
	void 		 	   *rising_irq_callback_param;
};

/**
 * @brief   初始化 EXTI
 * @param[out] dev exti_dev_t 结构体指针
 * @param[in]  cfg exti_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */		
int drv_exti_init(exti_dev_t *dev, const exti_cfg_t *cfg);

#endif
