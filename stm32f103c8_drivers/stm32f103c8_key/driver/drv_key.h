#ifndef DRV_KEY_H
#define DRV_KEY_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_KEY_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_KEY_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
#define DRV_KEY_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t	gpio_port_t;
typedef uint32_t	gpio_pin_t;

#else
#error drv_key.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
#define GPIO_LEVEL_HIGH	1
#endif

#ifndef GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW 	0
#endif

/* 配置结构体 */
typedef struct {
	void (*delay_ms)(uint32_t ms);
	gpio_port_t port;
	gpio_pin_t pin;
	bool press_level;   // 按下后的电平
} key_cfg_t;

typedef struct key_dev key_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*get_status)(key_dev_t *dev, bool *pressed);
	int (*deinit)(key_dev_t *dev);
} key_ops_t;

/* 设备结构体 */
typedef struct key_dev {
	key_cfg_t cfg;
	const key_ops_t *ops;
} key_dev_t;

/**
 * @brief   初始化按键设备驱动
 * @param[out] dev key_dev_t 结构体指针
 * @param[in]  cfg key_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_key_init(key_dev_t *dev, const key_cfg_t *cfg);

#endif
