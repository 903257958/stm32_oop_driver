#ifndef DRV_GPIO_H
#define DRV_GPIO_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
#define DRV_GPIO_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_GPIO_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
#define DRV_GPIO_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t	gpio_port_t;
typedef uint32_t	gpio_pin_t;

#else
#error drv_gpio.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
#define GPIO_LEVEL_HIGH	1
#endif

#ifndef GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW	0
#endif

typedef enum {
	IN_PU = 0,	// 上拉输入
	IN_PD,		// 下拉输入
	IN_PN,		// 浮空输入
	OUT_PP,		// 推挽输出
	OUT_OD,		// 开漏输出
} gpio_mode_t;

/* 配置结构体 */
typedef struct {
	gpio_port_t port;
	gpio_pin_t pin;
	gpio_mode_t mode;
} gpio_cfg_t;

typedef struct gpio_dev gpio_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*set)(gpio_dev_t *dev);
	int (*reset)(gpio_dev_t *dev);
	int (*read)(gpio_dev_t *dev, uint8_t *level);
	int (*write)(gpio_dev_t *dev, uint8_t level);
	int (*toggle)(gpio_dev_t *dev);
	int (*deinit)(gpio_dev_t *dev);
} gpio_ops_t;

/* 设备结构体 */
struct gpio_dev {
	gpio_cfg_t cfg;
	const gpio_ops_t *ops;
};

/**
 * @brief   初始化 GPIO 设备驱动
 * @param[out] dev gpio_dev_t 结构体指针
 * @param[in]  cfg gpio_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_gpio_init(gpio_dev_t *dev, const gpio_cfg_t *cfg);

#endif
