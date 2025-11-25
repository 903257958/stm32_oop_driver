#ifndef DRV_LED_H
#define DRV_LED_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_LED_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_LED_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
#define DRV_LED_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t	gpio_port_t;
typedef uint32_t	gpio_pin_t;

#else
#error drv_led.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
#define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW  0
#endif

typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_ON  = 1
} led_state_t;

/* 配置结构体 */
typedef struct {
	gpio_port_t port;
	gpio_pin_t 	pin;
	uint8_t		active_level;
	led_state_t init_state;
} led_cfg_t;

typedef struct led_dev led_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*on)(led_dev_t *dev);
	int (*off)(led_dev_t *dev);
	int (*toggle)(led_dev_t *dev);
	int (*get_state)(led_dev_t *dev, led_state_t *state);
	int (*deinit)(led_dev_t *dev);
} led_ops_t;

/* 设备结构体 */
struct led_dev {
	led_cfg_t cfg;
	const led_ops_t *ops;
};

/**
 * @brief   初始化 LED 设备驱动
 * @param[out] dev led_dev_t 结构体指针
 * @param[in]  cfg led_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_led_init(led_dev_t *dev, const led_cfg_t *cfg);

#endif
