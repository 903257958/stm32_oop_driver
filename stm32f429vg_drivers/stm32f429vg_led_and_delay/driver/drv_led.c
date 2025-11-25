#include "drv_led.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void led_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_LED_PLATFORM_STM32F1
	switch ((uint32_t)port) {
	case (uint32_t)GPIOA: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); break;
	case (uint32_t)GPIOB: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); break;
	case (uint32_t)GPIOC: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); break;
	case (uint32_t)GPIOD: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); break;
	case (uint32_t)GPIOE: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); break;
	case (uint32_t)GPIOF: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); break;
	case (uint32_t)GPIOG: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE); break;
	default: break;
    }
#elif DRV_LED_PLATFORM_STM32F4
	switch ((uint32_t)port) {
	case (uint32_t)GPIOA: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); break;
	case (uint32_t)GPIOB: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); break;
	case (uint32_t)GPIOC: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); break;
	case (uint32_t)GPIOD: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); break;
	case (uint32_t)GPIOE: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); break;
	case (uint32_t)GPIOF: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); break;
	case (uint32_t)GPIOG: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); break;
	default: break;
    }
#elif DRV_LED_PLATFORM_GD32F1
	switch ((uint32_t)port) {
	case (uint32_t)GPIOA: rcu_periph_clock_enable(RCU_GPIOA); break;
	case (uint32_t)GPIOB: rcu_periph_clock_enable(RCU_GPIOB); break;
	case (uint32_t)GPIOC: rcu_periph_clock_enable(RCU_GPIOC); break;
	case (uint32_t)GPIOD: rcu_periph_clock_enable(RCU_GPIOD); break;
	case (uint32_t)GPIOE: rcu_periph_clock_enable(RCU_GPIOE); break;
	case (uint32_t)GPIOF: rcu_periph_clock_enable(RCU_GPIOF); break;
	case (uint32_t)GPIOG: rcu_periph_clock_enable(RCU_GPIOG); break;
	default: break;
    }
#endif
}

/**
 * @brief	初始化 GPIO
 * @param[in] cfg led_cfg_t 结构体指针
 */
static void led_hw_gpio_init(const led_cfg_t *cfg)
{
#if DRV_LED_PLATFORM_STM32F1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->pin;
	GPIO_Init(cfg->port, &GPIO_InitStructure);
#elif DRV_LED_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->pin;
	GPIO_Init(cfg->port, &GPIO_InitStructure);
#elif DRV_LED_PLATFORM_GD32F1
	gpio_init(cfg->port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, cfg->pin);
#endif
}

/**
 * @brief	写 GPIO 引脚电平
 * @param[in] port  端口
 * @param[in] pin   引脚
 * @param[in] level 电平
 */
static inline void led_hw_gpio_write_bit(gpio_port_t port, gpio_pin_t pin, uint8_t level)
{
#if DRV_LED_PLATFORM_STM32F1 || DRV_LED_PLATFORM_STM32F4
	GPIO_WriteBit(port, pin, (BitAction)level);
#elif DRV_LED_PLATFORM_GD32F1
	gpio_bit_write(port, pin, (bit_status)(level));
#endif
}

/**
 * @brief	GPIO 读输出引脚电平
 * @param[in] port 端口
 * @param[in] pin  引脚
 * @return	电平
 */
static inline uint8_t led_hw_gpio_read_out_bit(gpio_port_t port, gpio_pin_t pin)
{
#if DRV_LED_PLATFORM_STM32F1 || DRV_LED_PLATFORM_STM32F4
	return GPIO_ReadOutputDataBit(port, pin);
#elif DRV_LED_PLATFORM_GD32F1
	return (uint8_t)gpio_output_bit_get(port, pin);
#endif
}

/**
 * @brief   初始化 LED 硬件
 * @param[in] cfg led_cfg_t 结构体指针
 */
static void led_hw_init(const led_cfg_t *cfg)
{
	led_hw_gpio_clock_enable(cfg->port);
	led_hw_gpio_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

static int led_on_impl(led_dev_t *dev);
static int led_off_impl(led_dev_t *dev);
static int led_toggle_impl(led_dev_t *dev);
static int led_get_state_impl(led_dev_t *dev, led_state_t *state);
static int led_deinit_impl(led_dev_t *dev);

/* 操作接口表 */
static const led_ops_t led_ops = {
	.on 	   = led_on_impl,
	.off 	   = led_off_impl,
	.toggle    = led_toggle_impl,
	.get_state = led_get_state_impl,
	.deinit	   = led_deinit_impl
};

/**
 * @brief   初始化 LED 设备驱动
 * @param[out] dev led_dev_t 结构体指针
 * @param[in]  cfg led_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_led_init(led_dev_t *dev, const led_cfg_t *cfg)
{
    if (!dev || !cfg)
        return -EINVAL;
    
    dev->cfg = *cfg;
    dev->ops = &led_ops;
    
    led_hw_init(cfg);

    if (dev->cfg.init_state == LED_STATE_ON)
        led_on_impl(dev);
    else
        led_off_impl(dev);

    return 0;
}

/**
 * @brief   打开 LED
 * @param[in] dev led_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int led_on_impl(led_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	led_hw_gpio_write_bit(dev->cfg.port, dev->cfg.pin, dev->cfg.active_level);
	return 0;
}

/**
 * @brief   关闭 LED
 * @param[in] dev led_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int led_off_impl(led_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	led_hw_gpio_write_bit(dev->cfg.port, dev->cfg.pin, !dev->cfg.active_level);
	return 0;
}

/**
 * @brief   翻转 LED
 * @param[in] dev led_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int led_toggle_impl(led_dev_t *dev)
{
	if (!dev)
		return -EINVAL;
    
    uint8_t cur_level = led_hw_gpio_read_out_bit(dev->cfg.port, dev->cfg.pin);
    led_state_t cur_state = (cur_level == dev->cfg.active_level) ? LED_STATE_ON : LED_STATE_OFF;
    
	return (cur_state == LED_STATE_OFF) ? led_on_impl(dev) : led_off_impl(dev);
}

/**
 * @brief   获取 LED 的状态
 * @param[in]  dev    led_dev_t 结构体指针
 * @param[out] state 状态：false 灭，true 亮
 * @return	0 表示成功，其他值表示失败
 */
static int led_get_state_impl(led_dev_t *dev, led_state_t *state)
{
	if (!dev || !state)
		return -EINVAL;

    uint8_t cur_level = led_hw_gpio_read_out_bit(dev->cfg.port, dev->cfg.pin);
    
    *state = (cur_level == dev->cfg.active_level) ? LED_STATE_ON : LED_STATE_OFF;
	return 0;
}

/**
 * @brief   去初始化 LED
 * @param[in] dev led_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int led_deinit_impl(led_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	led_off_impl(dev);
	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
