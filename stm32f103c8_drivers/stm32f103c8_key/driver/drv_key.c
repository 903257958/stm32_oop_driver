#include "drv_key.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void key_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_KEY_PLATFORM_STM32F1
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
#elif DRV_KEY_PLATFORM_STM32F4
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
#elif DRV_KEY_PLATFORM_GD32F1
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
 * @param[in] cfg key_cfg_t 结构体指针
 */
static void key_hw_gpio_init(const key_cfg_t *cfg)
{
#if DRV_KEY_PLATFORM_STM32F1
    GPIO_InitTypeDef GPIO_InitStructure;
	if (cfg->press_level)
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	else
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->pin;
	GPIO_Init(cfg->port, &GPIO_InitStructure);

#elif DRV_KEY_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	if (cfg->press_level)
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	else
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->pin;
	GPIO_Init(cfg->port, &GPIO_InitStructure);

#elif DRV_KEY_PLATFORM_GD32F1
	if (cfg->press_level)
		gpio_init(cfg->port, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, cfg->pin);
	else
		gpio_init(cfg->port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, cfg->pin);
#endif
}

/**
 * @brief	GPIO 读输入引脚电平
 * @param[in] port 端口
 * @param[in] pin  引脚
 * @return	电平
 */
static inline uint8_t key_hw_gpio_read_in_bit(gpio_port_t port, gpio_pin_t pin)
{
#if DRV_KEY_PLATFORM_STM32F1 || DRV_KEY_PLATFORM_STM32F4
	return GPIO_ReadInputDataBit(port, pin);
#elif DRV_KEY_PLATFORM_GD32F1
	return (uint8_t)gpio_input_bit_get(port, pin);
#endif
}

/**
 * @brief   初始化按键硬件
 * @param[in] cfg key_cfg_t 结构体指针
 */
static void key_hw_init(const key_cfg_t *cfg)
{	
	key_hw_gpio_clock_enable(cfg->port);
	key_hw_gpio_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */
                                
static int key_get_status_impl(key_dev_t *dev, bool *status);
static int key_deinit_impl(key_dev_t *dev);

/* 操作接口表 */
static const key_ops_t key_ops = {
	.get_status = key_get_status_impl,
	.deinit 	= key_deinit_impl
};

/**
 * @brief   初始化按键设备驱动
 * @param[out] dev key_dev_t 结构体指针
 * @param[in]  cfg key_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_key_init(key_dev_t *dev, const key_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

	dev->cfg = *cfg;
	dev->ops = &key_ops;
	
	key_hw_init(cfg);
	return 0;
}

/**
 * @brief   获取按键状态
 * @param[in] dev 	 key_dev_t 结构体指针
 * @param[in] status true 表示按下，false 表示未按下
 * @return	0 表示成功，其他值表示失败
 */
static int key_get_status_impl(key_dev_t *dev, bool *status)
{
	if (!dev)
		return -EINVAL;
	
	*status = false;

	if (key_hw_gpio_read_in_bit(dev->cfg.port, dev->cfg.pin) == dev->cfg.press_level) {
		dev->cfg.delay_ms(20);
		while((key_hw_gpio_read_in_bit(dev->cfg.port, dev->cfg.pin) == dev->cfg.press_level));
        dev->cfg.delay_ms(20);
		*status = true;
	}
	
	return 0;
}

/**
 * @brief   去初始化按键
 * @param[in] dev key_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int key_deinit_impl(key_dev_t *dev)
{
	if (!dev)
		return -EINVAL;
	
	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
