#include "drv_gpio.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void gpio_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_GPIO_PLATFORM_STM32F1
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
#elif DRV_GPIO_PLATFORM_STM32F4
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
#elif DRV_GPIO_PLATFORM_GD32F1
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
 * @param[in] cfg gpio_cfg_t 结构体指针
 */
static void gpio_hw_gpio_init(const gpio_cfg_t *cfg)
{
#if DRV_GPIO_PLATFORM_STM32F1
    GPIO_InitTypeDef GPIO_InitStructure;
	switch (cfg->mode) {
	case IN_PU:  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		   break;
	case IN_PD:  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		   break;
	case IN_PN:  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; break;
	case OUT_PP: GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      break;
	case OUT_OD: GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;      break;
	default: break;
	}
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->pin;
	GPIO_Init(cfg->port, &GPIO_InitStructure);

#elif DRV_GPIO_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
	switch (cfg->mode) {
	case IN_PU:
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		break;
	case IN_PD:
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		break;
	case IN_PN:
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		break;
	case OUT_PP:
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		break;
	case OUT_OD:
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		break;
	default:
		break;
	}
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->pin;
	GPIO_Init(cfg->port, &GPIO_InitStructure);

#elif DRV_GPIO_PLATFORM_GD32F1
	switch (cfg->mode) {
	case IN_PU:  gpio_init(cfg->port, GPIO_MODE_IPU, 		 GPIO_OSPEED_50MHZ, cfg->pin); break;
	case IN_PD:  gpio_init(cfg->port, GPIO_MODE_IPD, 		 GPIO_OSPEED_50MHZ, cfg->pin); break;
	case IN_PN:  gpio_init(cfg->port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, cfg->pin); break;
	case OUT_PP: gpio_init(cfg->port, GPIO_MODE_OUT_PP, 	 GPIO_OSPEED_50MHZ, cfg->pin); break;
	case OUT_OD: gpio_init(cfg->port, GPIO_MODE_OUT_OD, 	 GPIO_OSPEED_50MHZ, cfg->pin); break;
	default: break;
	}
#endif
}

/**
 * @brief	GPIO 读输入引脚电平
 * @param[in] port 端口
 * @param[in] pin  引脚
 * @return	电平
 */
static inline uint8_t gpio_hw_gpio_read_in_bit(gpio_port_t port, gpio_pin_t pin)
{
#if DRV_GPIO_PLATFORM_STM32F1 || DRV_GPIO_PLATFORM_STM32F4
	return GPIO_ReadInputDataBit(port, pin);
#elif DRV_GPIO_PLATFORM_GD32F1
	return (uint8_t)gpio_input_bit_get(port, pin);
#endif
}

/**
 * @brief	GPIO 读输出引脚电平
 * @param[in] port 端口
 * @param[in] pin  引脚
 * @return	电平
 */
static inline uint8_t gpio_hw_gpio_read_out_bit(gpio_port_t port, gpio_pin_t pin)
{
#if DRV_GPIO_PLATFORM_STM32F1 || DRV_GPIO_PLATFORM_STM32F4
	return GPIO_ReadOutputDataBit(port, pin);
#elif DRV_GPIO_PLATFORM_GD32F1
	return (uint8_t)gpio_output_bit_get(port, pin);
#endif
}

/**
 * @brief	写 GPIO 引脚电平
 * @param[in] port  端口
 * @param[in] pin   引脚
 * @param[in] level 电平
 */
static inline void gpio_hw_gpio_write_bit(gpio_port_t port, gpio_pin_t pin, uint8_t level)
{
#if DRV_GPIO_PLATFORM_STM32F1 || DRV_GPIO_PLATFORM_STM32F4
	GPIO_WriteBit(port, pin, (BitAction)level);
#elif DRV_GPIO_PLATFORM_GD32F1
	gpio_bit_write(port, pin, (bit_status)(level));
#endif
}

/**
 * @brief   初始化 GPIO 硬件
 * @param[in] cfg gpio_cfg_t 结构体指针
 */
static void gpio_hw_init(const gpio_cfg_t *cfg)
{	
	gpio_hw_gpio_clock_enable(cfg->port);
	gpio_hw_gpio_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

static int gpio_set_impl(gpio_dev_t *dev);
static int gpio_reset_impl(gpio_dev_t *dev);
static int gpio_read_impl(gpio_dev_t *dev, uint8_t *level);
static int gpio_write_impl(gpio_dev_t *dev, uint8_t level);
static int gpio_toggle_impl(gpio_dev_t *dev);
static int gpio_deinit_impl(gpio_dev_t *dev);

/* 操作接口表 */
static const gpio_ops_t gpio_ops = {
	.set    = gpio_set_impl,
	.reset  = gpio_reset_impl,
	.read   = gpio_read_impl,
	.write  = gpio_write_impl,
	.toggle = gpio_toggle_impl,
	.deinit = gpio_deinit_impl
};

/**
 * @brief   初始化 GPIO 设备驱动
 * @param[out] dev gpio_dev_t 结构体指针
 * @param[in]  cfg gpio_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_gpio_init(gpio_dev_t *dev, const gpio_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

	dev->cfg = *cfg;
	dev->ops = &gpio_ops;

	gpio_hw_init(cfg);
	return 0;
}

/**
 * @brief   GPIO 置位
 * @param[in] dev gpio_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int gpio_set_impl(gpio_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	if (dev->cfg.mode != OUT_PP && dev->cfg.mode != OUT_OD)
		return -EINVAL;
	
	gpio_hw_gpio_write_bit(dev->cfg.port, dev->cfg.pin, GPIO_LEVEL_HIGH);
	return 0;
}

/**
 * @brief   GPIO 复位
 * @param[in] dev gpio_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int gpio_reset_impl(gpio_dev_t *dev)
{
	if (!dev)
		return -EINVAL;
	
	if (dev->cfg.mode != OUT_PP && dev->cfg.mode != OUT_OD)
		return -EINVAL;
	
	gpio_hw_gpio_write_bit(dev->cfg.port, dev->cfg.pin, GPIO_LEVEL_LOW);
	return 0;
}

/**
 * @brief   GPIO 读电平
 * @param[in]  dev   gpio_dev_t 结构体指针
 * @param[out] level 电平值
 * @return	0 表示成功，其他值表示失败
 */
static int gpio_read_impl(gpio_dev_t *dev, uint8_t *level)
{
	if (!dev)
		return -EINVAL;

	if (dev->cfg.mode == OUT_PP || dev->cfg.mode == OUT_OD)
		*level = gpio_hw_gpio_read_out_bit(dev->cfg.port, dev->cfg.pin);
	else
		*level = gpio_hw_gpio_read_in_bit(dev->cfg.port, dev->cfg.pin);
	
	return 0;
}

/**
 * @brief   GPIO 写电平
 * @param[in] dev   gpio_dev_t 结构体指针
 * @param[in] level 电平值
 * @return	0 表示成功，其他值表示失败
 */
static int gpio_write_impl(gpio_dev_t *dev, uint8_t level)
{
	if (!dev)
		return -EINVAL;

	if (dev->cfg.mode != OUT_PP && dev->cfg.mode != OUT_OD)
		return -EINVAL;
		
	gpio_hw_gpio_write_bit(dev->cfg.port, dev->cfg.pin, level);
	return 0;
}

/**
 * @brief   GPIO 翻转
 * @param[in] dev gpio_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int gpio_toggle_impl(gpio_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	if (dev->cfg.mode != OUT_PP && dev->cfg.mode != OUT_OD)
		return -EINVAL;
	
	if (gpio_hw_gpio_read_out_bit(dev->cfg.port, dev->cfg.pin) == GPIO_LEVEL_HIGH)
		gpio_hw_gpio_write_bit(dev->cfg.port, dev->cfg.pin, GPIO_LEVEL_LOW);
	else
		gpio_hw_gpio_write_bit(dev->cfg.port, dev->cfg.pin, GPIO_LEVEL_HIGH);
	
	return 0;
}

/**
 * @brief   去初始化 GPIO
 * @param[in] dev gpio_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int gpio_deinit_impl(gpio_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
