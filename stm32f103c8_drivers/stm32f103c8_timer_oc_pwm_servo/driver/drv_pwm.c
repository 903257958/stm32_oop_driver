#include "drv_pwm.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void pwm_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_PWM_PLATFORM_STM32F1
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
#elif DRV_PWM_PLATFORM_STM32F4
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
#endif
}

/**
 * @brief	使能定时器外设时钟
 * @param[in] timer_periph 定时器外设
 */
static void pwm_hw_timer_clock_enable(timer_periph_t timer_periph)
{
#if DRV_PWM_PLATFORM_STM32F1 || DRV_PWM_PLATFORM_STM32F4
	switch ((uint32_t)timer_periph) {
	case (uint32_t)TIM2: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); break;
	case (uint32_t)TIM3: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); break;
	case (uint32_t)TIM4: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); break;
	case (uint32_t)TIM5: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); break;
	default: break;
    }
#endif
}

#if DRV_PWM_PLATFORM_STM32F4
/**
 * @brief	获取 GPIO 引脚源（STM32F4特有）
 * @param[in] pin GPIO 引脚
 * @return	引脚源编号（如9对应PinSource9）
 */
static inline uint8_t pwm_hw_get_gpio_pin_source(gpio_pin_t pin)
{
    for (uint8_t i = 0; i < 16; i++)
        if (pin & (1 << i))
			return i;  // GPIO_Pin = 1<<i，对应 PinSource = i
    return 0xFF;
}

/**
 * @brief	获取 GPIO AF（STM32F4特有）
 * @param[in] timer_periph 定时器外设
 * @return	GPIO AF
 */
static inline uint8_t pwm_hw_get_gpio_af(timer_periph_t timer_periph)
{
    switch ((uint32_t)timer_periph) {
	case (uint32_t)TIM2: return GPIO_AF_TIM2;
	case (uint32_t)TIM3: return GPIO_AF_TIM3;
	case (uint32_t)TIM4: return GPIO_AF_TIM4;
	case (uint32_t)TIM5: return GPIO_AF_TIM5;
	default: return 0xFF;
	}
}

#endif	/* DRV_PWM_PLATFORM_STM32F4 */

/**
 * @brief	初始化 GPIO
 * @param[in] cfg pwm_cfg_t 结构体指针
 */
static void pwm_hw_gpio_init(const pwm_cfg_t *cfg)
{
#if DRV_PWM_PLATFORM_STM32F1
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = cfg->pin;
    GPIO_Init(cfg->port, &GPIO_InitStructure);

#elif DRV_PWM_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = cfg->pin;
    GPIO_Init(cfg->port, &GPIO_InitStructure);

	GPIO_PinAFConfig(cfg->port, 
					 pwm_hw_get_gpio_pin_source(cfg->pin), 
					 pwm_hw_get_gpio_af(cfg->timer_periph));
#endif
}

/**
 * @brief	初始化定时器外设
 * @param[in] cfg pwm_cfg_t 结构体指针
 */
static void pwm_hw_timer_init(const pwm_cfg_t *cfg)
{
#if DRV_PWM_PLATFORM_STM32F1 || DRV_PWM_PLATFORM_STM32F4
	/* 配置时钟源为内部时钟 */
	TIM_InternalClockConfig(cfg->timer_periph);
	
	/* 时基单元初始化 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// 时钟分频参数，不分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	// 计数器模式：向上计数
	TIM_TimeBaseInitStructure.TIM_Prescaler = cfg->psc;				// PSC预分频器的值，范围0~65535
	TIM_TimeBaseInitStructure.TIM_Period = cfg->arr;				// ARR自动重装器的值，范围0~65535
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			// 重复计数器的值（高级定时器用）
	TIM_TimeBaseInit(cfg->timer_periph, &TIM_TimeBaseInitStructure);
	
	/* 配置输出比较 */
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);							// 给结构体赋默认值
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				// 设置输出比较模式：PWM模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		// 设置输出比较极性：高极性（REF极性不反转）
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	// 设置输出使能
	TIM_OCInitStructure.TIM_Pulse = 1;		// 设置CCR	Freq=CK_PSC/(PSC+1)/(ARR+1)	Duty=CCR/(ARR+1)	Reso=1/(ARR+1)
	switch (cfg->oc_channel) {
	case 1: TIM_OC1Init(cfg->timer_periph, &TIM_OCInitStructure); break;
	case 2: TIM_OC2Init(cfg->timer_periph, &TIM_OCInitStructure); break;
	case 3: TIM_OC3Init(cfg->timer_periph, &TIM_OCInitStructure); break;
	case 4: TIM_OC4Init(cfg->timer_periph, &TIM_OCInitStructure); break;
	default: break;
	}
	
	/* 启用定时器 */
	TIM_Cmd(cfg->timer_periph, ENABLE);
#endif
}

/**
 * @brief	开启/关闭定时器外设
 * @param[in] timer_periph 定时器外设
 * @param[in] is_enable    定时器状态
 */
static inline void pwm_hw_timer_cmd(timer_periph_t timer_periph, bool is_enable)
{
#if DRV_PWM_PLATFORM_STM32F1 || DRV_PWM_PLATFORM_STM32F4
	if (is_enable)
		TIM_Cmd(timer_periph, ENABLE);
	else
		TIM_Cmd(timer_periph, DISABLE);
#endif
}

/**
 * @brief	设置定时器的 PSC 值
 * @param[in] timer_periph 定时器外设
 * @param[in] psc    	   PSC 值
 */
static inline void pwm_hw_timer_set_psc(timer_periph_t timer_periph, uint16_t psc)
{
#if DRV_PWM_PLATFORM_STM32F1 || DRV_PWM_PLATFORM_STM32F4
	timer_periph->PSC = psc;
    timer_periph->EGR = TIM_PSCReloadMode_Update;	// 重新加载预分频器值
#endif
}

/**
 * @brief	设置定时器的 ARR 值
 * @param[in] timer_periph 定时器外设
 * @param[in] psc    	   ARR 值
 */
static inline void pwm_hw_timer_set_arr(timer_periph_t timer_periph, uint16_t arr)
{
#if DRV_PWM_PLATFORM_STM32F1 || DRV_PWM_PLATFORM_STM32F4
	timer_periph->ARR = arr;
    timer_periph->EGR = TIM_PSCReloadMode_Update;	// 重新加载预分频器值
#endif
}

/**
 * @brief	设置定时器的 CCR 值
 * @param[in] cfg pwm_cfg_t 结构体指针
 * @param[in] psc    	   CCR 值
 */
static inline void pwm_hw_timer_set_ccr(const pwm_cfg_t *cfg, uint16_t ccr)
{
#if DRV_PWM_PLATFORM_STM32F1 || DRV_PWM_PLATFORM_STM32F4
	switch (cfg->oc_channel) {
	case 1: TIM_SetCompare1(cfg->timer_periph, ccr); break;
	case 2: TIM_SetCompare2(cfg->timer_periph, ccr); break;
	case 3: TIM_SetCompare3(cfg->timer_periph, ccr); break;
	case 4: TIM_SetCompare4(cfg->timer_periph, ccr); break;
	default: break;
	}
#endif
}

/**
 * @brief   初始化 PWM 硬件
 * @param[in] cfg pwm_cfg_t 结构体指针
 */
static void pwm_hw_init(const pwm_cfg_t *cfg)
{
	pwm_hw_timer_clock_enable(cfg->timer_periph);
	pwm_hw_gpio_clock_enable(cfg->port);

	pwm_hw_gpio_init(cfg);
	pwm_hw_timer_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */
								
static int pwm_set_psc_impl(pwm_dev_t *dev, uint16_t psc);
static int pwm_set_arr_impl(pwm_dev_t *dev, uint16_t arr);
static int pwm_set_compare_impl(pwm_dev_t *dev, uint16_t compare);
static int pwm_deinit_impl(pwm_dev_t *dev);

/* 操作接口表 */
static const pwm_ops_t pwm_ops = {
	.set_psc 	 = pwm_set_psc_impl,
	.set_arr 	 = pwm_set_arr_impl,
	.set_compare = pwm_set_compare_impl,
	.deinit   	 = pwm_deinit_impl
};

/**
 * @brief   初始化 PWM 驱动（不包括高级定时器）
 * @param[out] dev pwm_dev_t 结构体指针
 * @param[in]  cfg pwm_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
int drv_pwm_init(pwm_dev_t *dev, const pwm_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

	dev->cfg = *cfg;
	dev->ops = &pwm_ops;
	
	pwm_hw_init(cfg);
	return 0;
}

/**
 * @brief   PWM 设置 PSC 寄存器的值
 * @param[in] dev pwm_dev_t 结构体指针
 * @param[in] psc 要写入的 PSC 的值
 * @return	0 表示成功，其他值表示失败
 */	
static int pwm_set_psc_impl(pwm_dev_t *dev, uint16_t psc)
{
	if (!dev)
        return -EINVAL;

	pwm_hw_timer_cmd(dev->cfg.timer_periph, false);
	pwm_hw_timer_set_psc(dev->cfg.timer_periph, psc);
	pwm_hw_timer_cmd(dev->cfg.timer_periph, true);
	return 0;
}

/**
 * @brief   PWM 设置 ARR 寄存器的值
 * @param[in] dev pwm_dev_t 结构体指针
 * @param[in] arr 要写入的 ARR 的值
 * @return	0 表示成功，其他值表示失败
 */	
static int pwm_set_arr_impl(pwm_dev_t *dev, uint16_t arr)
{
	if (!dev)
        return -EINVAL;
		
    pwm_hw_timer_cmd(dev->cfg.timer_periph, false);
	pwm_hw_timer_set_arr(dev->cfg.timer_periph, arr);
    pwm_hw_timer_cmd(dev->cfg.timer_periph, true);
	return 0;
}

/**
 * @brief   PWM 设置 CCR 的值
 * @param[in] dev pwm_dev_t 结构体指针
 * @param[in] arr 要写入的 CCR 的值
 * @return	0 表示成功，其他值表示失败
 */	
static int pwm_set_compare_impl(pwm_dev_t *dev, uint16_t compare)
{
	if (!dev)
        return -EINVAL;

	pwm_hw_timer_set_ccr(&dev->cfg, compare);
	return 0;
}

/**
 * @brief   去初始化 PWM
 * @param[in] dev pwm_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int pwm_deinit_impl(pwm_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
