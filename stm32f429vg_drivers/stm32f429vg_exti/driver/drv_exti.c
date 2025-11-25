#include "drv_exti.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

/* 硬件信息结构体 */
typedef struct {
	gpio_pin_t pin;
	iqrn_type_t iqrn;
	exti_line_t line;
	uint8_t idx;
} exti_hw_info_t;

/* 硬件信息列表 */
static const exti_hw_info_t exti_hw_info_table[] = {
#if DRV_EXTI_PLATFORM_STM32F1 || DRV_EXTI_PLATFORM_STM32F4
	{ GPIO_Pin_0,  EXTI0_IRQn,     EXTI_Line0,  0  },
	{ GPIO_Pin_1,  EXTI1_IRQn,     EXTI_Line1,  1  },
	{ GPIO_Pin_2,  EXTI2_IRQn,     EXTI_Line2,  2  },
	{ GPIO_Pin_3,  EXTI3_IRQn,     EXTI_Line3,  3  },
	{ GPIO_Pin_4,  EXTI4_IRQn,     EXTI_Line4,  4  },
	{ GPIO_Pin_5,  EXTI9_5_IRQn,   EXTI_Line5,  5  },
	{ GPIO_Pin_6,  EXTI9_5_IRQn,   EXTI_Line6,  6  },
	{ GPIO_Pin_7,  EXTI9_5_IRQn,   EXTI_Line7,  7  },
	{ GPIO_Pin_8,  EXTI9_5_IRQn,   EXTI_Line8,  8  },
	{ GPIO_Pin_9,  EXTI9_5_IRQn,   EXTI_Line9,  9  },
	{ GPIO_Pin_10, EXTI15_10_IRQn, EXTI_Line10, 10 },
	{ GPIO_Pin_11, EXTI15_10_IRQn, EXTI_Line11, 11 },
	{ GPIO_Pin_12, EXTI15_10_IRQn, EXTI_Line12, 12 },
	{ GPIO_Pin_13, EXTI15_10_IRQn, EXTI_Line13, 13 },
	{ GPIO_Pin_14, EXTI15_10_IRQn, EXTI_Line14, 14 },
	{ GPIO_Pin_15, EXTI15_10_IRQn, EXTI_Line15, 15 },

#elif DRV_EXTI_PLATFORM_GD32F1
	{ GPIO_PIN_0,  EXTI0_IRQn,     EXTI_0,  0  },
	{ GPIO_PIN_1,  EXTI1_IRQn,     EXTI_1,  1  },
	{ GPIO_PIN_2,  EXTI2_IRQn,     EXTI_2,  2  },
	{ GPIO_PIN_3,  EXTI3_IRQn,     EXTI_3,  3  },
	{ GPIO_PIN_4,  EXTI4_IRQn,     EXTI_4,  4  },
	{ GPIO_PIN_5,  EXTI5_9_IRQn,   EXTI_5,  5  },
	{ GPIO_PIN_6,  EXTI5_9_IRQn,   EXTI_6,  6  },
	{ GPIO_PIN_7,  EXTI5_9_IRQn,   EXTI_7,  7  },
	{ GPIO_PIN_8,  EXTI5_9_IRQn,   EXTI_8,  8  },
	{ GPIO_PIN_9,  EXTI5_9_IRQn,   EXTI_9,  9  },
	{ GPIO_PIN_10, EXTI10_15_IRQn, EXTI_10, 10 },
	{ GPIO_PIN_11, EXTI10_15_IRQn, EXTI_11, 11 },
	{ GPIO_PIN_12, EXTI10_15_IRQn, EXTI_12, 12 },
	{ GPIO_PIN_13, EXTI10_15_IRQn, EXTI_13, 13 },
	{ GPIO_PIN_14, EXTI10_15_IRQn, EXTI_14, 14 },
	{ GPIO_PIN_15, EXTI10_15_IRQn, EXTI_15, 15 },
#endif
};

#define MAX_EXTI_NUM	(sizeof(exti_hw_info_table) / sizeof(exti_hw_info_t))

/**
 * @brief	获取 EXTI 硬件信息
 * @param[in] pin 引脚
 * @return	成功返回对应硬件信息指针，失败返回 NULL
 */
static inline const exti_hw_info_t* exti_get_hw_info(gpio_pin_t pin)
{
    for (uint8_t i = 0; i < MAX_EXTI_NUM; i++) {
        if (exti_hw_info_table[i].pin == pin) {
            return &exti_hw_info_table[i];
        }
    }
    return NULL;
}

/* 内部使用的触发模式枚举，用于中断判断 */
typedef enum {
    EXTI_TRIGGER_FALLING_EDGE = 0,
    EXTI_TRIGGER_RISING_EDGE,
    EXTI_TRIGGER_BOTH_EDGE,
} exti_trigger_mode_t;

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void exti_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_EXTI_PLATFORM_STM32F1
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
#elif DRV_EXTI_PLATFORM_STM32F4
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
#elif DRV_EXTI_PLATFORM_GD32F1
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
 * @param[in] cfg exti_cfg_t 结构体指针
 */
static void exti_hw_gpio_init(const exti_cfg_t *cfg)
{
#if DRV_EXTI_PLATFORM_STM32F1
    GPIO_InitTypeDef GPIO_InitStructure;
	switch ((uint32_t)cfg->trigger) {
	case (uint32_t)EXTI_Trigger_Falling:
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		break;
	case (uint32_t)EXTI_Trigger_Rising:
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
		break;
	case (uint32_t)EXTI_Trigger_Rising_Falling:
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		break;
	default: break;
	}
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->pin;
	GPIO_Init(cfg->port, &GPIO_InitStructure);

#elif DRV_EXTI_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
	switch ((uint32_t)cfg->trigger) {
	case (uint32_t)EXTI_Trigger_Falling:
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		break;
	case (uint32_t)EXTI_Trigger_Rising:
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		break;
	case (uint32_t)EXTI_Trigger_Rising_Falling:
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		break;
	default:
		break;
	}
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->pin;
	GPIO_Init(cfg->port, &GPIO_InitStructure);

#elif DRV_EXTI_PLATFORM_GD32F1
	switch ((uint32_t)cfg->trigger) {
	case (uint32_t)EXTI_TRIG_FALLING:
		gpio_init(cfg->port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, cfg->pin);
		break;
	case (uint32_t)EXTI_TRIG_RISING:
		gpio_init(cfg->port, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, cfg->pin);
		break;
	case (uint32_t)EXTI_TRIG_BOTH:
		gpio_init(cfg->port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, cfg->pin);
		break;
	default: break;
	}
#endif
}

/**
 * @brief	获取 GPIO 端口源
 * @param[in] port GPIO 端口
 * @return	引脚源编号
 */
static inline uint8_t exti_hw_get_gpio_port_source(gpio_port_t port)
{
#if DRV_EXTI_PLATFORM_STM32F1
	switch ((uint32_t)port) {
		case (uint32_t)GPIOA: return GPIO_PortSourceGPIOA;
		case (uint32_t)GPIOB: return GPIO_PortSourceGPIOB;
		case (uint32_t)GPIOC: return GPIO_PortSourceGPIOC;
		case (uint32_t)GPIOD: return GPIO_PortSourceGPIOD;
		case (uint32_t)GPIOE: return GPIO_PortSourceGPIOE;
		case (uint32_t)GPIOF: return GPIO_PortSourceGPIOF;
		case (uint32_t)GPIOG: return GPIO_PortSourceGPIOG;
		default: return 0xFF;
	}
#elif DRV_EXTI_PLATFORM_STM32F4
	switch ((uint32_t)port) {
		case (uint32_t)GPIOA: return EXTI_PortSourceGPIOA;
		case (uint32_t)GPIOB: return EXTI_PortSourceGPIOB;
		case (uint32_t)GPIOC: return EXTI_PortSourceGPIOC;
		case (uint32_t)GPIOD: return EXTI_PortSourceGPIOD;
		case (uint32_t)GPIOE: return EXTI_PortSourceGPIOE;
		case (uint32_t)GPIOF: return EXTI_PortSourceGPIOF;
		case (uint32_t)GPIOG: return EXTI_PortSourceGPIOG;
		default: return 0xFF;
	}
#elif DRV_EXTI_PLATFORM_GD32F1
	switch ((uint32_t)port) {
		case (uint32_t)GPIOA: return GPIO_PORT_SOURCE_GPIOA;
		case (uint32_t)GPIOB: return GPIO_PORT_SOURCE_GPIOB;
		case (uint32_t)GPIOC: return GPIO_PORT_SOURCE_GPIOC;
		case (uint32_t)GPIOD: return GPIO_PORT_SOURCE_GPIOD;
		case (uint32_t)GPIOE: return GPIO_PORT_SOURCE_GPIOE;
		case (uint32_t)GPIOF: return GPIO_PORT_SOURCE_GPIOF;
		case (uint32_t)GPIOG: return GPIO_PORT_SOURCE_GPIOG;
		default: return 0xFF;
	}
#endif
}

/**
 * @brief	获取 GPIO 引脚源
 * @param[in] pin GPIO 引脚
 * @return	引脚源编号
 */
static inline uint8_t exti_hw_get_gpio_pin_source(gpio_pin_t pin)
{
#if DRV_EXTI_PLATFORM_STM32F1 || DRV_EXTI_PLATFORM_STM32F4 || DRV_EXTI_PLATFORM_GD32F1
    for (uint8_t i = 0; i < 16; i++)
        if (pin & (1 << i))
			return i;
    return 0xFF;
#endif
}

/**
 * @brief   获取触发模式
 * @param[in] trigger 不同平台的触发模式枚举
 * @return	exti_trigger_mode_t 枚举
 */
static exti_trigger_mode_t exti_hw_convert_trigger(uint32_t trigger)
{
#if DRV_EXTI_PLATFORM_STM32F1 || DRV_EXTI_PLATFORM_STM32F4
    switch (trigger) {
    case (uint32_t)EXTI_Trigger_Falling: 		return EXTI_TRIGGER_FALLING_EDGE;
    case (uint32_t)EXTI_Trigger_Rising:  		return EXTI_TRIGGER_RISING_EDGE;
    case (uint32_t)EXTI_Trigger_Rising_Falling: return EXTI_TRIGGER_BOTH_EDGE;
    default: return EXTI_TRIGGER_FALLING_EDGE;
    }
#elif DRV_EXTI_PLATFORM_GD32F1
	switch (trigger) {
    case (uint32_t)EXTI_TRIG_FALLING: return EXTI_TRIGGER_FALLING_EDGE;
    case (uint32_t)EXTI_TRIG_RISING:  return EXTI_TRIGGER_RISING_EDGE;
    case (uint32_t)EXTI_TRIG_BOTH:    return EXTI_TRIGGER_BOTH_EDGE;
    default: 
	return EXTI_TRIGGER_FALLING_EDGE;
    }
#endif
}

/**
 * @brief	初始化 EXTI 外设
 * @param[in] cfg exti_cfg_t 结构体指针
 */
static void exti_hw_exti_init(const exti_cfg_t *cfg)
{
	const exti_hw_info_t *hw_info = exti_get_hw_info(cfg->pin);

#if DRV_EXTI_PLATFORM_STM32F1 || DRV_EXTI_PLATFORM_STM32F4
#if DRV_EXTI_PLATFORM_STM32F1
	/* 配置 AFIO */
	GPIO_EXTILineConfig(exti_hw_get_gpio_port_source(cfg->port), 
						exti_hw_get_gpio_pin_source(cfg->pin));
#elif DRV_EXTI_PLATFORM_STM32F4
	/* 配置 SYSCFG */
	SYSCFG_EXTILineConfig(exti_hw_get_gpio_port_source(cfg->port), 
						  exti_hw_get_gpio_pin_source(cfg->pin));
#endif
	/* 配置 EXTI */
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = hw_info->line;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = cfg->trigger;
	EXTI_Init(&EXTI_InitStructure);
	
	/* 配置中断 */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = hw_info->iqrn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = cfg->pre_priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = cfg->sub_priority;
	NVIC_Init(&NVIC_InitStructure);

#elif DRV_EXTI_PLATFORM_GD32F1
	/* 配置 AFIO */
	gpio_exti_source_select(exti_hw_get_gpio_port_source(cfg->port), 
							exti_hw_get_gpio_pin_source(cfg->pin));

	/* 配置 EXTI */
	exti_init(hw_info->line, EXTI_INTERRUPT, cfg->trigger);
	exti_interrupt_flag_clear(hw_info->line);
	
	/* 配置中断 */
	nvic_irq_enable(hw_info->iqrn, cfg->pre_priority, cfg->sub_priority);
#endif
}

/**
 * @brief	GPIO 读输入引脚电平
 * @param[in] port 端口
 * @param[in] pin  引脚
 * @return	电平
 */
static inline uint8_t exti_hw_gpio_read_in_bit(gpio_port_t port, gpio_pin_t pin)
{
#if DRV_EXTI_PLATFORM_STM32F1 || DRV_EXTI_PLATFORM_STM32F4
	return GPIO_ReadInputDataBit(port, pin);
#elif DRV_EXTI_PLATFORM_GD32F1
	return (uint8_t)gpio_input_bit_get(port, pin);
#endif
}

/**
 * @brief	检查 EXTI 中断标志
 * @param[in] hw_info exti_hw_info_t 结构体指针
 * @return	true 表示中断触发，false 表示未触发
 */
static inline bool exti_hw_get_it_flag(const exti_hw_info_t *hw_info)
{
#if DRV_EXTI_PLATFORM_STM32F1 || DRV_EXTI_PLATFORM_STM32F4
	return (bool)EXTI_GetITStatus(hw_info->line);
#elif DRV_EXTI_PLATFORM_GD32F1
	return (bool)exti_interrupt_flag_get(hw_info->line);
#endif
}

/**
 * @brief	清除 EXTI 中断标志
 * @param[in] hw_info exti_hw_info_t 结构体指针
 */
static inline void exti_hw_clear_it_flag(const exti_hw_info_t *hw_info)
{
#if DRV_EXTI_PLATFORM_STM32F1 || DRV_EXTI_PLATFORM_STM32F4
	EXTI_ClearITPendingBit(hw_info->line);
#elif DRV_EXTI_PLATFORM_GD32F1
	exti_interrupt_flag_clear(hw_info->line);
#endif
}

/**
 * @brief   初始化 EXTI 硬件
 * @param[in] cfg exti_cfg_t 结构体指针
 */
static void exti_hw_init(const exti_cfg_t *cfg)
{
#if DRV_EXTI_PLATFORM_STM32F1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	// 开启 AFIO 时钟
#elif DRV_EXTI_PLATFORM_STM32F4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	// 开启 SYSCFG 时钟
#elif DRV_EXTI_PLATFORM_GD32F1
	rcu_periph_clock_enable(RCU_AF);	// 开启 AFIO 时钟
#endif
	exti_hw_gpio_clock_enable(cfg->port);

	exti_hw_gpio_init(cfg);
	exti_hw_exti_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

/* 私有数据结构体 */
typedef struct {
	exti_dev_t *dev;
	bool	    in_use;
} exti_priv_t;

static exti_priv_t g_exti_priv[MAX_EXTI_NUM];

static exti_priv_t *exti_priv_alloc(gpio_pin_t pin);
static void exti_priv_free(exti_priv_t *priv);
static int exti_register_falling_irq_callback_impl(exti_dev_t *dev, 
												   exti_irq_callback_t callback, 
												   void *param);
static int exti_register_rising_irq_callback_impl(exti_dev_t *dev, 
												  exti_irq_callback_t callback, 
												  void *param);
static int exti_deinit_impl(exti_dev_t *dev);

/* 操作接口表 */
static const exti_ops_t exti_ops = {
	.register_falling_irq_callback = exti_register_falling_irq_callback_impl, 
	.register_rising_irq_callback  = exti_register_rising_irq_callback_impl, 
	.deinit                        = exti_deinit_impl
};

/**
 * @brief   初始化 EXTI
 * @param[out] dev exti_dev_t 结构体指针
 * @param[in]  cfg exti_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */		
int drv_exti_init(exti_dev_t *dev, const exti_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

	exti_priv_t *priv = exti_priv_alloc(cfg->pin);
	if (!priv)
		return -ENOMEM;
	
	priv->dev = dev;
	dev->priv = priv;

	dev->cfg = *cfg;
	dev->ops = &exti_ops;
	dev->falling_irq_callback 	    = NULL;
	dev->falling_irq_callback_param = NULL;
    dev->rising_irq_callback 	    = NULL;
	dev->rising_irq_callback_param  = NULL;

	exti_hw_init(cfg);

	return 0;
}

/**
 * @brief   根据索引从私有数据数组中分配一个空闲槽位
 * @param[in] pin 引脚
 * @return	成功返回槽位指针，失败返回 NULL
 */
static exti_priv_t *exti_priv_alloc(gpio_pin_t pin)
{
	const exti_hw_info_t *hw_info = exti_get_hw_info(pin);
    if (!hw_info)
		return NULL;
    
    uint8_t idx = hw_info->idx;
    if (g_exti_priv[idx].in_use)
		return NULL;
    
    g_exti_priv[idx].in_use = true;
    return &g_exti_priv[idx];
}

/**
 * @brief   释放私有数据槽位
 * @param[in,out] priv 待释放的槽位 exti_priv_t 结构体指针
 */
static void exti_priv_free(exti_priv_t *priv)
{
	if (priv)
		priv->in_use = false;
}

/**
 * @brief   EXTI 注册下降沿外部中断回调函数
 * @param[in] dev      exti_dev_t 结构体指针
 * @param[in] callback 下降沿外部中断回调函数指针
 * @param[in] param    下降沿外部中断回调函数参数
 * @return	0 表示成功，其他值表示失败
 */	
static int exti_register_falling_irq_callback_impl(exti_dev_t *dev, 
												   exti_irq_callback_t callback, 
												   void *param)
{
	if (!dev)
		return -EINVAL;

	dev->falling_irq_callback 	    = callback;
	dev->falling_irq_callback_param = param;
	return 0;
}

/**
 * @brief   EXTI 注册上升沿外部中断回调函数
 * @param[in] dev      exti_dev_t 结构体指针
 * @param[in] callback 上升沿外部中断回调函数指针
 * @param[in] param    上升沿外部中断回调函数参数
 * @return	0 表示成功，其他值表示失败
 */	
static int exti_register_rising_irq_callback_impl(exti_dev_t *dev, 
												  exti_irq_callback_t callback, 
												  void *param)
{
	if (!dev)
		return -EINVAL;

	dev->rising_irq_callback 	   = callback;
	dev->rising_irq_callback_param = param;
	return 0;
}

/**
 * @brief   去初始化 EXTI
 * @param[in,out] dev exti_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
static int exti_deinit_impl(exti_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	exti_priv_t *priv = (exti_priv_t *)dev->priv;
	exti_priv_free(priv);
	dev->priv = NULL;
	dev->ops = NULL;
	return 0;
}

/**
 * @brief   通用中断处理函数，内部使用
 * @param[in] pin 引脚
 */	
static void exti_irq_handler(gpio_pin_t pin)
{
	const exti_hw_info_t *hw_info = exti_get_hw_info(pin);
	exti_priv_t *priv = &g_exti_priv[hw_info->idx];
	exti_dev_t *dev = priv->dev;
	uint8_t level;

	if (exti_hw_get_it_flag(hw_info)) {
		exti_hw_clear_it_flag(hw_info);
		level = exti_hw_gpio_read_in_bit(dev->cfg.port, dev->cfg.pin);

		switch (exti_hw_convert_trigger(dev->cfg.trigger)) {
		case EXTI_TRIGGER_FALLING_EDGE:
			if (level == GPIO_LEVEL_LOW && dev->falling_irq_callback)
				dev->falling_irq_callback(dev->falling_irq_callback_param);
			break;
		case EXTI_TRIGGER_RISING_EDGE:
			if (level == GPIO_LEVEL_HIGH && dev->rising_irq_callback)
				dev->rising_irq_callback(dev->rising_irq_callback_param);
			break;
		case EXTI_TRIGGER_BOTH_EDGE:
			if (level == GPIO_LEVEL_LOW) {
				if (dev->falling_irq_callback)
					dev->falling_irq_callback(dev->falling_irq_callback_param);
			} else {
				if (dev->rising_irq_callback)
					dev->rising_irq_callback(dev->rising_irq_callback_param);
            }
			break;
		default:
			break;
		}
	}
}

#if DRV_EXTI_PLATFORM_STM32F1 || DRV_EXTI_PLATFORM_STM32F4
void EXTI0_IRQHandler(void) { exti_irq_handler(GPIO_Pin_0); }
void EXTI1_IRQHandler(void) { exti_irq_handler(GPIO_Pin_1); }
void EXTI2_IRQHandler(void) { exti_irq_handler(GPIO_Pin_2); }
void EXTI3_IRQHandler(void) { exti_irq_handler(GPIO_Pin_3); }
void EXTI4_IRQHandler(void) { exti_irq_handler(GPIO_Pin_4); }
void EXTI9_5_IRQHandler(void)
{
	for (uint8_t i = 5; i <= 9; i++) {
        gpio_pin_t pin = (1U << i);
        exti_irq_handler(pin);
    }
}
void EXTI15_10_IRQHandler(void)
{
	for (int i = 10; i <= 15; i++) {
        gpio_pin_t pin = (1U << i);
        exti_irq_handler(pin);
    }
}

#elif DRV_EXTI_PLATFORM_GD32F1
void EXTI0_IRQHandler(void) { exti_irq_handler(GPIO_PIN_0); }
void EXTI1_IRQHandler(void) { exti_irq_handler(GPIO_PIN_1); }
void EXTI2_IRQHandler(void) { exti_irq_handler(GPIO_PIN_2); }
void EXTI3_IRQHandler(void) { exti_irq_handler(GPIO_PIN_3); }
void EXTI4_IRQHandler(void) { exti_irq_handler(GPIO_PIN_4); }
void EXTI5_9_IRQHandler(void)
{
	for (uint8_t i = 5; i <= 9; i++) {
        gpio_pin_t pin = (1U << i);
        exti_irq_handler(pin);
    }
}
void EXTI10_15_IRQHandler(void)
{
	for (int i = 10; i <= 15; i++) {
        gpio_pin_t pin = (1U << i);
        exti_irq_handler(pin);
    }
}
#endif

/* ------------------------------- 核心驱动层结束 ------------------------------- */
