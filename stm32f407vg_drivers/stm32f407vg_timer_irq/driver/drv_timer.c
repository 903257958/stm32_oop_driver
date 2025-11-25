#include "drv_timer.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */
#if DRV_TIMER_PLATFORM_STM32F1
#define TIMER_FREQ	72000000

#elif DRV_TIMER_PLATFORM_STM32F4
#if defined(STM32F40_41xxx)
#define TIMER_FREQ	84000000
#elif defined(STM32F429_439xx)
#define TIMER_FREQ	90000000
#endif

#elif DRV_TIMER_PLATFORM_GD32F1
#define TIMER_FREQ	108000000
#endif

/* 硬件信息结构体 */
typedef struct {
	timer_periph_t timer_periph;
	iqrn_type_t iqrn;
	uint8_t idx;
} timer_hw_info_t;

/* 硬件信息列表 */
static const timer_hw_info_t timer_hw_info_table[] = {
#if DRV_TIMER_PLATFORM_STM32F1
	{ TIM2, TIM2_IRQn, 0 },
	{ TIM3, TIM3_IRQn, 1 },
	{ TIM4, TIM4_IRQn, 2 },
#if defined(STM32F10X_HD)
	{ TIM5, TIM5_IRQn, 3 },
	{ TIM6, TIM6_IRQn, 4 },
	{ TIM7, TIM7_IRQn, 5 },
#endif
#endif	/* DRV_TIMER_PLATFORM_STM32F1 */

#if DRV_TIMER_PLATFORM_STM32F4
	{ TIM2, TIM2_IRQn,     0 },
	{ TIM3, TIM3_IRQn,     1 },
	{ TIM4, TIM4_IRQn,     2 },
	{ TIM5, TIM5_IRQn,     3 },
	{ TIM6, TIM6_DAC_IRQn, 4 },
	{ TIM7, TIM7_IRQn,     5 },
#endif	/* DRV_TIMER_PLATFORM_STM32F4 */

#if DRV_TIMER_PLATFORM_GD32F1
	{ TIMER1, TIMER1_IRQn, 0 },
	{ TIMER2, TIMER2_IRQn, 1 },
	{ TIMER3, TIMER3_IRQn, 2 },
#endif	/* DRV_TIMER_PLATFORM_GD32F1 */
};

#define MAX_TIMER_NUM	(sizeof(timer_hw_info_table) / sizeof(timer_hw_info_t))

/**
 * @brief	获取硬件信息
 * @param[in] timer_periph 定时器外设
 * @return	成功返回对应硬件信息指针，失败返回 NULL
 */
static inline const timer_hw_info_t* timer_get_hw_info(timer_periph_t timer_periph)
{
    for (uint8_t i = 0; i < MAX_TIMER_NUM; i++) {
        if (timer_hw_info_table[i].timer_periph == timer_periph) {
            return &timer_hw_info_table[i];
        }
    }
    return NULL;
}

/**
 * @brief	使能定时器外设时钟
 * @param[in] timer_periph 定时器外设
 */
static void timer_hw_timer_clock_enable(timer_periph_t timer_periph)
{
#if DRV_TIMER_PLATFORM_STM32F1 || DRV_TIMER_PLATFORM_STM32F4
	switch ((uint32_t)timer_periph) {
	case (uint32_t)TIM2: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); break;
	case (uint32_t)TIM3: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); break;
	case (uint32_t)TIM4: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); break;
	case (uint32_t)TIM5: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); break;
	case (uint32_t)TIM6: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); break;
	case (uint32_t)TIM7: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); break;
	default: break;
    }
#elif DRV_TIMER_PLATFORM_GD32F1
	switch ((uint32_t)timer_periph) {
	case (uint32_t)TIMER1: rcu_periph_clock_enable(RCU_TIMER1); break;
	case (uint32_t)TIMER2: rcu_periph_clock_enable(RCU_TIMER2); break;
	case (uint32_t)TIMER3: rcu_periph_clock_enable(RCU_TIMER3); break;
	default: break;
    }
#endif
}

/**
 * @brief	初始化定时器外设
 * @param[in] cfg timer_cfg_t 结构体指针
 */
static void timer_hw_timer_init(const timer_cfg_t *cfg)
{
	const timer_hw_info_t *hw_info = timer_get_hw_info(cfg->timer_periph);

#if DRV_TIMER_PLATFORM_STM32F1 || DRV_TIMER_PLATFORM_STM32F4
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
	
	/* 配置中断 */
    if (cfg->use_irq) {
        TIM_ClearFlag(cfg->timer_periph, TIM_FLAG_Update);		// 清除定时器更新标志位
        TIM_ITConfig(cfg->timer_periph, TIM_IT_Update, ENABLE);	// 更新中断
            
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = hw_info->iqrn;							// 中断通道
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								// 中断通道使能
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = cfg->pre_priority;   // 抢占优先级
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = cfg->sub_priority;			// 响应优先级
        NVIC_Init(&NVIC_InitStructure);
    }
	
	/* 启用定时器 */
	TIM_Cmd(cfg->timer_periph, ENABLE);

#elif DRV_TIMER_PLATFORM_GD32F1
	/* 配置时钟源为内部时钟 */
	timer_internal_clock_config(cfg->timer_periph);
	
	/* 时基单元初始化 */
	timer_parameter_struct timer_initpara;
	timer_initpara.clockdivision = TIMER_CKDIV_DIV1;	// 时钟分频参数，不分频
	timer_initpara.alignedmode = TIMER_COUNTER_EDGE;	// 对齐模式
	timer_initpara.counterdirection = TIMER_COUNTER_UP;	// 计数器模式：向上计数
	timer_initpara.prescaler = cfg->psc;				// PSC预分频器的值，范围0~65535
	timer_initpara.period = cfg->arr;					// ARR自动重装器的值，范围0~65535
	timer_initpara.repetitioncounter = 0;				// 重复计数器的值（高级定时器用）
	timer_init(cfg->timer_periph, &timer_initpara);
	
	/* 配置中断 */
    if (cfg->use_irq) {
        timer_interrupt_flag_clear(cfg->timer_periph, TIMER_INT_FLAG_UP);	// 清除定时器更新标志位
        timer_interrupt_enable(cfg->timer_periph, TIMER_INT_UP);			// 配置中断源，开启更新中断
        nvic_irq_enable(hw_info->iqrn, cfg->pre_priority, cfg->sub_priority);
    }
	
	/* 启用定时器 */
	timer_enable(cfg->timer_periph);
#endif
}

/**
 * @brief	获取定时器计数值
 * @param[in] timer_periph 定时器外设
 * @return	计数值
 */
static inline uint16_t timer_hw_get_counter(timer_periph_t timer_periph)
{
#if DRV_TIMER_PLATFORM_STM32F1 || DRV_TIMER_PLATFORM_STM32F4
	return TIM_GetCounter(timer_periph);

#elif DRV_TIMER_PLATFORM_GD32F1
	return timer_counter_read(timer_periph);
#endif
}

#if TIMER1_IRQ_HANDLER_ENABLE || TIMER2_IRQ_HANDLER_ENABLE || \
    TIMER3_IRQ_HANDLER_ENABLE || TIMER4_IRQ_HANDLER_ENABLE || \
    TIMER5_IRQ_HANDLER_ENABLE || TIMER6_IRQ_HANDLER_ENABLE || \
    TIMER7_IRQ_HANDLER_ENABLE
/**
 * @brief	检查定时器中断标志
 * @param[in] timer_periph 定时器外设
 * @return	true 表示中断触发，false 表示未触发
 */
static inline bool timer_hw_get_it_flag(timer_periph_t timer_periph)
{
#if DRV_TIMER_PLATFORM_STM32F1 || DRV_TIMER_PLATFORM_STM32F4
	return (bool)TIM_GetITStatus(timer_periph, TIM_IT_Update);

#elif DRV_TIMER_PLATFORM_GD32F1
	return (bool)timer_interrupt_flag_get(timer_periph, TIMER_INT_FLAG_UP);
#endif
}

/**
 * @brief	清除定时器中断标志
 * @param[in] timer_periph 定时器外设
 */
static inline void timer_hw_clear_it_flag(timer_periph_t timer_periph)
{
#if DRV_TIMER_PLATFORM_STM32F1 || DRV_TIMER_PLATFORM_STM32F4
	TIM_ClearITPendingBit(timer_periph, TIM_IT_Update);

#elif DRV_TIMER_PLATFORM_GD32F1
	timer_interrupt_flag_clear(timer_periph, TIMER_INT_FLAG_UP);
#endif
}
#endif

/**
 * @brief   初始化定时器硬件
 * @param[in] cfg timer_cfg_t 结构体指针
 */
static void timer_hw_init(const timer_cfg_t *cfg)
{
	timer_hw_timer_clock_enable(cfg->timer_periph);
	timer_hw_timer_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

/* 私有数据结构体 */
typedef struct {
	timer_dev_t *dev;
	bool		 in_use;
} timer_priv_t;

static timer_priv_t g_timer_priv[MAX_TIMER_NUM];

static timer_priv_t *timer_priv_alloc(timer_periph_t timer_periph);
static void timer_priv_free(timer_priv_t *priv);
static int timer_delay_us_impl(timer_dev_t *dev, uint32_t us);
static int timer_delay_ms_impl(timer_dev_t *dev, uint32_t ms);
static int timer_register_irq_callback_impl(timer_dev_t *dev, 
											timer_irq_callback_t callback, 
											void *param);
static int timer_deinit_impl(timer_dev_t *dev);

/* 操作接口表 */
static const timer_ops_t timer_ops = {
	.delay_us 			   = timer_delay_us_impl,
	.delay_ms 			   = timer_delay_ms_impl,
	.register_irq_callback = timer_register_irq_callback_impl, 
	.deinit   			   = timer_deinit_impl
};

/**
 * @brief   初始化定时器设备驱动（不包括高级定时器）
 * @details 以 STM32F1 为例，定时器时钟为 72 MHz，计数频率由分频系数 PSC 决定：
 *          计数频率 = 72 MHz / (PSC + 1)，其倒数为计数周期。
 *          - 当用于微秒级延时时，需配置 PSC = (TIMER_FREQ / 1000000 - 1)，
 *            此时计数周期为 1 µs
 *          - 当用于毫秒级延时时，需配置 ARR >= 1000，
 *            定时周期 = (ARR + 1) µs，最大约 65.5 ms。
 *          - 若 PSC 或 ARR 未正确配置，
 *            则该定时器仅可用于中断，无法提供延时功能。
 * @param[out] dev timer_dev_t 结构体指针
 * @param[in]  cfg timer_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
int drv_timer_init(timer_dev_t *dev, const timer_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

	timer_priv_t *priv = timer_priv_alloc(cfg->timer_periph);
	if (!priv)
		return -ENOMEM;
	
	priv->dev 				= dev;
	dev->priv 				= priv;
    dev->cfg                = *cfg;
	dev->ops  				= &timer_ops;
	dev->irq_callback 		= NULL;
	dev->irq_callback_param = NULL;
    
	timer_hw_init(cfg);
	
	return 0;
}

/**
 * @brief   根据索引从私有数据数组中分配一个空闲槽位
 * @param[in] timer_periph 定时器外设
 * @return	成功返回槽位指针，失败返回 NULL
 */
static timer_priv_t *timer_priv_alloc(timer_periph_t timer_periph)
{
	const timer_hw_info_t *hw_info = timer_get_hw_info(timer_periph);
    if (!hw_info)
		return NULL;
    
    uint8_t idx = hw_info->idx;
    if (g_timer_priv[idx].in_use)
		return NULL;
    
    g_timer_priv[idx].in_use = true;
    return &g_timer_priv[idx];
}

/**
 * @brief   释放私有数据槽位
 * @param[in,out] priv 待释放的槽位 timer_priv_t 结构体指针
 */
static void timer_priv_free(timer_priv_t *priv)
{
	if (priv)
		priv->in_use = false;
}

/**
 * @brief   定时器实现微秒级延时，需要正确配置 PSC 寄存器
 * @param[in] dev timer_dev_t 结构体指针
 * @param[in] us  要延时的微秒数
 * @return	0 表示成功，其他值表示失败
 */	
static int timer_delay_us_impl(timer_dev_t *dev, uint32_t us)
{
	if (!dev)
        return -EINVAL;
    
    if (us == 0)
        return 0;
    
	if (dev->cfg.psc != (TIMER_FREQ / 1000000 - 1))	// 检查 PSC 配置
		return -EINVAL;

	uint32_t start = timer_hw_get_counter(dev->cfg.timer_periph);	// 起始计数值
	uint32_t period = dev->cfg.arr + 1;								// 最大定时周期
	uint32_t elapsed;

	do {
		uint32_t current = timer_hw_get_counter(dev->cfg.timer_periph);					// 当前计数值
		elapsed = (current >= start) ? (current - start) : (period - start + current);	// 防溢出
	} while (elapsed < us);

	return 0;
}

/**
 * @brief   定时器实现毫秒级延时，需要正确配置 PSC 与 ARR 寄存器
 * @param[in] dev timer_dev_t 结构体指针
 * @param[in] us  要延时的毫秒数
 * @return	0 表示成功，其他值表示失败
 */	
static int timer_delay_ms_impl(timer_dev_t *dev, uint32_t ms)
{
	if (!dev)
        return -EINVAL;

	if (ms == 0)
        return 0;
    
    if (dev->cfg.arr < 1000)	// 检查 ARR 配置
		return -EINVAL;

	int ret;

	while (ms--) {
		ret = timer_delay_us_impl(dev, 1000);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/**
 * @brief   定时器注册中断回调函数
 * @param[in] dev      timer_dev_t 结构体指针
 * @param[in] callback 中断回调函数指针
 * @param[in] param    中断回调函数参数
 * @return	0 表示成功，其他值表示失败
 */	
static int timer_register_irq_callback_impl(timer_dev_t *dev, 
											timer_irq_callback_t callback, 
											void *param)
{
	if (!dev)
		return -EINVAL;

	dev->irq_callback 		= callback;
	dev->irq_callback_param = param;
	return 0;
}

/**
 * @brief   去初始化定时器
 * @param[in,out] dev timer_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
static int timer_deinit_impl(timer_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	timer_priv_t *priv = (timer_priv_t *)dev->priv;
	timer_priv_free(priv);
	dev->priv = NULL;
	dev->ops = NULL;
	return 0;
}

#if TIMER1_IRQ_HANDLER_ENABLE || TIMER2_IRQ_HANDLER_ENABLE || \
    TIMER3_IRQ_HANDLER_ENABLE || TIMER4_IRQ_HANDLER_ENABLE || \
    TIMER5_IRQ_HANDLER_ENABLE || TIMER6_IRQ_HANDLER_ENABLE || \
    TIMER7_IRQ_HANDLER_ENABLE
/**
 * @brief   定时器通用中断函数，内部使用
 * @param[in] timer_periph 定时器外设
 */	
static void timer_irq_handler(timer_periph_t timer_periph)
{
	const timer_hw_info_t *hw_info = timer_get_hw_info(timer_periph);
	timer_priv_t *priv = &g_timer_priv[hw_info->idx];
	timer_dev_t *dev = priv->dev;

    if (timer_hw_get_it_flag(timer_periph)) {
        timer_hw_clear_it_flag(timer_periph);
		if (dev->irq_callback)
			dev->irq_callback(dev->irq_callback_param);
    }
}
#endif

/* 各中断服务函数 */
#if TIMER1_IRQ_HANDLER_ENABLE
#if DRV_TIMER_PLATFORM_GD32F1
void TIMER1_IRQHandler(void) { timer_irq_handler(TIMER1); }
#endif
#endif

#if TIMER2_IRQ_HANDLER_ENABLE
#if DRV_TIMER_PLATFORM_STM32F1 || DRV_TIMER_PLATFORM_STM32F4
void TIM2_IRQHandler(void) { timer_irq_handler(TIM2); }
#elif DRV_TIMER_PLATFORM_GD32F1
void TIMER2_IRQHandler(void) { timer_irq_handler(TIMER2); }
#endif
#endif

#if TIMER3_IRQ_HANDLER_ENABLE
#if DRV_TIMER_PLATFORM_STM32F1 || DRV_TIMER_PLATFORM_STM32F4
void TIM3_IRQHandler(void) { timer_irq_handler(TIM3); }
#elif DRV_TIMER_PLATFORM_GD32F1
void TIMER3_IRQHandler(void) { timer_irq_handler(TIMER3); }
#endif
#endif

#if TIMER4_IRQ_HANDLER_ENABLE
#if DRV_TIMER_PLATFORM_STM32F1 || DRV_TIMER_PLATFORM_STM32F4
void TIM4_IRQHandler(void) { timer_irq_handler(TIM4); }
#endif
#endif

#if TIMER5_IRQ_HANDLER_ENABLE
#if defined(STM32F10X_HD) || DRV_TIMER_PLATFORM_STM32F4
void TIM5_IRQHandler(void) { timer_irq_handler(TIM5); }
#endif
#endif

#if TIMER6_IRQ_HANDLER_ENABLE
#if defined(STM32F10X_HD)
void TIM6_IRQHandler(void) { timer_irq_handler(TIM6); }
#elif DRV_TIMER_PLATFORM_STM32F4
void TIM6_DAC_IRQHandler(void) { timer_irq_handler(TIM6); }
#endif
#endif

#if TIMER7_IRQ_HANDLER_ENABLE
#if defined(STM32F10X_HD) || DRV_TIMER_PLATFORM_STM32F4
void TIM7_IRQHandler(void) { timer_irq_handler(TIM7); }
#endif
#endif

/* ------------------------------- 核心驱动层结束 ------------------------------- */
