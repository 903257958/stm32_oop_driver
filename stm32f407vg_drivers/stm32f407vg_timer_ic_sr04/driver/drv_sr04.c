#include "drv_sr04.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */
#define GPIO_LEVEL_HIGH 1
#define GPIO_LEVEL_LOW 	0

#if DRV_SR04_PLATFORM_STM32F1
#define TIMER_FREQ	72000000

#elif DRV_SR04_PLATFORM_STM32F4
#if defined(STM32F40_41xxx)
#define TIMER_FREQ	84000000
#elif defined(STM32F429_439xx)
#define TIMER_FREQ	90000000
#endif

#elif DRV_SR04_PLATFORM_GD32F1
#define TIMER_FREQ	108000000
#endif

/* 硬件信息结构体 */
typedef struct {
	timer_periph_t timer_periph;
	iqrn_type_t iqrn;
#if DRV_SR04_PLATFORM_STM32F4
	uint8_t af;
#endif
	uint8_t idx;
} sr04_hw_info_t;

/* 硬件信息列表 */
static const sr04_hw_info_t sr04_hw_info_table[] = {
#if DRV_SR04_PLATFORM_STM32F1
	{ TIM2, TIM2_IRQn, 0 },
	{ TIM3, TIM3_IRQn, 1 },
	{ TIM4, TIM4_IRQn, 2 },
#if defined(STM32F10X_HD)
	{ TIM5, TIM5_IRQn, 3 },
#endif
#endif	/* DRV_SR04_PLATFORM_STM32F1 */

#if DRV_SR04_PLATFORM_STM32F4
	{ TIM2, TIM2_IRQn, GPIO_AF_TIM2, 0 },
	{ TIM3, TIM3_IRQn, GPIO_AF_TIM3, 1 },
	{ TIM4, TIM4_IRQn, GPIO_AF_TIM4, 2 },
	{ TIM5, TIM5_IRQn, GPIO_AF_TIM5, 3 },
#endif	/* DRV_SR04_PLATFORM_STM32F4 */

#if DRV_SR04_PLATFORM_GD32F1
	{ TIMER1, TIMER1_IRQn, 0 },
	{ TIMER2, TIMER2_IRQn, 1 },
	{ TIMER3, TIMER3_IRQn, 2 },
#endif	/* DRV_SR04_PLATFORM_GD32F1 */
};

#define MAX_SR04_NUM	(sizeof(sr04_hw_info_table) / sizeof(sr04_hw_info_t))

/**
 * @brief	获取硬件信息
 * @param[in] timer_periph 定时器外设
 * @return	成功返回对应硬件信息指针，失败返回 NULL
 */
static inline const sr04_hw_info_t* sr04_get_hw_info(timer_periph_t timer_periph)
{
    for (uint8_t i = 0; i < MAX_SR04_NUM; i++) {
        if (sr04_hw_info_table[i].timer_periph == timer_periph) {
            return &sr04_hw_info_table[i];
        }
    }
    return NULL;
}

/**
 * @brief	使能定时器外设时钟
 * @param[in] timer_periph 定时器外设
 */
static void sr04_hw_timer_clock_enable(timer_periph_t timer_periph)
{
#if DRV_SR04_PLATFORM_STM32F1 || DRV_SR04_PLATFORM_STM32F4
	switch ((uint32_t)timer_periph) {
	case (uint32_t)TIM2: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); break;
	case (uint32_t)TIM3: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); break;
	case (uint32_t)TIM4: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); break;
	case (uint32_t)TIM5: RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); break;
	default: break;
    }
#elif DRV_SR04_PLATFORM_GD32F1
	switch ((uint32_t)timer_periph) {
	case (uint32_t)TIMER1: rcu_periph_clock_enable(RCU_TIMER1); break;
	case (uint32_t)TIMER2: rcu_periph_clock_enable(RCU_TIMER2); break;
	case (uint32_t)TIMER3: rcu_periph_clock_enable(RCU_TIMER3); break;
	default: break;
    }
#endif
}

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void sr04_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_SR04_PLATFORM_STM32F1
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
#elif DRV_SR04_PLATFORM_STM32F4
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
#elif DRV_SR04_PLATFORM_GD32F1
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

#if DRV_SR04_PLATFORM_STM32F4
/**
 * @brief	获取 GPIO 引脚源（STM32F4特有）
 * @param[in] pin GPIO 引脚
 * @return	引脚源编号（如9对应PinSource9）
 */
static inline uint8_t sr04_hw_get_gpio_pin_source(gpio_pin_t pin)
{
    for (uint8_t i = 0; i < 16; i++)
        if (pin & (1 << i))
			return i;  // GPIO_Pin = 1<<i，对应 PinSource = i
    return 0xFF;
}
#endif	/* DRV_SR04_PLATFORM_STM32F4 */

/**
 * @brief	初始化 GPIO
 * @param[in] cfg sr04_cfg_t 结构体指针
 */
static void sr04_hw_gpio_init(const sr04_cfg_t *cfg)
{
#if DRV_SR04_PLATFORM_STM32F1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = cfg->trig_pin;
	GPIO_Init(cfg->trig_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin   = cfg->echo_pin;
	GPIO_Init(cfg->echo_port, &GPIO_InitStructure);
#elif DRV_SR04_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin   = cfg->trig_pin;
	GPIO_Init(cfg->trig_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin   = cfg->echo_pin;
	GPIO_Init(cfg->echo_port, &GPIO_InitStructure);

	const sr04_hw_info_t *hw_info = sr04_get_hw_info(cfg->timer_periph);
	GPIO_PinAFConfig(cfg->echo_port, sr04_hw_get_gpio_pin_source(cfg->echo_pin), hw_info->af);
#elif DRV_SR04_PLATFORM_GD32F1
	gpio_init(cfg->trig_port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, cfg->trig_pin);
	gpio_init(cfg->echo_port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, cfg->echo_pin);
#endif
}

/**
 * @brief	初始化定时器外设
 * @param[in] cfg sr04_cfg_t 结构体指针
 */
static void sr04_hw_timer_init(const sr04_cfg_t *cfg)
{
	const sr04_hw_info_t *hw_info = sr04_get_hw_info(cfg->timer_periph);

#if DRV_SR04_PLATFORM_STM32F1 || DRV_SR04_PLATFORM_STM32F4
	/* 配置时钟源为内部时钟 */
	TIM_InternalClockConfig(cfg->timer_periph);
	
	/* 时基单元初始化 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			// 时钟分频参数，不分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		// 计数器模式：向上计数
	TIM_TimeBaseInitStructure.TIM_Prescaler = TIMER_FREQ / 1000000 - 1;	// PSC预分频器的值，计数频率1MHz，计数周期1us
	TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;						// ARR自动重装器的值，范围0~65535
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;				// 重复计数器的值（高级定时器用）
	TIM_TimeBaseInit(cfg->timer_periph, &TIM_TimeBaseInitStructure);
	
	/* 初始化输入捕获 */
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;							// 输入滤波器参数
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;		// 极性，选择为上升沿触发捕获
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			// 捕获预分频，选择不分频，每次信号都触发捕获
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	// 输入信号交叉，选择直通，不交叉
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	if (cfg->ic_channel == 1)
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	else if (cfg->ic_channel == 2)
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	else if (cfg->ic_channel == 3)
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	else if (cfg->ic_channel == 4)
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(cfg->timer_periph, &TIM_ICInitStructure);

	/* 配置中断 */
	TIM_ITConfig(cfg->timer_periph, TIM_IT_CC1 << (cfg->ic_channel - 1), ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = hw_info->iqrn;							// 中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								// 中断通道使能
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = cfg->pre_priority;	// 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = cfg->sub_priority;			// 响应优先级
    NVIC_Init(&NVIC_InitStructure);
	
	/* 启用定时器 */
	TIM_Cmd(cfg->timer_periph, ENABLE);

#elif DRV_SR04_PLATFORM_GD32F1
	/* 配置时钟源为内部时钟 */
	timer_internal_clock_config(cfg->timer_periph);

	/* 时基单元初始化 */
	timer_parameter_struct timer_initpara;
	timer_initpara.clockdivision = TIMER_CKDIV_DIV1;		// 时钟分频参数，不分频
	timer_initpara.alignedmode = TIMER_COUNTER_EDGE;		// 对齐模式
	timer_initpara.counterdirection = TIMER_COUNTER_UP;		// 计数器模式：向上计数
	timer_initpara.prescaler = TIMER_FREQ / 1000000 - 1;	// PSC预分频器的值，计数频率1MHz，计数周期1us
	timer_initpara.period = 0xFFFF;							// ARR自动重装器的值，范围0~65535
	timer_initpara.repetitioncounter = 0;					// 重复计数器的值（高级定时器用）
	timer_init(cfg->timer_periph, &timer_initpara);

    /* 初始化输入捕获 */
	timer_ic_parameter_struct timer_ic_parameter_icpara;
	timer_ic_parameter_icpara.icfilter = 0x0;								// 输入滤波器参数
	timer_ic_parameter_icpara.icpolarity = TIMER_IC_POLARITY_RISING;		// 极性：上升沿触发捕获
	timer_ic_parameter_icpara.icprescaler = TIMER_IC_PSC_DIV1;				// 捕获预分频：不分频，每次信号都触发捕获
	timer_ic_parameter_icpara.icselection = TIMER_IC_SELECTION_DIRECTTI;	// 输入信号交叉：直通，不交叉
	timer_input_capture_config(cfg->timer_periph, (uint16_t)cfg->ic_channel, &timer_ic_parameter_icpara);

	/* 配置中断 */
	timer_interrupt_flag_clear(cfg->timer_periph,    // 启用前清除捕获标志，防止假触发
							   TIMER_INT_FLAG_UP  | 
							   TIMER_INT_FLAG_CH0 | 
							   TIMER_INT_FLAG_CH1 | 
							   TIMER_INT_FLAG_CH2 | 
							   TIMER_INT_FLAG_CH3);
	timer_interrupt_enable(cfg->timer_periph, TIMER_INT_CH0 << cfg->ic_channel);
	nvic_irq_enable(hw_info->iqrn, cfg->pre_priority, cfg->sub_priority);
    
    /* 启用定时器 */
	timer_enable(cfg->timer_periph);
#endif
}

/**
 * @brief	写 GPIO 引脚电平
 * @param[in] port  端口
 * @param[in] pin   引脚
 * @param[in] level 电平
 */
static inline void sr04_hw_gpio_write_bit(gpio_port_t port, gpio_pin_t pin, uint8_t level)
{
#if DRV_SR04_PLATFORM_STM32F1 || DRV_SR04_PLATFORM_STM32F4
	GPIO_WriteBit(port, pin, (BitAction)level);
#elif DRV_SR04_PLATFORM_GD32F1
	gpio_bit_write(port, pin, (bit_status)(level));
#endif
}

/**
 * @brief	检查定时器中断标志
 * @param[in] timer_periph 定时器外设
 * @param[in] ic_channel   输入捕获通道
 * @return	true 表示中断触发，false 表示未触发
 */
static inline bool sr04_hw_get_it_flag(timer_periph_t timer_periph, uint8_t ic_channel)
{
#if DRV_SR04_PLATFORM_STM32F1 || DRV_SR04_PLATFORM_STM32F4
	return (bool)TIM_GetITStatus(timer_periph, TIM_IT_CC1 << (ic_channel - 1));

#elif DRV_SR04_PLATFORM_GD32F1
	return (bool)timer_interrupt_flag_get(timer_periph, TIMER_INT_FLAG_CH0 << ic_channel);
#endif
}

/**
 * @brief	清除定时器中断标志
 * @param[in] timer_periph 定时器外设
 * @param[in] ic_channel   输入捕获通道
 */
static inline void sr04_hw_clear_it_flag(timer_periph_t timer_periph, uint8_t ic_channel)
{
#if DRV_SR04_PLATFORM_STM32F1 || DRV_SR04_PLATFORM_STM32F4
	TIM_ClearITPendingBit(timer_periph, TIM_IT_CC1 << (ic_channel - 1));

#elif DRV_SR04_PLATFORM_GD32F1
	timer_interrupt_flag_clear(timer_periph, TIMER_INT_FLAG_CH0 << ic_channel);
#endif
}

/**
 * @brief	获取定时器输入捕获通道值
 * @param[in] timer_periph 定时器外设
 * @param[in] ic_channel   输入捕获通道
 * @return	输入捕获通道值
 */
static inline uint32_t sr04_hw_get_timer_ic_capture(timer_periph_t timer_periph, uint8_t ic_channel)
{
#if DRV_SR04_PLATFORM_STM32F1 || DRV_SR04_PLATFORM_STM32F4
	switch (ic_channel) {
	case 1: return TIM_GetCapture1(timer_periph);
	case 2: return TIM_GetCapture2(timer_periph);
	case 3: return TIM_GetCapture3(timer_periph);
	case 4: return TIM_GetCapture4(timer_periph);
	default: return 0xFFFF;
	}
#elif DRV_SR04_PLATFORM_GD32F1
	return timer_channel_capture_value_register_read(timer_periph, (uint16_t)ic_channel);
#endif
}

/**
 * @brief	配置定时器通道极性
 * @param[in] timer_periph 定时器外设
 * @param[in] ic_channel   输入捕获通道
 * @param[in] is_falling   是否为下次捕获下降沿
 */
static inline void sr04_hw_set_timer_ic_polarity(timer_periph_t timer_periph, uint8_t ic_channel, bool is_falling)
{
	uint16_t ocpolarity;
#if DRV_SR04_PLATFORM_STM32F1 || DRV_SR04_PLATFORM_STM32F4
	if (is_falling)
		ocpolarity = TIM_ICPolarity_Falling;
	else
		ocpolarity = TIM_ICPolarity_Rising;

	switch (ic_channel) {
	case 1: TIM_OC1PolarityConfig(timer_periph, ocpolarity); break;
	case 2: TIM_OC2PolarityConfig(timer_periph, ocpolarity); break;
	case 3: TIM_OC3PolarityConfig(timer_periph, ocpolarity); break;
	case 4: TIM_OC4PolarityConfig(timer_periph, ocpolarity); break;
	}
#elif DRV_SR04_PLATFORM_GD32F1
	if (is_falling)
		ocpolarity = TIMER_OC_POLARITY_LOW;
	else
		ocpolarity = TIMER_OC_POLARITY_HIGH;

	timer_channel_output_polarity_config(timer_periph, (uint16_t)ic_channel, ocpolarity);
#endif
}


/**
 * @brief   初始化 SR04 硬件
 * @param[in] cfg sr04_cfg_t 结构体指针
 */
static void sr04_hw_init(const sr04_cfg_t *cfg)
{
	sr04_hw_timer_clock_enable(cfg->timer_periph);
	sr04_hw_gpio_clock_enable(cfg->trig_port);
	sr04_hw_gpio_clock_enable(cfg->echo_port);

	sr04_hw_gpio_init(cfg);
	sr04_hw_timer_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

/* 私有数据结构体 */
typedef struct {
	sr04_dev_t 		 *dev;
	volatile uint16_t t1;		// 捕获的上升沿时间（Echo开始变高）
	volatile uint16_t t2;		// 捕获的下降沿时间（Echo结束变低）
	volatile bool 	  is_first;	// 标记当前是第一次捕获（上升沿）还是第二次捕获（下降沿）
	volatile bool 	  is_done;	// 捕获过程是否完成
	bool			  in_use;
} sr04_priv_t;

static sr04_priv_t g_sr04_priv[MAX_SR04_NUM];

static sr04_priv_t *sr04_priv_alloc(timer_periph_t timer_periph);
static void sr04_priv_free(sr04_priv_t *priv);
static void sr04_trigger(sr04_dev_t *dev);
static int sr04_get_distance_impl(sr04_dev_t *dev, float *distance_cm);
static int sr04_deinit_impl(sr04_dev_t *dev);
static void sr04_timer_irq_handler(timer_periph_t timer_periph);

/* 操作接口表 */
static const sr04_ops_t sr04_ops = {
	.get_distance = sr04_get_distance_impl, 
	.deinit   	  = sr04_deinit_impl
};

/**
 * @brief   初始化 SR04 设备驱动
 * @param[out] dev sr04_dev_t 结构体指针
 * @param[in]  cfg sr04_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_sr04_init(sr04_dev_t *dev, const sr04_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

	sr04_priv_t *priv = sr04_priv_alloc(cfg->timer_periph);
	if (!priv)
		return -ENOMEM;
	
	priv->dev 	   = dev;
	priv->t1 	   = 0;
	priv->t2 	   = 0;
	priv->is_first = true;
	priv->is_done  = false;
	
	dev->priv = priv;
	dev->cfg  = *cfg;
	dev->ops  = &sr04_ops;

	sr04_hw_init(cfg);
	return 0;
}

/**
 * @brief   根据索引从私有数据数组中分配一个空闲槽位
 * @param[in] timer_periph 定时器外设
 * @return	成功返回槽位指针，失败返回 NULL
 */
static sr04_priv_t *sr04_priv_alloc(timer_periph_t timer_periph)
{
	const sr04_hw_info_t *hw_info = sr04_get_hw_info(timer_periph);
    if (!hw_info)
		return NULL;
    
    uint8_t idx = hw_info->idx;
    if (g_sr04_priv[idx].in_use)
		return NULL;
    
    g_sr04_priv[idx].in_use = true;
    return &g_sr04_priv[idx];
}

/**
 * @brief   释放私有数据槽位
 * @param[in,out] priv 待释放的槽位 sr04_priv_t 结构体指针
 */
static void sr04_priv_free(sr04_priv_t *priv)
{
	if (priv)
		priv->in_use = false;
}

/**
 * @brief   SR04 触发测量，内部使用
 * @param[in] dev sr04_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
static void sr04_trigger(sr04_dev_t *dev)
{
	sr04_priv_t *priv = (sr04_priv_t *)dev->priv;

	sr04_hw_gpio_write_bit(dev->cfg.trig_port, dev->cfg.trig_pin, GPIO_LEVEL_LOW);
	dev->cfg.delay_us(2);
    sr04_hw_gpio_write_bit(dev->cfg.trig_port, dev->cfg.trig_pin, GPIO_LEVEL_HIGH);
    dev->cfg.delay_us(12);
    sr04_hw_gpio_write_bit(dev->cfg.trig_port, dev->cfg.trig_pin, GPIO_LEVEL_LOW);

	priv->is_first = true;
	priv->is_done = false;
}

/**
 * @brief   SR04 获取距离值
 * @param[in] dev sr04_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int sr04_get_distance_impl(sr04_dev_t *dev, float *distance_cm)
{
	if (!dev)
		return -EINVAL;

	sr04_priv_t *priv = (sr04_priv_t *)dev->priv;
	uint8_t timeout_ms;
	uint16_t duration;

	/* 触发测量 */
	sr04_trigger(dev);

    /* 等待捕获完成，最多延迟30ms */
    timeout_ms = 30;
    while (!priv->is_done && timeout_ms--)
		dev->cfg.delay_ms(1);

	/* 捕获完成，计算距离 */
    if (priv->is_done) {
        duration = (priv->t2 >= priv->t1) ? \
				   (priv->t2 - priv->t1) : \
				   (0xFFFF - priv->t1 + priv->t2);	// 处理定时器溢出

		*distance_cm = duration * 0.017f;  // 声速340m/s = 0.034cm/us，除以2表示往返距离
		return 0;
    }

	/* 超时或失败 */
	*distance_cm = -1.0f;
    return -ETIMEDOUT;
}

/**
 * @brief   去初始化 SR04 设备
 * @param[in,out] dev sr04_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int sr04_deinit_impl(sr04_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	sr04_priv_t *priv = (sr04_priv_t *)dev->priv;
	sr04_priv_free(priv);
	dev->priv = NULL;
	dev->ops = NULL;
	return 0;
}

/**
 * @brief   SR04 定时器中断回调函数，内部使用
 * @param[in] timer_periph 定时器外设
 */
static void sr04_timer_irq_handler(timer_periph_t timer_periph)
{
	const sr04_hw_info_t *hw_info = sr04_get_hw_info(timer_periph);
	sr04_priv_t *priv = &g_sr04_priv[hw_info->idx];
	sr04_dev_t *dev = priv->dev;
	uint16_t capture_val;
	uint8_t ic_channel = dev->cfg.ic_channel;

	if (sr04_hw_get_it_flag(timer_periph, ic_channel)) {
        sr04_hw_clear_it_flag(timer_periph, ic_channel);
		capture_val = sr04_hw_get_timer_ic_capture(timer_periph, ic_channel);

		if (priv->is_first) {
			/* 当前是第一次捕获（上升沿） */
			priv->t1 = capture_val;	// 保存上升沿时间
			priv->is_first = false;	// 下次再次捕获时不是第一次捕获

			/* 切换极性：下次捕获下降沿 */
			sr04_hw_set_timer_ic_polarity(timer_periph, ic_channel, true);

		} else {
			/* 当前是第二次捕获（下降沿） */
			priv->t2 = capture_val;	// 保存下降沿时间
			priv->is_done = true;	// 捕获完成
			priv->is_first = true;	// 下次再次捕获时是第一次捕获

			/* 重新切换为上升沿 */
			sr04_hw_set_timer_ic_polarity(timer_periph, ic_channel, false);
		}
	}
}

/* 各中断服务函数 */
#if SR04_TIMER1_IRQ_HANDLER_ENABLE
#if DRV_SR04_PLATFORM_GD32F1
void TIMER1_IRQHandler(void) { sr04_timer_irq_handler(TIMER1); }
#endif
#endif

#if SR04_TIMER2_IRQ_HANDLER_ENABLE
#if DRV_SR04_PLATFORM_STM32F1 || DRV_SR04_PLATFORM_STM32F4
void TIM2_IRQHandler(void) { sr04_timer_irq_handler(TIM2); }
#elif DRV_SR04_PLATFORM_GD32F1
void TIMER2_IRQHandler(void) { sr04_timer_irq_handler(TIMER2); }
#endif
#endif

#if SR04_TIMER3_IRQ_HANDLER_ENABLE
#if DRV_SR04_PLATFORM_STM32F1 || DRV_SR04_PLATFORM_STM32F4
void TIM3_IRQHandler(void) { sr04_timer_irq_handler(TIM3); }
#elif DRV_SR04_PLATFORM_GD32F1
void TIMER3_IRQHandler(void) { sr04_timer_irq_handler(TIMER3); }
#endif
#endif

#if SR04_TIMER4_IRQ_HANDLER_ENABLE
#if DRV_SR04_PLATFORM_STM32F1 || DRV_SR04_PLATFORM_STM32F4
void TIM4_IRQHandler(void) { sr04_timer_irq_handler(TIM4); }
#endif
#endif

#if SR04_TIMER5_IRQ_HANDLER_ENABLE
#if defined(STM32F10X_HD) || DRV_SR04_PLATFORM_STM32F4
void TIM5_IRQHandler(void) { sr04_timer_irq_handler(TIM5); }
#endif
#endif

/* ------------------------------- 核心驱动层结束 ------------------------------- */
