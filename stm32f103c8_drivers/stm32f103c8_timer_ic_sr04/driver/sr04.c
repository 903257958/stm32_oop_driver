#include "sr04.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_MD) || defined(STM32F10X_HD)

#define TIMER_FREQ	72000000

#define	__sr04_timer_clock_enable(timx)	{	if (timx == TIM2)		{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);} \
											else if (timx == TIM3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);} \
											else if (timx == TIM4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);} \
										}

#define	__sr04_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
											else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
											else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
											else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
											else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
											else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
											else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
										}

#define	__sr04_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__sr04_config_io_in_pn(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__gpio_write_bit(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

	#if defined(STM32F10X_MD)

	#define __sr04_get_irqn_channel(timx)	(	timx == TIM2 ? TIM2_IRQn : \
												timx == TIM3 ? TIM3_IRQn : \
												timx == TIM4 ? TIM4_IRQn : \
												(int)0	)
	#elif defined(STM32F10X_HD)

	#define __sr04_get_irqn_channel(timx)	(	timx == TIM2 ? TIM2_IRQn : \
												timx == TIM3 ? TIM3_IRQn : \
												timx == TIM4 ? TIM4_IRQn : \
												timx == TIM5 ? TIM5_IRQn : \
												timx == TIM6 ? TIM6_IRQn : \
												timx == TIM7 ? TIM7_IRQn : \
												(int)0	)
	#endif

#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)

	#if defined(STM32F40_41xxx)

	#define TIMER_FREQ	84000000

	#elif defined(STM32F429_439xx)

	#define TIMER_FREQ	90000000
	
	#endif

#define	__sr04_timer_clock_enable(timx)	{	if (timx == TIM2)		{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);} \
											else if (timx == TIM3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);} \
											else if (timx == TIM4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);} \
											else if (timx == TIM5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);} \
										}

#define	__sr04_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
											else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
											else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
											else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
											else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
											else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
											else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
										}

#define	__sr04_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
                                                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
                                                GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
                                                GPIO_InitStructure.GPIO_Pin = pin; \
                                                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
                                                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
                                                GPIO_Init(port, &GPIO_InitStructure); \
                                            }

#define	__sr04_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__gpio_write_bit(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#define __sr04_get_irqn_channel(timx)	(	timx == TIM2 ? TIM2_IRQn : \
											timx == TIM3 ? TIM3_IRQn : \
											timx == TIM4 ? TIM4_IRQn : \
											timx == TIM5 ? TIM5_IRQn : \
											(int)0	)
                                                
#define __sr04_get_gpio_pin_sourse(pin)	(	pin == GPIO_Pin_0 ? GPIO_PinSource0 : \
											pin == GPIO_Pin_1 ? GPIO_PinSource1 : \
											pin == GPIO_Pin_2 ? GPIO_PinSource2 : \
											pin == GPIO_Pin_3 ? GPIO_PinSource3 : \
											pin == GPIO_Pin_4 ? GPIO_PinSource4 : \
											pin == GPIO_Pin_5 ? GPIO_PinSource5 : \
											pin == GPIO_Pin_6 ? GPIO_PinSource6 : \
											pin == GPIO_Pin_7 ? GPIO_PinSource7 : \
											pin == GPIO_Pin_8 ? GPIO_PinSource8 : \
											pin == GPIO_Pin_9 ? GPIO_PinSource9 : \
											pin == GPIO_Pin_10 ? GPIO_PinSource10 : \
											pin == GPIO_Pin_11 ? GPIO_PinSource11 : \
											pin == GPIO_Pin_12 ? GPIO_PinSource12 : \
											pin == GPIO_Pin_13 ? GPIO_PinSource13 : \
											pin == GPIO_Pin_14 ? GPIO_PinSource14 : \
											pin == GPIO_Pin_15 ? GPIO_PinSource15 : \
											(int)0)
                                                
#define __sr04_get_gpio_af(timx)	(	timx == TIM2 ? GPIO_AF_TIM2 : \
                                        timx == TIM3 ? GPIO_AF_TIM3 : \
                                        timx == TIM4 ? GPIO_AF_TIM4 : \
                                        timx == TIM5 ? GPIO_AF_TIM5 : \
                                        (int)0)

#endif

#endif

/* SR04设备指针数组 */
sr04_dev_t *g_sr04_dev[MAX_SR04_NUM] = {NULL};
uint8_t g_sr04_num;

/* SR04私有数据结构体 */
typedef struct {
	volatile uint16_t t1;	// 捕获的上升沿时间（Echo开始变高）
	volatile uint16_t t2;	// 捕获的下降沿时间（Echo结束变低）
	volatile bool is_first;	// 标记当前是第一次捕获（上升沿）还是第二次捕获（下降沿）
	volatile bool done;		// 捕获过程是否完成
} sr04_priv_data_t;

/* 函数声明 */
static void __sr04_trigger(sr04_dev_t *dev);
static int8_t __sr04_get_distance(sr04_dev_t *dev);
static int8_t __sr04_deinit(sr04_dev_t *dev);
static void __sr04_timx_irq_callback(timer_periph_t timx);

/******************************************************************************
 * @brief	SR04初始化
 * @param	dev	:	sr04_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t sr04_init(sr04_dev_t *dev)
{
	if (!dev)
		return -1;

	if (g_sr04_num >= MAX_SR04_NUM)
		return -2;	// 初始化前发现已注册的设备已经等于最大设备数，初始化失败

	/* 记录设备指针用于中断处理 */
	uint8_t index = g_sr04_num;
	g_sr04_dev[index] = dev;

	/* 初始化私有数据 */
	dev->priv_data = (sr04_priv_data_t *)malloc(sizeof(sr04_priv_data_t));
	if (!dev->priv_data)
		return -3;
	
	sr04_priv_data_t *priv_data = (sr04_priv_data_t *)dev->priv_data;
	priv_data->t1 = 0;
	priv_data->t2 = 0;
	priv_data->is_first = true;
	priv_data->done = false;

	/* 开启时钟 */
	__sr04_timer_clock_enable(dev->config.timx);
	__sr04_io_clock_enable(dev->config.trig_port);
	__sr04_io_clock_enable(dev->config.echo_port);

	/* 配置GPIO */
    #if defined(STM32F10X_MD) || defined(STM32F10X_HD)
	__sr04_config_io_out_pp(dev->config.trig_port, dev->config.trig_pin);	// Trig 推挽输出
	__sr04_config_io_in_pn(dev->config.echo_port, dev->config.echo_pin);	// Echo 浮空输入
    
    #elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
    __sr04_config_io_out_pp(dev->config.trig_port, dev->config.trig_pin);	// Trig 推挽输出
	__sr04_config_io_af_pp(dev->config.echo_port, dev->config.echo_pin);	// Echo 复用模式
    
    GPIO_PinAFConfig(dev->config.echo_port, 
                    __sr04_get_gpio_pin_sourse(dev->config.echo_pin), 
                    __sr04_get_gpio_af(dev->config.timx));
    #endif

	/* 初始化时基单元 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			// 时钟分频，选择不分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		// 计数器模式，选择向上计数
    TIM_TimeBaseInitStructure.TIM_Prescaler = TIMER_FREQ / 1000000 - 1;	// PSC，计数频率1MHz，计数周期1us
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;						// ARR，可测最大时间65.5ms
    TIM_TimeBaseInit(dev->config.timx, &TIM_TimeBaseInitStructure);

    /* 初始化输入捕获 */
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;							// 输入滤波器参数
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;		// 极性，选择为上升沿触发捕获
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			// 捕获预分频，选择不分频，每次信号都触发捕获
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	// 输入信号交叉，选择直通，不交叉
	if (dev->config.ic_channel == 1)
	{
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	}
	else if (dev->config.ic_channel == 2)
	{
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	}
	else if (dev->config.ic_channel == 3)
	{
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	}
	else if (dev->config.ic_channel == 4)
	{
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	}
    TIM_ICInit(dev->config.timx, &TIM_ICInitStructure);

    /* 初始化NVIC */
    NVIC_InitTypeDef NVIC_InitStructure;
	if (dev->config.ic_channel == 1)
	{
		TIM_ITConfig(dev->config.timx, TIM_IT_CC1, ENABLE);
	}
	else if (dev->config.ic_channel == 2)
	{
		TIM_ITConfig(dev->config.timx, TIM_IT_CC2, ENABLE);
	}
	else if (dev->config.ic_channel == 3)
	{
		TIM_ITConfig(dev->config.timx, TIM_IT_CC3, ENABLE);
	}
	else if (dev->config.ic_channel == 4)
	{
		TIM_ITConfig(dev->config.timx, TIM_IT_CC4, ENABLE);
	}
    NVIC_InitStructure.NVIC_IRQChannel = __sr04_get_irqn_channel(dev->config.timx);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 启动定时器 */
    TIM_Cmd(dev->config.timx, ENABLE);

	/* SR04设备数加1 */
	g_sr04_num++;

	/* 函数指针赋值 */
	dev->get_distance = __sr04_get_distance;
	dev->deinit = __sr04_deinit;
	
	dev->init_flag = true;

	return 0;
}

/******************************************************************************
 * @brief	SR04触发测量，内部使用
 * @param	dev	:	sr04_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static void __sr04_trigger(sr04_dev_t *dev)
{
	sr04_priv_data_t *priv_data = (sr04_priv_data_t *)dev->priv_data;

	__gpio_write_bit(dev->config.trig_port, dev->config.trig_pin, GPIO_LEVEL_LOW);
    SR04_DELAY_US(2);
    __gpio_write_bit(dev->config.trig_port, dev->config.trig_pin, GPIO_LEVEL_HIGH);
    SR04_DELAY_US(10);
    __gpio_write_bit(dev->config.trig_port, dev->config.trig_pin, GPIO_LEVEL_LOW);

	priv_data->is_first = true;
	priv_data->done = false;
}

/******************************************************************************
 * @brief	SR04获取距离值
 * @param	dev	:	sr04_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __sr04_get_distance(sr04_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	sr04_priv_data_t *priv_data = (sr04_priv_data_t *)dev->priv_data;

	uint8_t timeout_ms;
	uint16_t duration;

	/* 触发测量 */
	__sr04_trigger(dev);

    /* 等待捕获完成，最多延迟30ms */
    timeout_ms = 30;
    while (!priv_data->done && timeout_ms--)
	{
		SR04_DELAY_MS(1);
	}

	/* 捕获完成，计算距离 */
    if (priv_data->done)
    {
        duration = 	(priv_data->t2 >= priv_data->t1) ? \
					(priv_data->t2 - priv_data->t1) : \
					(0xFFFF - priv_data->t1 + priv_data->t2);	// 处理定时器溢出

		dev->distance_cm = duration * 0.017f;  // 声速340m/s = 0.034cm/us，除以2表示往返距离
		return 0;
    }

	/* 超时或失败 */
	dev->distance_cm = -1.0f;
    return -2;
}

/******************************************************************************
 * @brief	去初始化SR04
 * @param	dev   :  sr04_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __sr04_deinit(sr04_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	sr04_priv_data_t *priv_data = (sr04_priv_data_t *)dev->priv_data;
	
	free(priv_data);
	dev->priv_data = NULL;
	dev->init_flag = false;
	
	return 0;
}

/******************************************************************************
 * @brief	SR04定时器中断回调函数，内部使用
 * @param	timx	:	定时器外设
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static void __sr04_timx_irq_callback(timer_periph_t timx)
{
	uint8_t i, ic_channel;
	uint16_t capture_val;

	/* 遍历找到与此定时器对应的SR04设备 */
	for (i = 0; i < g_sr04_num; i++)
    {
        sr04_dev_t *dev = g_sr04_dev[i];
        if (dev->config.timx != timx) continue;

        sr04_priv_data_t *priv_data = (sr04_priv_data_t *)dev->priv_data;
        ic_channel = dev->config.ic_channel;

        if ((ic_channel == 1 && TIM_GetITStatus(timx, TIM_IT_CC1) != RESET) ||
            (ic_channel == 2 && TIM_GetITStatus(timx, TIM_IT_CC2) != RESET) ||
            (ic_channel == 3 && TIM_GetITStatus(timx, TIM_IT_CC3) != RESET) ||
            (ic_channel == 4 && TIM_GetITStatus(timx, TIM_IT_CC4) != RESET))
        {
            TIM_ClearITPendingBit(timx, TIM_IT_CC1 << (ic_channel - 1));	// 清中断标志

            capture_val = 0;
            switch (ic_channel)
            {
                case 1: capture_val = TIM_GetCapture1(timx); break;
                case 2: capture_val = TIM_GetCapture2(timx); break;
                case 3: capture_val = TIM_GetCapture3(timx); break;
                case 4: capture_val = TIM_GetCapture4(timx); break;
            }

            if (priv_data->is_first)
            {
				/* 当前是第一次捕获（上升沿） */
                priv_data->t1 = capture_val;	// 保存上升沿时间

                /* 切换极性：下次捕获下降沿 */
                switch (ic_channel)
                {
                    case 1: TIM_OC1PolarityConfig(timx, TIM_ICPolarity_Falling); break;
                    case 2: TIM_OC2PolarityConfig(timx, TIM_ICPolarity_Falling); break;
                    case 3: TIM_OC3PolarityConfig(timx, TIM_ICPolarity_Falling); break;
                    case 4: TIM_OC4PolarityConfig(timx, TIM_ICPolarity_Falling); break;
                }

                priv_data->is_first = false;
            }
            else
            {
				/* 当前是第二次捕获（下降沿） */
                priv_data->t2 = capture_val;	// 保存下降沿时间
                priv_data->done = true;			// 捕获完成

                /* 重新切换为上升沿 */
                switch (ic_channel)
                {
                    case 1: TIM_OC1PolarityConfig(timx, TIM_ICPolarity_Rising); break;
                    case 2: TIM_OC2PolarityConfig(timx, TIM_ICPolarity_Rising); break;
                    case 3: TIM_OC3PolarityConfig(timx, TIM_ICPolarity_Rising); break;
                    case 4: TIM_OC4PolarityConfig(timx, TIM_ICPolarity_Rising); break;
                }

                priv_data->is_first = true;
            }
        }
    }
}

#if SR04_TIM2_IRQ_HANDLER_ENABLE
/******************************************************************************
 * @brief	TIM2中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void TIM2_IRQHandler(void)
{
	__sr04_timx_irq_callback(TIM2);
}
#endif

#if SR04_TIM3_IRQ_HANDLER_ENABLE
/******************************************************************************
 * @brief	TIM2中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void TIM3_IRQHandler(void)
{
    __sr04_timx_irq_callback(TIM3);
}
#endif

#if SR04_TIM4_IRQ_HANDLER_ENABLE
/******************************************************************************
 * @brief	TIM4中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void TIM4_IRQHandler(void)
{
	__sr04_timx_irq_callback(TIM4);
}
#endif

#if SR04_TIM5_IRQ_HANDLER_ENABLE
/******************************************************************************
 * @brief	TIM5中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void TIM5_IRQHandler(void)
{
	__sr04_timx_irq_callback(TIM5);
}
#endif
