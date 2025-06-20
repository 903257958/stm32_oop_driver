#include "timer.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_MD)

#define MAX_TIMER_NUM	3

#define TIMER_FREQ	72000000
	
#define __timer_get_irqn_channel(TIMx)	(	TIMx == TIM2 ? TIM2_IRQn : \
											TIMx == TIM3 ? TIM3_IRQn : \
											TIMx == TIM4 ? TIM4_IRQn : \
											(int)0	)
										
#define __timer_get_index(TIMx)	(	TIMx == TIM2 ? 0 : \
									TIMx == TIM3 ? 1 : \
									TIMx == TIM4 ? 2 : \
									(int)-1	)
										
#define	__timer_clock_enable(TIMx)	{	if (TIMx == TIM2)		{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);} \
										else if (TIMx == TIM3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);} \
										else if (TIMx == TIM4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);} \
									}
                                            
#elif defined(STM32F10X_HD)

#define MAX_TIMER_NUM	6

#define TIMER_FREQ	72000000
	
#define __timer_get_irqn_channel(TIMx)	(	TIMx == TIM2 ? TIM2_IRQn : \
											TIMx == TIM3 ? TIM3_IRQn : \
											TIMx == TIM4 ? TIM4_IRQn : \
											TIMx == TIM5 ? TIM5_IRQn : \
											TIMx == TIM6 ? TIM6_IRQn : \
											TIMx == TIM7 ? TIM7_IRQn : \
											(int)0	)
										
#define __timer_get_index(TIMx)	(	TIMx == TIM2 ? 0 : \
									TIMx == TIM3 ? 1 : \
									TIMx == TIM4 ? 2 : \
									TIMx == TIM5 ? 3 : \
									TIMx == TIM6 ? 4 : \
									TIMx == TIM7 ? 5 : \
									(int)-1	)
										
#define	__timer_clock_enable(TIMx)	{	if (TIMx == TIM2)		{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);} \
										else if (TIMx == TIM3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);} \
										else if (TIMx == TIM4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);} \
										else if (TIMx == TIM5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);} \
										else if (TIMx == TIM6)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);} \
										else if (TIMx == TIM7)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);} \
									}

#elif defined(STM32F40_41xxx)

#define MAX_TIMER_NUM	6

#define TIMER_FREQ	84000000

#define __timer_get_irqn_channel(TIMx)	(	TIMx == TIM2 ? TIM2_IRQn : \
											TIMx == TIM3 ? TIM3_IRQn : \
											TIMx == TIM4 ? TIM4_IRQn : \
											TIMx == TIM5 ? TIM5_IRQn : \
											TIMx == TIM6 ? TIM6_DAC_IRQn : \
											TIMx == TIM7 ? TIM7_IRQn : \
											(int)0	)
										
#define __timer_get_index(TIMx)	(	TIMx == TIM2 ? 0 : \
									TIMx == TIM3 ? 1 : \
									TIMx == TIM4 ? 2 : \
									TIMx == TIM5 ? 3 : \
									TIMx == TIM6 ? 4 : \
									TIMx == TIM7 ? 5 : \
									(int)-1	)
										
#define	__timer_clock_enable(TIMx)	{	if (TIMx == TIM2)		{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);} \
										else if (TIMx == TIM3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);} \
										else if (TIMx == TIM4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);} \
										else if (TIMx == TIM5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);} \
										else if (TIMx == TIM6)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);} \
										else if (TIMx == TIM7)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);} \
									}

#elif defined(STM32F429_439xx)

#define MAX_TIMER_NUM	6

#define TIMER_FREQ	90000000

#define __timer_get_irqn_channel(TIMx)	(	TIMx == TIM2 ? TIM2_IRQn : \
											TIMx == TIM3 ? TIM3_IRQn : \
											TIMx == TIM4 ? TIM4_IRQn : \
											TIMx == TIM5 ? TIM5_IRQn : \
                                            TIMx == TIM6 ? TIM6_DAC_IRQn : \
											TIMx == TIM7 ? TIM7_IRQn : \
											(int)0	)
										
#define __timer_get_index(TIMx)	(	TIMx == TIM2 ? 0 : \
									TIMx == TIM3 ? 1 : \
									TIMx == TIM4 ? 2 : \
									TIMx == TIM5 ? 3 : \
                                    TIMx == TIM6 ? 4 : \
									TIMx == TIM7 ? 5 : \
									(int)-1	)
										
#define	__timer_clock_enable(TIMx)	{	if (TIMx == TIM2)		{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);} \
										else if (TIMx == TIM3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);} \
										else if (TIMx == TIM4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);} \
										else if (TIMx == TIM5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);} \
										else if (TIMx == TIM6)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);} \
										else if (TIMx == TIM7)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);} \
									}
                                            
#endif

#endif

/* 函数指针数组，保存用户注册的回调函数 */
static timer_irq_callback_t g_timer_irq_callback[MAX_TIMER_NUM];

/* 空指针数组，保存用户注册的回调函数参数 */
static void *g_timer_irq_callback_param[MAX_TIMER_NUM] = {NULL};

/* 函数声明 */
static int __timer_delay_us(timer_dev_t *dev, uint32_t us);
static int __timer_delay_ms(timer_dev_t *dev, uint32_t ms);
static int8_t __timer_deinit(timer_dev_t *dev);

/******************************************************************************
 * @brief	初始化定时器并配置中断
			以STM32F1为例，定时器时钟72MHz，72M/PSC为计数频率，其倒数为计数周期
			用作微秒级定时器时，PSC = 72 - 1，计数周期 = 1us，定时周期 = (ARR + 1)(us)，最大定时周期约为65.5ms
			用作毫秒级定时器时，PSC = 7200 - 1，计数周期 = 0.1ms，定时周期 = ((ARR + 1)/10))(ms)，最大定时周期约为6.55s
 * @param	dev	:	timer_dev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t timer_init(timer_dev_t *dev)
{
	if (!dev)
		return -1;
	
	int8_t index = __timer_get_index(dev->config.timx);
	
	/* 保存用户注册的回调函数指针与参数 */
	g_timer_irq_callback[index] = dev->config.irq_callback;
	g_timer_irq_callback_param[index] = dev->config.irq_callback_param;

	/* 配置时钟 */
	__timer_clock_enable(dev->config.timx);
	
	/* 配置时钟源 */
	TIM_InternalClockConfig(dev->config.timx);
	
	/* 时基单元初始化 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// 时钟分频参数，不分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	// 计数器模式：向上计数
	TIM_TimeBaseInitStructure.TIM_Prescaler = dev->config.psc;		// PSC预分频器的值，范围0~65535
	TIM_TimeBaseInitStructure.TIM_Period = dev->config.arr;			// ARR自动重装器的值，范围0~65535
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			// 重复计数器的值（高级定时器用）
	TIM_TimeBaseInit(dev->config.timx, &TIM_TimeBaseInitStructure);
	
	/* 配置中断 */
    if (dev->config.irq_callback != NULL)
    {
        TIM_ClearFlag(dev->config.timx, TIM_FLAG_Update);		// 清除定时器更新标志位
        TIM_ITConfig(dev->config.timx, TIM_IT_Update, ENABLE);	// 更新中断
            
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = __timer_get_irqn_channel(dev->config.timx);		// 中断通道
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;											// 中断通道使能
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = dev->config.preemption_priority;	// 抢占优先级
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = dev->config.sub_priority;				// 响应优先级
        NVIC_Init(&NVIC_InitStructure);
    }
	
	/* 启用定时器 */
	TIM_Cmd(dev->config.timx, ENABLE);
	
	/* 函数指针赋值 */
	dev->delay_us = __timer_delay_us;
	dev->delay_ms = __timer_delay_ms;
	dev->deinit = __timer_deinit;
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	TIM实现微秒级延时，需要正确配置PSC寄存器
 * @param	dev	:	timer_dev_t结构体指针
 * @param	us	:	要延时的微秒数
 * @return	延时成功返回us数，延时失败则返回-1
 ******************************************************************************/
static int __timer_delay_us(timer_dev_t *dev, uint32_t us)
{
	if (TIM_GetPrescaler(dev->config.timx) != TIMER_FREQ / 1000000 - 1)	// 判断PSC配置
		return -1;
	
	uint32_t start = TIM_GetCounter(dev->config.timx);	// 起始计数值
	uint32_t period = dev->config.arr + 1;				// 最大定时周期
	uint32_t elapsed;

	do {
		uint32_t current = TIM_GetCounter(dev->config.timx);							// 当前计数值
		elapsed = (current >= start) ? (current - start) : (period - start + current);	// 防溢出
	} while (elapsed < us);

	return us;
}

/******************************************************************************
 * @brief	TIM实现毫秒级延时，需要正确配置PSC寄存器
 * @param	dev	:	timer_dev_t结构体指针
 * @param	us	:	要延时的毫秒数
 * @return	延时成功返回ms数，延时失败则返回-1
 ******************************************************************************/
static int __timer_delay_ms(timer_dev_t *dev, uint32_t ms)
{
	if (TIM_GetPrescaler(dev->config.timx) != TIMER_FREQ / 1000000 - 1)	// 判断PSC配置
		return -1;
	
	while (ms--)
	{
		__timer_delay_us(dev, 1000);
	}

	return ms;
}

/******************************************************************************
 * @brief	去初始化定时器
 * @param	dev	:	timer_dev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __timer_deinit(timer_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	int8_t index = __timer_get_index(dev->config.timx);

    if(index >= 0)
	{
        g_timer_irq_callback[index] = NULL;
        g_timer_irq_callback_param[index] = NULL;
    }

	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}

/* 宏定义生成定时器中断处理函数 */
#define DEFINE_TIM_IRQ_HANDLER(timx, index) \
void TIM##timx##_IRQHandler(void) \
{ \
    if (TIM_GetITStatus(TIM##timx, TIM_IT_Update) == SET)	/* 检查中断标志位 */ \
	{ \
        TIM_ClearITPendingBit(TIM##timx, TIM_IT_Update);	/* 清除中断标志位 */ \
        if (g_timer_irq_callback[index]) \
            g_timer_irq_callback[index](g_timer_irq_callback_param[index]); \
    } \
}

#if TIM2_IRQ_HANDLER_ENABLE
DEFINE_TIM_IRQ_HANDLER(2, 0)
#endif

#if TIM3_IRQ_HANDLER_ENABLE
DEFINE_TIM_IRQ_HANDLER(3, 1)
#endif

#if TIM4_IRQ_HANDLER_ENABLE
DEFINE_TIM_IRQ_HANDLER(4, 2)
#endif

#if TIM5_IRQ_HANDLER_ENABLE
DEFINE_TIM_IRQ_HANDLER(5, 3)
#endif

#if TIM6_IRQ_HANDLER_ENABLE
#if defined(STM32F10X_HD)
DEFINE_TIM_IRQ_HANDLER(6, 4)
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)
void TIM6_DAC_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)	// 检查中断标志位
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);	// 清除中断标志位
		
		if (g_timer_irq_callback[4] != NULL)
		{
			g_timer_irq_callback[4](g_timer_irq_callback_param[4]);
		}
	}
}
#endif
#endif

#if TIM7_IRQ_HANDLER_ENABLE
DEFINE_TIM_IRQ_HANDLER(7, 5)
#endif
