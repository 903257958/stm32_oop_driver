#include <stdlib.h>
#include "timer.h"

#if defined(STM32F10X_HD)

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
										
#define	__timer_config_clock_enable(TIMx)	{	if(TIMx == TIM2)		{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);} \
												else if(TIMx == TIM3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);} \
												else if(TIMx == TIM4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);} \
												else if(TIMx == TIM5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);} \
												else if(TIMx == TIM6)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);} \
												else if(TIMx == TIM7)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);} \
												else					{timer_log("timer clock no enable\r\n");} \
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
										
#define	__timer_config_clock_enable(TIMx)	{	if(TIMx == TIM2)		{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);} \
												else if(TIMx == TIM3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);} \
												else if(TIMx == TIM4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);} \
												else if(TIMx == TIM5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);} \
												else if(TIMx == TIM6)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);} \
												else if(TIMx == TIM7)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);} \
												else					{timer_log("timer clock no enable\r\n");} \
											}

#elif defined(STM32F411xE)

#define MAX_TIMER_NUM	4

#define TIMER_FREQ	100000000

#define __timer_get_irqn_channel(TIMx)	(	TIMx == TIM2 ? TIM2_IRQn : \
											TIMx == TIM3 ? TIM3_IRQn : \
											TIMx == TIM4 ? TIM4_IRQn : \
											TIMx == TIM5 ? TIM5_IRQn : \
											(int)0	)
										
#define __timer_get_index(TIMx)	(	TIMx == TIM2 ? 0 : \
									TIMx == TIM3 ? 1 : \
									TIMx == TIM4 ? 2 : \
									TIMx == TIM5 ? 3 : \
									(int)-1	)
										
#define	__timer_config_clock_enable(TIMx)	{	if(TIMx == TIM2)		{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);} \
												else if(TIMx == TIM3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);} \
												else if(TIMx == TIM4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);} \
												else if(TIMx == TIM5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);} \
												else					{timer_log("timer clock no enable\r\n");} \
											}

#endif

/* 定时器私有数据结构体 */
typedef struct{
	uint32_t irqn;			// 中断号
	uint8_t index;			// 索引
}TimerPrivData_t;

/* 全局变量用于保存回调函数 */
static void (*g_irq_callback[MAX_TIMER_NUM])(void);

/* 函数声明 */
static int __timer_delay_us(TimerDev_t *pDev, uint32_t us);
static int __timer_delay_ms(TimerDev_t *pDev, uint32_t ms);
static int __timer_deinit(TimerDev_t *pDev);

/******************************************************************************
 * @brief	初始化定时器并配置中断
			以STM32F1为例，定时器时钟72MHz，72M/PSC为计数频率，其倒数为计数周期
			用作微秒级定时器时，PSC = 72 - 1，计数周期 = 1us，定时周期 = (ASC + 1)(us)，最大定时周期约为65.5ms
			用作毫秒级定时器时，PSC = 7200 - 1，计数周期 = 0.1ms，定时周期 = ((ASC + 1)/10))(ms)，最大定时周期约为6.55s
 * @param	pDev	:	TimerDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int timer_init(TimerDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 初始化私有数据 */
	pDev->pPrivData = (TimerPrivData_t *)malloc(sizeof(TimerPrivData_t));
	if (!pDev->pPrivData)
		return -1;
	
	TimerPrivData_t *pPrivData = (TimerPrivData_t *)pDev->pPrivData;
	
	pPrivData->irqn = __timer_get_irqn_channel(pDev->info.timx);
	pPrivData->index =__timer_get_index(pDev->info.timx);
	
	/* 配置时钟 */
	__timer_config_clock_enable(pDev->info.timx);
	
	/* 配置时钟源 */
	TIM_InternalClockConfig(pDev->info.timx);
	
	/* 时基单元初始化 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;				// 时钟分频参数，不分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;			// 计数器模式：向上计数
	TIM_TimeBaseInitStructure.TIM_Prescaler = pDev->info.psc;				// PSC预分频器的值，范围0~65535
	TIM_TimeBaseInitStructure.TIM_Period = pDev->info.arr;					// ARR自动重装器的值，范围0~65535
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;					// 重复计数器的值（高级定时器用）
	TIM_TimeBaseInit(pDev->info.timx, &TIM_TimeBaseInitStructure);
	
	/* 配置中断 */
	TIM_ClearFlag(pDev->info.timx, TIM_FLAG_Update);		// 清除定时器更新标志位
	TIM_ITConfig(pDev->info.timx, TIM_IT_Update, ENABLE);	// 更新中断
		
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = pPrivData->irqn;			// 中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					// 中断通道使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		// 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				// 响应优先级
	NVIC_Init(&NVIC_InitStructure);
	
	/* 启用定时器 */
	TIM_Cmd(pDev->info.timx, ENABLE);
	
	/* 注册回调函数 */
	g_irq_callback[pPrivData->index] = pDev->info.irq_callback;
	
	/* 函数指针赋值 */
	pDev->delay_us = __timer_delay_us;
	pDev->delay_ms = __timer_delay_ms;
	pDev->deinit = __timer_deinit;
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	TIM实现微秒级延时，需要正确配置PSC寄存器
 * @param	pDev	：	TimerDev_t结构体指针
 * @param	us		：	要延时的微秒数
 * @return	延时成功返回us数，延时失败则返回-1
 ******************************************************************************/
static int __timer_delay_us(TimerDev_t *pDev, uint32_t us)
{
	if (TIM_GetPrescaler(pDev->info.timx) != TIMER_FREQ / 1000000 - 1)	// 判断PSC配置
		return -1;
	
    uint32_t start = TIM_GetCounter(pDev->info.timx);					// 记录当前计数值
    
    while ((TIM_GetCounter(pDev->info.timx) - start) < us);				// 等待指定的微秒数

	return us;
}

/******************************************************************************
 * @brief	TIM实现毫秒级延时，需要正确配置PSC寄存器
 * @param	pDev	：	TimerDev_t结构体指针
 * @param	us		：	要延时的毫秒数
 * @return	延时成功返回ms数，延时失败则返回-1
 ******************************************************************************/
static int __timer_delay_ms(TimerDev_t *pDev, uint32_t ms)
{
	if (TIM_GetPrescaler(pDev->info.timx) != TIMER_FREQ / 1000000 - 1)	// 判断PSC配置
		return -1;
	
	while (ms--)
	{
		__timer_delay_us(pDev, 1000);
	}

	return ms;
}

/******************************************************************************
 * @brief	去初始化定时器
 * @param	pDev   :  TimerDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __timer_deinit(TimerDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	/* 释放私有数据内存 */
	free(pDev->pPrivData);
    pDev->pPrivData = NULL;
	
	pDev->initFlag = false;	// 修改初始化标志
	
	return 0;
}

/******************************************************************************
 * @brief	TIM2中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)	// 检查中断标志位
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	// 清除中断标志位
		
		if (g_irq_callback[0] != NULL)
		{
			g_irq_callback[0]();
		}
	}
}

/******************************************************************************
 * @brief	TIM3中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)	// 检查中断标志位
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	// 清除中断标志位
		
		if (g_irq_callback[1] != NULL)
		{
			g_irq_callback[1]();
		}
	}
}

/******************************************************************************
 * @brief	TIM4中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)	// 检查中断标志位
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);	// 清除中断标志位
		
		if (g_irq_callback[2] != NULL)
		{
			g_irq_callback[2]();
		}
	}
}

/******************************************************************************
 * @brief	TIM5中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)	// 检查中断标志位
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);	// 清除中断标志位
		
		if (g_irq_callback[3] != NULL)
		{
			g_irq_callback[3]();
		}
	}
}

/******************************************************************************
 * @brief	TIM6中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
#if defined(STM32F10X_HD)
void TIM6_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)	// 检查中断标志位
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);	// 清除中断标志位
		
		if (g_irq_callback[4] != NULL)
		{
			g_irq_callback[4]();
		}
	}
}
#elif defined(STM32F40_41xxx)
void TIM6_DAC_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)	// 检查中断标志位
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);	// 清除中断标志位
		
		if (g_irq_callback[4] != NULL)
		{
			g_irq_callback[4]();
		}
	}
}
#endif

#if defined(STM32F10X_HD) || defined(STM32F40_41xxx)
/******************************************************************************
 * @brief	TIM7中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)	// 检查中断标志位
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);	// 清除中断标志位
		
		if (g_irq_callback[5] != NULL)
		{
			g_irq_callback[5]();
		}
	}
}
#endif
