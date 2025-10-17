#include "timer.h"

#ifdef USE_STDPERIPH_DRIVER

/**************************** GD32F1 系列 ****************************/
#if defined(GD32F10X_MD)

#define MAX_TIMER_NUM	3

#define TIMER_FREQ	108000000

#define timer_clock_enable(timerx)									    \
    do {                                           				        \
        if (timerx == TIMER1)      rcu_periph_clock_enable(RCU_TIMER1); \
        else if (timerx == TIMER2) rcu_periph_clock_enable(RCU_TIMER2); \
        else if (timerx == TIMER3) rcu_periph_clock_enable(RCU_TIMER3); \
    } while (0)

#define timer_get_irqn(timerx) ((timerx) == TIMER1 ? TIMER1_IRQn : \
							   ((timerx) == TIMER2 ? TIMER2_IRQn : \
							   ((timerx) == TIMER3 ? TIMER3_IRQn : (IRQn_Type)0)))

#define timer_get_index(timerx)	((timerx) == TIMER1 ? 0 : \
								((timerx) == TIMER2 ? 1 : \
								((timerx) == TIMER3 ? 2 : (int)-1)))                
#endif	/* GD32F1 系列 */

#endif	/* USE_STDPERIPH_DRIVER */

/* 函数指针数组，保存用户注册的回调函数 */
static timer_irq_callback_t g_timer_irq_callback[MAX_TIMER_NUM];

/* 空指针数组，保存用户注册的回调函数参数 */
static void *g_timer_irq_callback_arg[MAX_TIMER_NUM] = {NULL};

/* 函数声明 */
static int timer_delay_us(timer_dev_t *dev, uint32_t us);
static int timer_delay_ms(timer_dev_t *dev, uint32_t ms);
static int timer_drv_deinit(timer_dev_t *dev);

/**
 * @brief   初始化定时器并配置中断
 * @details 以 GD32F1 为例，定时器时钟 108MHz，108M/PSC+1 为计数频率，其倒数为计数周期
			用作微秒级定时器时，PSC=108-1，计数周期 = 1us，定时周期 = (ARR+1)(us)，最大定时周期约为65.5ms
 * @param[in,out] dev timer_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */		
int timer_drv_init(timer_dev_t *dev)
{
	if (!dev)
		return -1;
	
	int8_t index = timer_get_index(dev->config.timx);
	
	/* 保存用户注册的回调函数指针与参数 */
	g_timer_irq_callback[index] = dev->config.irq_callback;
	g_timer_irq_callback_arg[index] = dev->config.irq_callback_arg;

	/* 配置时钟 */
	timer_clock_enable(dev->config.timx);
	
	/* 配置时钟源为内部时钟 */
	timer_internal_clock_config(dev->config.timx);
	
	/* 时基单元初始化 */
	timer_parameter_struct timer_initpara;
	timer_initpara.clockdivision = TIMER_CKDIV_DIV1;	// 时钟分频参数，不分频
	timer_initpara.alignedmode = TIMER_COUNTER_EDGE;	// 对齐模式
	timer_initpara.counterdirection = TIMER_COUNTER_UP;	// 计数器模式：向上计数
	timer_initpara.prescaler = dev->config.psc;			// PSC预分频器的值，范围0~65535
	timer_initpara.period = dev->config.arr;			// ARR自动重装器的值，范围0~65535
	timer_initpara.repetitioncounter = 0;				// 重复计数器的值（高级定时器用）
	timer_init(dev->config.timx, &timer_initpara);
	
	/* 配置中断 */
    if (dev->config.irq_callback != NULL) {
		timer_interrupt_flag_clear(dev->config.timx, TIMER_INT_FLAG_UP);	// 清除定时器更新标志位
		timer_interrupt_enable(dev->config.timx, TIMER_INT_UP);				// 配置中断源，开启更新中断
		nvic_irq_enable(timer_get_irqn(dev->config.timx), 
						dev->config.pre_priority, dev->config.sub_priority);
    }
	
	/* 启用定时器 */
	timer_enable(dev->config.timx);
	
	dev->delay_us = timer_delay_us;
	dev->delay_ms = timer_delay_ms;
	dev->deinit = timer_drv_deinit;
	
	dev->init_flag = true;
	return 0;
}

/**
 * @brief   定时器实现微秒级延时，需要正确配置 PSC 寄存器
 * @param[in] dev timer_dev_t 结构体指针
 * @param[in] us  要延时的微秒数
 * @return	0 表示成功，其他值表示失败
 */	
static int timer_delay_us(timer_dev_t *dev, uint32_t us)
{
	if (!dev || !dev->init_flag)
		return -1;

	if (timer_prescaler_read(dev->config.timx) != TIMER_FREQ / 1000000 - 1)	// 检查 PSC 配置
		return -2;

	if (us == 0)
        return 0;
	
	uint32_t start = timer_counter_read(dev->config.timx);	// 起始计数值
	uint32_t period = dev->config.arr + 1;					// 最大定时周期
	uint32_t elapsed;

	do {
		uint32_t current = timer_counter_read(dev->config.timx);						// 当前计数值
		elapsed = (current >= start) ? (current - start) : (period - start + current);	// 防溢出
	} while (elapsed < us);

	return 0;
}

/**
 * @brief   定时器实现毫秒级延时，需要正确配置 PSC 寄存器
 * @param[in] dev timer_dev_t 结构体指针
 * @param[in] us  要延时的毫秒数
 * @return	0 表示成功，其他值表示失败
 */	
static int timer_delay_ms(timer_dev_t *dev, uint32_t ms)
{
	if (!dev || !dev->init_flag)
		return -1;

	if (ms == 0)
        return 0;
	
	int ret;

	while (ms--) {
		ret = timer_delay_us(dev, 1000);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/**
 * @brief   去初始化定时器
 * @param[in,out] dev timer_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
static int timer_drv_deinit(timer_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	int8_t index = timer_get_index(dev->config.timx);

    if (index >= 0) {
        g_timer_irq_callback[index] = NULL;
        g_timer_irq_callback_arg[index] = NULL;
    }

	dev->init_flag = false;
	
	return 0;
}

/* 宏定义生成定时器中断处理函数 */
#define DEFINE_TIM_IRQ_HANDLER(timx, index) \
void TIMER##timx##_IRQHandler(void) \
{ \
    if (timer_interrupt_flag_get(TIMER##timx, TIMER_INT_FLAG_UP) == SET)	/* 检查中断标志位 */ \
	{ \
        timer_interrupt_flag_clear(TIMER##timx, TIMER_INT_FLAG_UP);	/* 清除中断标志位 */ \
        if (g_timer_irq_callback[index]) \
            g_timer_irq_callback[index](g_timer_irq_callback_arg[index]); \
    } \
}

#if TIM1_IRQ_HANDLER_ENABLE
DEFINE_TIM_IRQ_HANDLER(1, 0)
#endif

#if TIM2_IRQ_HANDLER_ENABLE
DEFINE_TIM_IRQ_HANDLER(2, 1)
#endif

#if TIM3_IRQ_HANDLER_ENABLE
DEFINE_TIM_IRQ_HANDLER(3, 2)
#endif
