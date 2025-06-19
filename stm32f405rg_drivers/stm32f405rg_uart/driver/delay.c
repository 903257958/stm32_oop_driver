#include "delay.h"

/* 系统主频 */
extern uint32_t SystemCoreClock;

#if USE_FREERTOS
static bool g_init_flag = false;
static timer_dev_t g_timer_delay;
#endif

#if !USE_FREERTOS

/******************************************************************************
 * @brief	微秒级延时
 * @param	us	:  延时微秒数
 * @return	无
 ******************************************************************************/
void delay_us(uint32_t us)
{
    uint32_t tmp;
    
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;            // 使用系统主频作为SysTick的时钟源
    SysTick->LOAD = us * (SystemCoreClock / 1000000) - 1;   // 设置倒计时初始值
    SysTick->VAL = 0x00;                                    // 清空计数器，确保从头计时
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;               // 启动计数器，开始倒数
    do
    {
        tmp = SysTick->CTRL;
    } while ((tmp & 0x01) && !(tmp & (1 << 16)));           // 等待时间到达
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;              // 延时结束，关闭计数器
    SysTick->VAL = 0x00;                                    // 清空计数器
}

#else
/* 由于 FreeRTOS 内部使用 SysTick 来实现系统节拍，所以需要改为使用定时器来实现延时 */

/******************************************************************************
 * @brief	初始化延时
 * @param	timer	:  timer_dev_t 结构体指针
 * @return	无
 ******************************************************************************/
void delay_init(timer_dev_t *timer)
{
    /* 保存传入的定时器设备 */
    g_timer_delay = *timer;
    
    /* 定时器初始化 */
    timer_init(&g_timer_delay);

    g_init_flag = true;
}

/******************************************************************************
 * @brief	微秒级延时
 * @param	us	:  延时微秒数
 * @return	无
 ******************************************************************************/
void delay_us(uint32_t us)
{
    if (g_init_flag == false)
        return;
        
    g_timer_delay.delay_us(&g_timer_delay, us);
}
#endif

/******************************************************************************
 * @brief	毫秒级延时
 * @param	ms	:  延时毫秒数
 * @return	无
 ******************************************************************************/
void delay_ms(uint32_t ms)
{
	while(ms--)
	{
		delay_us(1000);
	}
}

/******************************************************************************
 * @brief	秒级延时
 * @param	s	:  延时秒数
 * @return	无
 ******************************************************************************/
void delay_s(uint32_t s)
{
	while(s--)
	{
		delay_ms(1000);
	}
}
