#include "delay.h"

#if !USE_FREERTOS
static uint16_t system_clock_mhz = 0; // 系统主频，单位为 MHz
#else
static TimerDev_t timer_delay;
#endif

#if !USE_FREERTOS
/******************************************************************************
 * @brief	初始化延时
 * @param	sysclk_mhz	:  系统主频（单位：MHz）
 * @return	无
 ******************************************************************************/
void delay_init(uint16_t sysclk_mhz)
{
    system_clock_mhz = sysclk_mhz;
}

/******************************************************************************
 * @brief	微秒级延时
 * @param	us	:  延时微秒数
 * @return	无
 ******************************************************************************/
void delay_us(uint32_t us)
{
    uint32_t temp;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;    // 使用系统时钟
    SysTick->LOAD = us * system_clock_mhz;          // 每微秒 system_clock_mhz 个时钟周期
    SysTick->VAL = 0x00;                            // 清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;       // 开始倒数
    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); // 等待时间到达
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;      // 关闭计数器
    SysTick->VAL = 0x00;                            // 清空计数器
}

#else
/* 由于 FreeRTOS 内部使用 SysTick 来实现系统节拍，所以需要改为使用定时器来实现延时 */

/******************************************************************************
 * @brief	初始化延时
 * @param	timer	:  TimerDev_t 结构体指针
 * @return	无
 ******************************************************************************/
void delay_init(TimerDev_t *timer)
{
    timer_delay = *timer;
    timer_init(&timer_delay);
}

/******************************************************************************
 * @brief	微秒级延时
 * @param	us	:  延时微秒数
 * @return	无
 ******************************************************************************/
void delay_us(uint32_t us)
{
    timer_delay.delay_us(&timer_delay, us);
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
