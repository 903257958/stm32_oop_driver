#include "drv_delay.h"
#include <stddef.h>

/* 系统主频 */
extern uint32_t SystemCoreClock;

/**
 * @brief   微秒级延时
 * @param[in] us 延时微秒数
 */
void delay_us(uint32_t us)
{
    if (us == 0)
        return;
    
    uint32_t tmp;
    
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;            // 使用系统主频作为SysTick的时钟源
    SysTick->LOAD = us * (SystemCoreClock / 1000000) - 1;   // 设置倒计时初始值
    SysTick->VAL = 0x00;                                    // 清空计数器，确保从头计时
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;               // 启动计数器，开始倒数
    do {
        tmp = SysTick->CTRL;
    } while ((tmp & 0x01) && !(tmp & (1 << 16)));           // 等待时间到达
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;              // 延时结束，关闭计数器
    SysTick->VAL = 0x00;                                    // 清空计数器
}

/**
 * @brief   毫秒级延时
 * @param[in] ms 延时毫秒数
 */
void delay_ms(uint32_t ms)
{
    if (ms == 0)
        return;
    
	while (ms--)
		delay_us(1000);
}

/**
 * @brief   秒级延时
 * @param[in] s 延时秒数
 */
void delay_s(uint32_t s)
{
    if (s == 0)
        return;

	while (s--)
		delay_ms(1000);
}
