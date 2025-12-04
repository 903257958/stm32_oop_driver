#include "drv_delay.h"
#include <stddef.h>

#if defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || defined (STM32F10X_MD) || defined (STM32F10X_MD_VL) || \
    defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_XL) || defined (STM32F10X_CL) 
#include "stm32f10x.h"
	
#elif defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F429_439xx) || \
      defined(STM32F401xx) || defined(STM32F410xx) || defined(STM32F411xE) || defined(STM32F412xG) || \
      defined(STM32F413_423xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#include "stm32f4xx.h"
    
#elif defined (GD32F10X_MD) || defined (GD32F10X_HD) || defined (GD32F10X_XD) || defined (GD32F10X_CL)
#include "gd32f10x.h"

#else
#error drv_delay.h: No processor defined!

#endif

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
