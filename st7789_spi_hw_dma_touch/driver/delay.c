#include "stm32f4xx.h"                  // Device header

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD = nus * 21; 						//时间加载	  		 
	SysTick->VAL = 0x00;							//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;		//开始倒数 	 
	do
	{
		temp = SysTick->CTRL;
	}while((temp & 0x01) && !(temp & (1 << 16)));	//等待时间到达   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;		//关闭计数器
	SysTick->VAL = 0X00;							//清空计数器 
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void delay_ms(uint32_t xms)
{
	while(xms--)
	{
		delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void delay_s(uint32_t xs)
{
	while(xs--)
	{
		delay_ms(1000);
	}
} 
