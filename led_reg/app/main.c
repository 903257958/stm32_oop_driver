#include "main.h"
#include "stm32f4xx.h"                  // Device header

int main(void)
{
	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	// GPIO_InitTypeDef GPIO_InitStructure;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	// GPIO_InitStructure.GPIO_Speed = GPIO_Medium_Speed;
	// GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC->AHB1ENR |= 1 << 2;				// 开启GPIOC时钟

	GPIOC->MODER &= ~(3 << (13 * 2)); 	// 清除Pin13的MODER值
	GPIOC->MODER |= 1 << (13 * 2); 		// Pin13设置为输出模式

	GPIOC->OTYPER &= ~(1 << 13);		// 清除Pin13的OTYPER位
	GPIOC->OTYPER |= (0 << 13);			// 设置Pin13为推挽输出

	GPIOC->OSPEEDR &= ~(3 << (13 * 2));	// 清除Pin 13的OSPEEDR位
	GPIOC->OSPEEDR |= 1 << (13 * 2);	// 设置Pin 13为中等速度

	GPIOC->PUPDR &= ~(3 << (13 * 2));  	// 清除Pin 13的PUPDR位
	GPIOC->PUPDR |= 0 << (13 * 2);     	// 设置Pin 13为无上拉/下拉
	
	while(1)
	{
		GPIOC->BSRRL |= (1 << 13);		// Pin13置位
		delay_ms(1000);

		GPIOC->BSRRH |= (1 << 13);		// Pin13复位
		delay_ms(1000);
	}
}
