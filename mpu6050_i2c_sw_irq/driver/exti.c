#include <stdlib.h>
#include "exti.h"

#define MAX_IRQ_HANDLER_NUM	16

IRQHandler_t gIRQHandlers[MAX_IRQ_HANDLER_NUM];

uint8_t gIRQHandlerCount = 0;

/******************************************************************************
 * @brief	中断处理结构体注册函数
 * @param	EXTILine		:  中断通道
 * @param	irq_callback	:  中断回调函数
 * @return	无
 ******************************************************************************/
void irq_handler_register(uint32_t EXTILine, void(*irq_callback)(void))
{
    if (gIRQHandlerCount < MAX_IRQ_HANDLER_NUM)
    {
        gIRQHandlers[gIRQHandlerCount].EXTILine = EXTILine;
        gIRQHandlers[gIRQHandlerCount].irq_callback = irq_callback;
        gIRQHandlerCount++;
    }
}

/******************************************************************************
 * @brief	通用中断处理函数
 * @param	EXTILine	:  中断通道
 * @return	无
 ******************************************************************************/
static void __exti_irq_handler(uint32_t EXTILine)
{
	for (uint8_t i = 0; i < gIRQHandlerCount; i++)		// 遍历所有已注册的中断处理结构体
	{
		if (gIRQHandlers[i].EXTILine == EXTILine && EXTI_GetITStatus(EXTILine) != RESET)
		{
			EXTI_ClearITPendingBit(EXTILine);			// 清除中断标志位
			if (gIRQHandlers[i].irq_callback != NULL)
			{
				gIRQHandlers[i].irq_callback();			// 执行中断回调函数
			}
		}
	}
}

void EXTI0_IRQHandler(void)
{
	__exti_irq_handler(EXTI_Line0);
}

void EXTI1_IRQHandler(void)
{
	__exti_irq_handler(EXTI_Line1);
}

void EXTI2_IRQHandler(void)
{
	__exti_irq_handler(EXTI_Line2);
}

void EXTI3_IRQHandler(void)
{
	__exti_irq_handler(EXTI_Line3);
}

void EXTI4_IRQHandler(void)
{
	__exti_irq_handler(EXTI_Line4);
}

void EXTI9_5_IRQHandler(void)
{
	__exti_irq_handler(EXTI_Line5);
	__exti_irq_handler(EXTI_Line6);
	__exti_irq_handler(EXTI_Line7);
	__exti_irq_handler(EXTI_Line8);
	__exti_irq_handler(EXTI_Line9);
}

void EXTI15_10_IRQHandler(void)
{
	__exti_irq_handler(EXTI_Line10);
	__exti_irq_handler(EXTI_Line11);
	__exti_irq_handler(EXTI_Line12);
	__exti_irq_handler(EXTI_Line13);
	__exti_irq_handler(EXTI_Line14);
	__exti_irq_handler(EXTI_Line15);
}
