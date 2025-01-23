#include "main.h"

USARTDev_t usart1 = {.info = {
    USART1, 115200,
    GPIOA, GPIO_Pin_9,
    GPIOA, GPIO_Pin_10
}};

# if 1
/* 串口空闲中断+DMA测试 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	usart_dma_init(&usart1);
	
	/* 串口发送测试 */
	usart1.printf(&usart1, "\r\nThis is usart1!\r\n");
	
	while(1)
	{
		/* 串口接收测试 */
 		if(usart1.recv_string_flag(&usart1))
		{
			usart1.printf(&usart1, usart1.recv_string(&usart1));
			usart1.dma_recv_enable(&usart1);
		}
	}
}

#else
/* 串口中断测试 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	usart_init(&usart1);

	/* 串口发送测试 */
	usart1.printf(&usart1, "\r\nThis is usart1!\r\n");
	
	while(1)
	{
		/* 串口接收测试 */
		if(usart1.recv_string_flag(&usart1))
		{
			usart1.printf(&usart1, usart1.recv_string(&usart1));
		}
	}
}

#endif
