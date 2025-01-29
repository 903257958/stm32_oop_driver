#include "main.h"

UARTDev_t uart1 = {.info = {
    USART1, 115200,
    GPIOA, GPIO_Pin_9,
    GPIOA, GPIO_Pin_10
}};

# if 1
/* 串口空闲中断+DMA测试 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	delay_init(168);
	
	uart_dma_init(&uart1);
	
	/* 串口发送测试 */
	uart1.printf(&uart1, "\r\nThis is uart1!\r\n");
	
	while(1)
	{
		/* 串口接收测试 */
 		if(uart1.recv_string_flag(&uart1))
		{
			uart1.printf(&uart1, uart1.recv_string(&uart1));
			uart1.dma_recv_enable(&uart1);
		}
	}
}

#else
/* 串口中断测试 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	delay_init(168);

	uart_init(&uart1);

	/* 串口发送测试 */
	uart1.printf(&uart1, "\r\nThis is uart1!\r\n");
	
	while(1)
	{
		/* 串口接收测试 */
		if(uart1.recv_string_flag(&uart1))
		{
			uart1.printf(&uart1, uart1.recv_string(&uart1));
		}
	}
}

#endif
