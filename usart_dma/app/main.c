#include "main.h"

//USARTDev_t usart1 = {.info = {USART1, 115200}};
USARTDev_t usart2 = {.info = {USART2, 115200}};
//USARTDev_t usart3 = {.info = {USART3, 115200}};
//USARTDev_t uart4 = {.info = {UART4, 115200}};
//USARTDev_t uart5 = {.info = {UART5, 115200}};
//USARTDev_t usart6 = {.info = {USART6, 115200}};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//usart_init(&usart1);
	//usart_dma_init(&usart1);
	
	//usart_init(&usart2);
	usart_dma_init(&usart2);
	
	//usart_init(&usart3);
	//usart_dma_init(&usart3);
	
	//usart_init(&uart4);
	//usart_dma_init(&uart4);
	
	//usart_init(&uart5);
	//usart_dma_init(&uart4);
	
	//usart_init(&usart6);
	//usart_dma_init(&usart6);
	
	/*串口发送测试*/
	//usart1.printf(&usart1, "\r\nThis is usart1!\r\n");
	usart2.printf(&usart2, "\r\nThis is usart2!\r\n");
	//usart3.printf(&usart3, "\r\nThis is usart3!\r\n");
	//uart4.printf(&uart4, "\r\nThis is uart4!\r\n");
	//uart5.printf(&uart5, "\r\nThis is uart5!\r\n");
	//usart6.printf(&usart6, "\r\nThis is usart6!\r\n");
	
	while(1)
	{
		/* 串口接收测试 */
//		if(usart1.recv_string_flag(&usart1))
//		{
//			usart1.printf(&usart1, usart1.recv_string(&usart1));
//		}
		//usart2.printf(&usart2, UART2_DMA_Rx_Buff);
		if(usart2.recv_string_flag(&usart2))
		{
			usart2.printf(&usart2, usart2.recv_string(&usart2));
			usart2.dma_recv_enable(&usart2);
		}
//		if(usart3.recv_string_flag(&usart3))
//		{
//			usart3.printf(&usart3, usart3.recv_string(&usart3));
//		}
//		if(uart4.recv_string_flag(&uart4))
//		{
//			uart4.printf(&uart4, uart4.recv_string(&uart4));
//		}
//		if(uart5.recv_string_flag(&uart5))
//		{
//			uart5.printf(&uart5, uart5.recv_string(&uart5));
//		}
//		if(usart6.recv_string_flag(&usart6))
//		{
//			usart6.printf(&usart6, usart6.recv_string(&usart6));
//		}
	}
}
