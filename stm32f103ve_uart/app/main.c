#include "main.h"

uart_dev_t uart1 = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
uart_dev_t uart2 = {.config = {USART2, 115200, GPIOA, GPIO_Pin_2, GPIOA, GPIO_Pin_3}};
uart_dev_t uart3 = {.config = {USART3, 115200, GPIOB, GPIO_Pin_10, GPIOB, GPIO_Pin_11}};
uart_dev_t uart4 = {.config = {UART4, 115200, GPIOC, GPIO_Pin_10, GPIOC, GPIO_Pin_11}};

char *uart1_rx_data;
char *uart2_rx_data;
char *uart3_rx_data;
char *uart4_rx_data;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	delay_init(72);
	
	uart_init(&uart1);
	uart_init(&uart2);
	uart_init(&uart3);
    uart_init(&uart4);
	
	/* 串口发送测试 */
	uart1.printf("\r\nThis is UART1!\r\n");
	uart2.printf("\r\nThis is UART2!\r\n");
	uart3.printf("\r\nThis is UART3!\r\n");
    uart4.printf("\r\nThis is UART4!\r\n");
	
	while(1)
	{
		/* 串口接收测试 */
		uart1_rx_data = uart1.recv();
		if (uart1_rx_data)
		{
			uart1.printf("UART1 received %d bytes: %s\r\n", strlen(uart1_rx_data), uart1_rx_data);
		}

		uart2_rx_data = uart2.recv();
		if (uart2_rx_data)
		{
			uart2.printf("UART2 received %d bytes: %s\r\n", strlen(uart2_rx_data), uart2_rx_data);
		}

		uart3_rx_data = uart3.recv();
		if (uart3_rx_data)
		{
			uart3.printf("UART3 received %d bytes: %s\r\n", strlen(uart3_rx_data), uart3_rx_data);
		}
        
        uart4_rx_data = uart4.recv();
		if (uart4_rx_data)
		{
			uart4.printf("UART4 received %d bytes: %s\r\n", strlen(uart4_rx_data), uart4_rx_data);
		}
	}
}
