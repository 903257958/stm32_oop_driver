#include "main.h"

UARTDev_t uart1 = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
UARTDev_t uart2 = {.config = {USART2, 115200, GPIOA, GPIO_Pin_2, GPIOA, GPIO_Pin_3}};
UARTDev_t uart3 = {.config = {USART3, 115200, GPIOB, GPIO_Pin_10, GPIOB, GPIO_Pin_11}};
UARTDev_t uart4 = {.config = {UART4, 115200, GPIOC, GPIO_Pin_10, GPIOC, GPIO_Pin_11}};
UARTDev_t uart5 = {.config = {UART5, 115200, GPIOC, GPIO_Pin_12, GPIOD, GPIO_Pin_2}};
UARTDev_t uart6 = {.config = {USART6, 115200, GPIOC, GPIO_Pin_6, GPIOC, GPIO_Pin_7}};

char *uart1_rx_data;
char *uart2_rx_data;
char *uart3_rx_data;
char *uart4_rx_data;
char *uart5_rx_data;
char *uart6_rx_data;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	delay_init(72);
	
	uart_init(&uart1);
	uart_init(&uart2);
	uart_init(&uart3);
    uart_init(&uart4);
	uart_init(&uart5);
    uart_init(&uart6);
	
	/* 串口发送测试 */
	uart1.printf("\r\nThis is UART1!\r\n");
	uart2.printf("\r\nThis is UART2!\r\n");
	uart3.printf("\r\nThis is UART3!\r\n");
    uart4.printf("\r\nThis is UART4!\r\n");
	uart5.printf("\r\nThis is UART5!\r\n");
    uart6.printf("\r\nThis is UART6!\r\n");
	
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

		uart5_rx_data = uart5.recv();
		if (uart5_rx_data)
		{
			uart5.printf("UART5 received %d bytes: %s\r\n", strlen(uart5_rx_data), uart5_rx_data);
		}

		uart6_rx_data = uart6.recv();
		if (uart6_rx_data)
		{
			uart6.printf("UART6 received %d bytes: %s\r\n", strlen(uart6_rx_data), uart6_rx_data);
		}
	}
}
