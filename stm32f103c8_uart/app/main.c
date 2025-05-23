#include "main.h"

UARTDev_t uart1 = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
UARTDev_t uart2 = {.config = {USART2, 115200, GPIOA, GPIO_Pin_2, GPIOA, GPIO_Pin_3}};
UARTDev_t uart3 = {.config = {USART3, 115200, GPIOB, GPIO_Pin_10, GPIOB, GPIO_Pin_11}};

char *uart1_rx_data;
char *uart2_rx_data;
char *uart3_rx_data;

uint8_t uart1_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};
uint8_t uart2_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};
uint8_t uart3_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};

/* 串口空闲中断+DMA测试 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	delay_init(72);
	
	uart_init(&uart1);
	uart_init(&uart2);
	uart_init(&uart3);
	
	/* 串口发送测试 */
	uart1.send(uart1_tx_data, 10);
	uart2.send(uart2_tx_data, 10);
	uart3.send(uart3_tx_data, 10);

	/* 串口打印测试 */
	uart1.printf("\r\nThis is UART1!\r\n");
	uart2.printf("\r\nThis is UART2!\r\n");
	uart3.printf("\r\nThis is UART3!\r\n");
	
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
	}
}
