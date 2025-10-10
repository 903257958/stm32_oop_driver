#include "bsp.h"
#include <string.h>

char *uart0_rx_data;
char *uart1_rx_data;
char *uart2_rx_data;

uint8_t uart0_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};
uint8_t uart1_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};
uint8_t uart2_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};

/* 串口空闲中断+DMA测试 */
int main(void)
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    bsp_init();

	/* 串口发送测试 */
	uart0.send(uart0_tx_data, 10);
	uart1.send(uart1_tx_data, 10);
	uart2.send(uart2_tx_data, 10);

	/* 串口打印测试 */
	uart0.printf("\r\nThis is UART0!\r\n");
	uart1.printf("\r\nThis is UART1!\r\n");
	uart2.printf("\r\nThis is UART2!\r\n");
	
	while (1) {
		/* 串口接收测试 */
        uart0_rx_data = uart0.recv();
		if (uart0_rx_data)
			uart0.printf("UART0 received %d bytes: %s\r\n", strlen(uart0_rx_data), uart0_rx_data);
        
		uart1_rx_data = uart1.recv();
		if (uart1_rx_data)
			uart1.printf("UART1 received %d bytes: %s\r\n", strlen(uart1_rx_data), uart1_rx_data);

		uart2_rx_data = uart2.recv();
		if (uart2_rx_data)
			uart2.printf("UART2 received %d bytes: %s\r\n", strlen(uart2_rx_data), uart2_rx_data);
	}
}
