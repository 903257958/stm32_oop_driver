#include "main.h"

/* 硬件设备定义 */
static uint8_t uart1_tx_buf[2048];
static uint8_t uart1_rx_buf[2048];
uart_dev_t uart1 = {
    .config = {
        .uartx          = USART1,
        .baud           = 115200,
        .tx_port        = GPIOA,
        .tx_pin         = GPIO_Pin_9,
        .rx_port        = GPIOA,
        .rx_pin         = GPIO_Pin_10,
        .tx_buf         = uart1_tx_buf,
        .rx_buf         = uart1_rx_buf,
        .tx_buf_size    = sizeof(uart1_tx_buf),
        .rx_buf_size    = sizeof(uart1_tx_buf),
        .rx_single_max  = 512
    }
};

static uint8_t uart2_tx_buf[2048];
static uint8_t uart2_rx_buf[2048];
uart_dev_t uart2 = {
    .config = {
        .uartx          = USART2,
        .baud           = 115200,
        .tx_port        = GPIOA,
        .tx_pin         = GPIO_Pin_2,
        .rx_port        = GPIOA,
        .rx_pin         = GPIO_Pin_3,
        .tx_buf         = uart2_tx_buf,
        .rx_buf         = uart2_rx_buf,
        .tx_buf_size    = sizeof(uart2_tx_buf),
        .rx_buf_size    = sizeof(uart2_rx_buf),
        .rx_single_max  = 512
    }
};

static uint8_t uart3_tx_buf[2048];
static uint8_t uart3_rx_buf[2048];
uart_dev_t uart3 = {
    .config = {
        .uartx          = USART3,
        .baud           = 115200,
        .tx_port        = GPIOB,
        .tx_pin         = GPIO_Pin_10,
        .rx_port        = GPIOB,
        .rx_pin         = GPIO_Pin_11,
        .tx_buf         = uart3_tx_buf,
        .rx_buf         = uart3_rx_buf,
        .tx_buf_size    = sizeof(uart3_tx_buf),
        .rx_buf_size    = sizeof(uart3_rx_buf),
        .rx_single_max  = 512
    }
};

/* 串口发送和接收数据定义 */
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
	
	while (1)
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
