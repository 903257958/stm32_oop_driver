#include "main.h"

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
        .rx_buf_size    = sizeof(uart2_tx_buf),
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
        .rx_buf_size    = sizeof(uart3_tx_buf),
        .rx_single_max  = 512
    }
};

static uint8_t uart4_tx_buf[2048];
static uint8_t uart4_rx_buf[2048];
uart_dev_t uart4 = {
    .config = {
        .uartx          = UART4,
        .baud           = 115200,
        .tx_port        = GPIOC,
        .tx_pin         = GPIO_Pin_10,
        .rx_port        = GPIOC,
        .rx_pin         = GPIO_Pin_11,
        .tx_buf         = uart4_tx_buf,
        .rx_buf         = uart4_rx_buf,
        .tx_buf_size    = sizeof(uart4_tx_buf),
        .rx_buf_size    = sizeof(uart4_tx_buf),
        .rx_single_max  = 512
    }
};

static uint8_t uart5_tx_buf[2048];
static uint8_t uart5_rx_buf[2048];
uart_dev_t uart5 = {
    .config = {
        .uartx          = UART5,
        .baud           = 115200,
        .tx_port        = GPIOC,
        .tx_pin         = GPIO_Pin_12,
        .rx_port        = GPIOD,
        .rx_pin         = GPIO_Pin_2,
        .tx_buf         = uart5_tx_buf,
        .rx_buf         = uart5_rx_buf,
        .tx_buf_size    = sizeof(uart5_tx_buf),
        .rx_buf_size    = sizeof(uart5_tx_buf),
        .rx_single_max  = 512
    }
};

static uint8_t uart6_tx_buf[2048];
static uint8_t uart6_rx_buf[2048];
uart_dev_t uart6 = {
    .config = {
        .uartx          = USART6,
        .baud           = 115200,
        .tx_port        = GPIOC,
        .tx_pin         = GPIO_Pin_6,
        .rx_port        = GPIOC,
        .rx_pin         = GPIO_Pin_7,
        .tx_buf         = uart6_tx_buf,
        .rx_buf         = uart6_rx_buf,
        .tx_buf_size    = sizeof(uart6_tx_buf),
        .rx_buf_size    = sizeof(uart6_tx_buf),
        .rx_single_max  = 512
    }
};

char *uart1_rx_data;
char *uart2_rx_data;
char *uart3_rx_data;
char *uart4_rx_data;
char *uart5_rx_data;
char *uart6_rx_data;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

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
