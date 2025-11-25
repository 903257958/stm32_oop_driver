#include "drv_delay.h"
#include "drv_uart.h"
#include <stddef.h>
#include <string.h>

static uart_dev_t uart1;
static uint8_t uart1_tx_buf[256] = { NULL };
static uint8_t uart1_rx_buf[256] = { NULL };
static const uart_cfg_t uart1_cfg = {
    .uart_periph     = USART1,
    .baud            = 115200,
    .tx_port         = GPIOA,
    .tx_pin          = GPIO_Pin_9,
    .rx_port         = GPIOA,
    .rx_pin          = GPIO_Pin_10,
    .tx_buf          = uart1_tx_buf,
    .rx_buf          = uart1_rx_buf,
    .tx_buf_size     = sizeof(uart1_tx_buf),
    .rx_buf_size     = sizeof(uart1_rx_buf),
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static uart_dev_t uart2;
static uint8_t uart2_tx_buf[256];
static uint8_t uart2_rx_buf[256];
static const uart_cfg_t uart2_cfg = {
    .uart_periph     = USART2,
    .baud            = 115200,
    .tx_port         = GPIOA,
    .tx_pin          = GPIO_Pin_2,
    .rx_port         = GPIOA,
    .rx_pin          = GPIO_Pin_3,
    .tx_buf          = uart2_tx_buf,
    .rx_buf          = uart2_rx_buf,
    .tx_buf_size     = sizeof(uart2_tx_buf),
    .rx_buf_size     = sizeof(uart2_rx_buf),
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static uart_dev_t uart3;
static uint8_t uart3_tx_buf[256];
static uint8_t uart3_rx_buf[256];
static const uart_cfg_t uart3_cfg = {
    .uart_periph     = USART3,
    .baud            = 115200,
    .tx_port         = GPIOB,
    .tx_pin          = GPIO_Pin_10,
    .rx_port         = GPIOB,
    .rx_pin          = GPIO_Pin_11,
    .tx_buf          = uart3_tx_buf,
    .rx_buf          = uart3_rx_buf,
    .tx_buf_size     = sizeof(uart3_tx_buf),
    .rx_buf_size     = sizeof(uart3_rx_buf),
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static uart_dev_t uart4;
static uint8_t uart4_tx_buf[256];
static uint8_t uart4_rx_buf[256];
static const uart_cfg_t uart4_cfg = {
    .uart_periph     = UART4,
    .baud            = 115200,
    .tx_port         = GPIOC,
    .tx_pin          = GPIO_Pin_10,
    .rx_port         = GPIOC,
    .rx_pin          = GPIO_Pin_11,
    .tx_buf          = uart4_tx_buf,
    .rx_buf          = uart4_rx_buf,
    .tx_buf_size     = sizeof(uart4_tx_buf),
    .rx_buf_size     = sizeof(uart4_rx_buf),
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static char *uart1_rx_data;
static char *uart2_rx_data;
static char *uart3_rx_data;
static char *uart4_rx_data;

static uint8_t uart1_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};
static uint8_t uart2_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};
static uint8_t uart3_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};
static uint8_t uart4_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_uart_init(&uart1, &uart1_cfg);
    drv_uart_init(&uart2, &uart2_cfg);
    drv_uart_init(&uart3, &uart3_cfg);
    drv_uart_init(&uart4, &uart4_cfg);

	/* 串口发送测试 */
	uart1.ops->send(uart1_tx_data, 10);
	uart2.ops->send(uart2_tx_data, 10);
	uart3.ops->send(uart3_tx_data, 10);
	uart4.ops->send(uart4_tx_data, 10);

	/* 串口打印测试 */
	uart1.ops->printf("\r\nThis is UART1!\r\n");
	uart2.ops->printf("\r\nThis is UART2!\r\n");
	uart3.ops->printf("\r\nThis is UART3!\r\n");
	uart4.ops->printf("\r\nThis is UART4!\r\n");
	
	while (1) {
        /* 串口空闲中断 + DMA 接收测试 */
		uart1_rx_data = uart1.ops->recv();
		if (uart1_rx_data)
			uart1.ops->printf("UART1 recv %d bytes: %s\r\n", strlen(uart1_rx_data), uart1_rx_data);

		uart2_rx_data = uart2.ops->recv();
		if (uart2_rx_data)
			uart2.ops->printf("UART2 recv %d bytes: %s\r\n", strlen(uart2_rx_data), uart2_rx_data);

		uart3_rx_data = uart3.ops->recv();
		if (uart3_rx_data)
			uart3.ops->printf("UART3 recv %d bytes: %s\r\n", strlen(uart3_rx_data), uart3_rx_data);

        uart4_rx_data = uart4.ops->recv();
		if (uart4_rx_data)
			uart4.ops->printf("UART4 recv %d bytes: %s\r\n", strlen(uart4_rx_data), uart4_rx_data);
	}
}
