#include "drv_delay.h"
#include "drv_uart.h"
#include <stddef.h>
#include <string.h>

static uart_dev_t uart1;
static uint8_t uart1_tx_buf[512];
static uint8_t uart1_rx_buf[512];
static const uart_cfg_t uart1_cfg = {
    .uart_periph     = USART1,
    .baudrate        = 115200,
    .tx_port         = GPIOA,
    .tx_pin          = GPIO_Pin_9,
    .rx_port         = GPIOA,
    .rx_pin          = GPIO_Pin_10,
    .tx_buf          = uart1_tx_buf,
    .rx_buf          = uart1_rx_buf,
    .tx_buf_size     = sizeof(uart1_tx_buf),
    .rx_buf_size     = sizeof(uart1_rx_buf),
    .rx_single_max   = 256,
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static uart_dev_t uart2;
static uint8_t uart2_tx_buf[512];
static uint8_t uart2_rx_buf[512];
static const uart_cfg_t uart2_cfg = {
    .uart_periph     = USART2,
    .baudrate        = 115200,
    .tx_port         = GPIOA,
    .tx_pin          = GPIO_Pin_2,
    .rx_port         = GPIOA,
    .rx_pin          = GPIO_Pin_3,
    .tx_buf          = uart2_tx_buf,
    .rx_buf          = uart2_rx_buf,
    .tx_buf_size     = sizeof(uart2_tx_buf),
    .rx_buf_size     = sizeof(uart2_rx_buf),
    .rx_single_max   = 256,
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static uart_dev_t uart3;
static uint8_t uart3_tx_buf[512];
static uint8_t uart3_rx_buf[512];
static const uart_cfg_t uart3_cfg = {
    .uart_periph     = USART3,
    .baudrate        = 115200,
    .tx_port         = GPIOB,
    .tx_pin          = GPIO_Pin_10,
    .rx_port         = GPIOB,
    .rx_pin          = GPIO_Pin_11,
    .tx_buf          = uart3_tx_buf,
    .rx_buf          = uart3_rx_buf,
    .tx_buf_size     = sizeof(uart3_tx_buf),
    .rx_buf_size     = sizeof(uart3_rx_buf),
    .rx_single_max   = 256,
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static uart_dev_t uart4;
static uint8_t uart4_tx_buf[512];
static uint8_t uart4_rx_buf[512];
static const uart_cfg_t uart4_cfg = {
    .uart_periph     = UART4,
    .baudrate        = 115200,
    .tx_port         = GPIOC,
    .tx_pin          = GPIO_Pin_10,
    .rx_port         = GPIOC,
    .rx_pin          = GPIO_Pin_11,
    .tx_buf          = uart4_tx_buf,
    .rx_buf          = uart4_rx_buf,
    .tx_buf_size     = sizeof(uart4_tx_buf),
    .rx_buf_size     = sizeof(uart4_rx_buf),
    .rx_single_max   = 256,
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static uint8_t uart1_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};
static uint8_t uart2_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};
static uint8_t uart3_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};
static uint8_t uart4_tx_data[10] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A};

/* vprintf 测试，用于留给外部封装可变参数的 printf */
static void test_uart_printf(uart_dev_t *dev, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    if (dev && dev->ops && dev->ops->vprintf)
        dev->ops->vprintf(dev, fmt, args);

    va_end(args);
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_uart_init(&uart1, &uart1_cfg);
    drv_uart_init(&uart2, &uart2_cfg);
    drv_uart_init(&uart3, &uart3_cfg);
    drv_uart_init(&uart4, &uart4_cfg);

	/* 串口发送测试 */
	uart1.ops->send_data(&uart1, uart1_tx_data, 10);
	uart2.ops->send_data(&uart2, uart2_tx_data, 10);
	uart3.ops->send_data(&uart3, uart3_tx_data, 10);
	uart4.ops->send_data(&uart4, uart4_tx_data, 10);

	/* 串口打印测试 */
    test_uart_printf(&uart1, "\r\nUART1 vprintf test: %d %s\r\n", 123, "abc");
    test_uart_printf(&uart2, "\r\nUART2 vprintf test: %d %s\r\n", 123, "abc");
    test_uart_printf(&uart3, "\r\nUART3 vprintf test: %d %s\r\n", 123, "abc");
    test_uart_printf(&uart4, "\r\nUART4 vprintf test: %d %s\r\n", 123, "abc");

	uart1.ops->printf(&uart1, "\r\nThis is UART1!\r\n");
	uart2.ops->printf(&uart2, "\r\nThis is UART2!\r\n");
	uart3.ops->printf(&uart3, "\r\nThis is UART3!\r\n");
	uart4.ops->printf(&uart4, "\r\nThis is UART4!\r\n");
	
#if 1
    /* 串口空闲中断 + DMA 接收字符串测试，接收缓冲区需由用户分配 */
    char uart1_rx_data[128];
    char uart2_rx_data[128];
    char uart3_rx_data[128];
    char uart4_rx_data[128];

	while (1) {
		if (uart1.ops->recv_str(&uart1, uart1_rx_data) == 0)
			uart1.ops->printf(&uart1, "UART1 recv %d bytes: %s\r\n", strlen(uart1_rx_data), uart1_rx_data);

		if (uart2.ops->recv_str(&uart2, uart2_rx_data) == 0)
			uart2.ops->printf(&uart2, "UART2 recv %d bytes: %s\r\n", strlen(uart2_rx_data), uart2_rx_data);

		if (uart3.ops->recv_str(&uart3, uart3_rx_data) == 0)
			uart3.ops->printf(&uart3, "UART3 recv %d bytes: %s\r\n", strlen(uart3_rx_data), uart3_rx_data);
        
        if (uart4.ops->recv_str(&uart4, uart4_rx_data) == 0)
			uart4.ops->printf(&uart4, "UART4 recv %d bytes: %s\r\n", strlen(uart4_rx_data), uart4_rx_data);
	}
#else
    /* 串口空闲中断 + DMA 接收数据测试，接收缓冲区无需由用户分配 */
    uint8_t *recv_data;
    uint32_t recv_data_len;
    uint32_t i;

    while (1) {
        if (uart1.ops->recv_data(&uart1, &recv_data, &recv_data_len) == 0) {
            uart1.ops->printf(&uart1, "UART1 recv_data %u bytes: ", recv_data_len);
            for (i = 0; i < recv_data_len; i++)
                uart1.ops->printf(&uart1, "%02X ", recv_data[i]);
            uart1.ops->printf(&uart1, "\r\n");
        }

        if (uart2.ops->recv_data(&uart2, &recv_data, &recv_data_len) == 0) {
            uart2.ops->printf(&uart2, "UART2 recv_data %u bytes: ", recv_data_len);
            for (i = 0; i < recv_data_len; i++)
                uart2.ops->printf(&uart2, "%02X ", recv_data[i]);
            uart2.ops->printf(&uart2, "\r\n");
        }

        if (uart3.ops->recv_data(&uart3, &recv_data, &recv_data_len) == 0) {
            uart3.ops->printf(&uart3, "UART3 recv_data %u bytes: ", recv_data_len);
            for (i = 0; i < recv_data_len; i++)
                uart3.ops->printf(&uart3, "%02X ", recv_data[i]);
            uart3.ops->printf(&uart3, "\r\n");
        }

        if (uart4.ops->recv_data(&uart4, &recv_data, &recv_data_len) == 0) {
            uart4.ops->printf(&uart4, "UART4 recv_data %u bytes: ", recv_data_len);
            for (i = 0; i < recv_data_len; i++)
                uart4.ops->printf(&uart4, "%02X ", recv_data[i]);
            uart4.ops->printf(&uart4, "\r\n");
        }
	}
#endif
}
