#include "main.h"

static uint8_t uart1_tx_buf[256];
static uint8_t uart1_rx_buf[256];
uart_dev_t debug = {
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
        .rx_single_max  = 64
    }
};
flash_dev_t flash;

int main(void)
{
	uint32_t i;
	uint32_t flash_read_addr;
	uint32_t write_buf[16];		// 64字节

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	delay_init(72);
	uart_init(&debug);
	flash_init(&flash);

	for (i = 0; i < 16; i++)
	{
		write_buf[i] = 0x12345678;
	}

	/* 擦除，程序本身也存储在Flash中，尽量擦除靠后的页 */
	flash.page_erase(&flash, 60, 4);

	/* 写入Flash */
	flash.write(&flash, 60 * 1024 + 0x08000000, write_buf, 64);

	/* 读取地址中的数据并打印 */
	flash_read_addr = 0x08000000 + 60 * 1024;
	for (i = 0; i < 16; i++)
	{
		uint32_t val = *(uint32_t *)(flash_read_addr + i * 4);
		debug.printf("addr: %x, data: %x\r\n", flash_read_addr + i * 4, val);
	}
	
	while (1)
	{
		
	}
}
