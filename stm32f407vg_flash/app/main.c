#include "main.h"

UARTDev_t debug = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
FlashDev_t flash;

uint32_t g_write_buf[16];	// 64字节

int main(void)
{
	uint32_t i;
	uint32_t flash_read_addr;

	delay_init(168);
	uart_init(&debug);
	flash_init(&flash);

	for (i = 0; i < 16; i++)
	{
		g_write_buf[i] = 0x12345678;
	}

	/* 擦除，程序本身也存储在Flash中，尽量擦除靠后的页 */
	flash.sector_erase(&flash, 2);

	/* 写入Flash */
	flash.write(&flash, 0x08008000, g_write_buf, 64);

	/* 读取地址中的数据并打印 */
	flash_read_addr = 0x08008000;
	for (i = 0; i < 16; i++)
	{
		uint32_t val = *(uint32_t *)(flash_read_addr + i * 4);
		debug.printf("addr: %x, data: %x\r\n", flash_read_addr + i * 4, val);
	}
	
	while (1)
	{
		
	}
}
