#include "main.h"

UARTDev_t debug = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
FMCDev_t stm32_fmc;

uint32_t g_write_buf[1024];	// 4096字节

int main(void)
{
	uint32_t i;
	uint32_t flash_read_addr;

	delay_init(72);
	uart_init(&debug);
	fmc_init(&stm32_fmc);

	for (i = 0; i < 1024; i++)
	{
		g_write_buf[i] = 0x12345678;
	}

	/* 擦除，程序本身也存储在Flash中，尽量擦除靠后的页 */
	stm32_fmc.page_erase(&stm32_fmc, 60, 4);

	/* 写入Flash */
	stm32_fmc.write(&stm32_fmc, 60 * 1024 + 0x08000000, g_write_buf, 4096);

	/* 读取地址中的数据并打印 */
	flash_read_addr = 0x08000000 + 60 * 1024;
	for (i = 0; i < 1024; i++)
	{
		uint32_t val = *(uint32_t *)(flash_read_addr + i * 4);
		debug.printf("addr: %x, data: %x\r\n", flash_read_addr + i * 4, val);
	}
	
	while(1)
	{
		
	}
}
