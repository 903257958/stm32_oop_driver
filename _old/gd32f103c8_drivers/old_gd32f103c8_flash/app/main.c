#include "main.h"

flash_dev_t flash;

int main(void)
{
    uint32_t i;
	uint32_t flash_read_addr;
	uint32_t write_buf[16];		// 64字节

	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

	uart0_init(921600);
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
		uart0_printf("addr: %x, data: %x\r\n", flash_read_addr + i * 4, val);
	}

	while(1)
	{
		
	}
}
