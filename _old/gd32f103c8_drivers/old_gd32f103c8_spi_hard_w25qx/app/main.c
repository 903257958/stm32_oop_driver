#include "main.h"

w25qx_dev_t w25q64 = {
	.config = {SPI0, GPIOA, GPIO_PIN_5, GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7, GPIOA, GPIO_PIN_4}
};

int main(void)
{
	uint16_t i, j;
	uint8_t mid;
	uint16_t did;
	uint8_t write_buf[256];
	uint8_t read_buf[256];
	uint8_t expected_byte;
	uint32_t error_count = 0;

	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

	uart0_init(115200);
	w25qx_init(&w25q64);
    
    /* 显示ID号 */
    w25q64.read_id(&w25q64, &mid, &did);	// 获取w25q64的ID号	
    uart0_printf("\r\nmid: 0x%x, did: 0x%x\r\n", mid, did);
	
	/* W25Q64功能函数测试 */
	w25q64.block_erase_64kb(&w25q64, 0);	// 擦除前64KB（第1个block）
	uart0_printf("Block Erased\r\n");
	delay_ms(500);
    
	/* 每块256页，每页256字节，写入 256 * 256 = 65536 个字节 */
	for (i = 0; i < 256; i++) 
	{
		for (j = 0; j < 256; j++)
		{
			write_buf[j] = i;
		}
		w25q64.page_write(&w25q64, i * 256, write_buf, 256);
	}
	uart0_printf("Write done\r\n");
	delay_ms(50);

	/* 读取验证 */
	for (i = 0; i < 256; i++)
	{
		w25q64.read_data(&w25q64, i * 256, read_buf, 256);

		for (j = 0; j < 256; j++)
		{
			expected_byte = i;
			if (read_buf[j] != expected_byte)
			{
				error_count++;
				uart0_printf("[Error] Addr: %u, Read: 0x%02X, Expect: 0x%02X\r\n",
				             i * 256 + j, read_buf[j], expected_byte);
			}
		}
	}
	uart0_printf("Read and verify done, errors: %d\r\n", error_count);
	
	while (1)
	{
		
	}
}
