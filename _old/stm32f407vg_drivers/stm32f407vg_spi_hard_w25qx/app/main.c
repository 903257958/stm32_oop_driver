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

w25qx_dev_t w25q128 = {
	.config = {SPI1, GPIOA, GPIO_Pin_5, GPIOA, GPIO_Pin_6, GPIOA, GPIO_Pin_7, GPIOC, GPIO_Pin_13}
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

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	uart_init(&debug);
	w25qx_init(&w25q128);
    
    /* 显示ID号 */
    w25q128.read_id(&w25q128, &mid, &did);	// 获取w25q64的ID号	
    debug.printf("\r\nmid: 0x%x, did: 0x%x\r\n", mid, did);
	
	/* W25Q64功能函数测试 */
	w25q128.block_erase_64kb(&w25q128, 0);	// 擦除前64KB（第1个block）
	debug.printf("Block Erased\r\n");
	delay_ms(500);
	
	/* 每块256页，每页256字节，写入 256 * 256 = 65536 个字节 */
	for (i = 0; i < 256; i++) 
	{
		for (j = 0; j < 256; j++)
		{
			write_buf[j] = (i << 4) + (j & 0x0F);
		}
		w25q128.page_write(&w25q128, i * 256, write_buf, 256);
	}
	debug.printf("Write done\r\n");
	delay_ms(50);

	/* 读取验证 */
	for (i = 0; i < 256; i++)
	{
		w25q128.read_data(&w25q128, i * 256, read_buf, 256);

		for (j = 0; j < 256; j++)
		{
			expected_byte = (i << 4) + (j & 0x0F);
			if (read_buf[j] != expected_byte)
			{
				error_count++;
				debug.printf("[Error] Addr: %u, Read: 0x%02X, Expect: 0x%02X\r\n",
				             i * 256 + j, read_buf[j], expected_byte);
			}
		}
	}
	debug.printf("Read and verify done, errors: %d\r\n", error_count);
	
	while (1)
	{
		
	}
}
