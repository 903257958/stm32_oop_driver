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
w25qx_dev_t w25q128 = {.config = {SPI2, GPIOB, GPIO_Pin_13, GPIOB, GPIO_Pin_14, GPIOB, GPIO_Pin_15, GPIOA, GPIO_Pin_15}};

int main(void)
{
	uint16_t i, j;
	uint8_t mid;
	uint16_t did;
	uint8_t write_data[256];
	uint8_t read_data[256];

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* 解除JTAG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	DBGMCU->CR &= ~((uint32_t)1 << 5);

	uart_init(&debug);
	w25qx_init(&w25q128);

	debug.printf("data:%d\r\n", 10);
    
    /* 显示ID号 */
    w25q128.read_id(&w25q128, &mid, &did);	// 获取w25q64的ID号	
    debug.printf("\r\nmid: 0x%x, did: 0x%x\r\n", mid, did);
	
	/* W25Q64功能函数测试 */
	w25q128.block_erase_64kb(&w25q128, 0);	// 块擦除
	
	/* 每块256页，每页256字节，写入 256 * 256 = 65536 个字节 */
	for (i = 0; i < 256; i++) 
	{
		for (j = 0; j < 256; j++)
		{
			write_data[j] = i;
		}
		w25q128.page_write(&w25q128, i * 256, write_data, 256);
	}

	delay_ms(50);

	/* 读取 */
	for (i = 0; i < 256; i++) 
	{
		w25q128.read_data(&w25q128, i * 256, read_data, 256);
		for (j = 0; j < 256; j++)
		{
			debug.printf("addr = %d, data = %x\r\n", i * 256 + j, read_data[j]);
		}
	}
	
	while (1)
	{
		
	}
}
