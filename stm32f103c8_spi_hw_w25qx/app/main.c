#include "main.h"

uint8_t g_mid;
uint16_t g_did;
uint8_t g_write_data[256];
uint8_t g_read_data[256];

UARTDev_t debug = {.config = {USART1, 921600, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
W25QXDev_t w25q128 = {.config = {SPI2, GPIOB, GPIO_Pin_13, GPIOB, GPIO_Pin_14, GPIOB, GPIO_Pin_15, GPIOA, GPIO_Pin_15}};

int main(void)
{
	uint16_t i, j;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* 解除JTAG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	DBGMCU->CR &= ~((uint32_t)1 << 5);

    delay_init(72);
	uart_init(&debug);
	w25qx_init(&w25q128);

	debug.printf("data:%d\r\n", 10);
    
    /* 显示ID号 */
    w25q128.read_id(&w25q128, &g_mid, &g_did);								// 获取w25q64的ID号	
    debug.printf("\r\nmid: 0x%x, did: 0x%x\r\n", g_mid, g_did);
	
	/* W25Q64功能函数测试 */
	w25q128.block_erase_64kb(&w25q128, 0);	// 块擦除
	
	/* 每块256页，每页256字节，写入 256 * 256 = 65536 个字节 */
	for (i = 0; i < 256; i++) 
	{
		for (j = 0; j < 256; j++)
		{
			g_write_data[j] = i;
		}
		w25q128.page_write(&w25q128, i * 256, g_write_data, 256);
	}

	delay_ms(50);

	/* 读取 */
	for (i = 0; i < 256; i++) 
	{
		w25q128.read_data(&w25q128, i * 256, g_read_data, 256);
		for (j = 0; j < 256; j++)
		{
			debug.printf("addr = %d, data = %x\r\n", i * 256 + j, g_read_data[j]);
		}
	}
	
	while(1)
	{
		delay_ms(3000);
	}
}
