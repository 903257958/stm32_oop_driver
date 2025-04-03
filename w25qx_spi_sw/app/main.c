#include "main.h"

uint8_t g_mid;										// 定义用于存放MID号的变量
uint16_t g_did;										// 定义用于存放DID号的变量
uint8_t g_array_write[] = {0x01, 0x02, 0x03, 0x04};	// 定义要写入数据的测试数组
uint8_t g_array_read[4];							// 定义要读取数据的测试数组

UARTDev_t debug = {.config = {
    USART1, 115200,
    GPIOA, GPIO_Pin_9,
    GPIOA, GPIO_Pin_10
}};

W25QXDev_t w25q128 = {.config = {
	GPIOA, GPIO_Pin_5, 
	GPIOA, GPIO_Pin_6, 
	GPIOA, GPIO_Pin_7, 
	GPIOC, GPIO_Pin_13
}};

int main(void)
{
	delay_init(168);
	uart_init(&debug);
	w25qx_init(&w25q128);
		
	while (1)
	{
        /* 显示ID号 */
        w25q128.read_id(&w25q128, &g_mid, &g_did);								// 获取W25Q128的ID号	
        debug.printf(&debug, "\r\nmid: 0x%x, did: 0x%x\r\n", g_mid, g_did);
        
		/* W25Q64功能函数测试 */
		w25q128.sector_erase(&w25q128, 0x000000);						// 扇区擦除	
		//w25q128.page_program(&w25q128, 0x000000, g_array_write, 4);	// 将写入数据的测试数组写入到W25Q128中
		w25q128.write_data(&w25q128, 0x000000, g_array_write, 4);
		w25q128.read_data(&w25q128, 0x000000, g_array_read, 4);			// 读取刚写入的测试数据到读取数据的测试数组中		
					
		/* 显示数据 */
		debug.printf(&debug, "write: 0x%x, 0x%x, 0x%x, 0x%x\r\n", g_array_write[0], g_array_write[1], g_array_write[2], g_array_write[3]);
		delay_ms(500);
		
		debug.printf(&debug, "read: 0x%x, 0x%x, 0x%x, 0x%x\r\n", g_array_read[0], g_array_read[1], g_array_read[2], g_array_read[3]);
		delay_ms(500);
		
		g_array_write[0]++;
		g_array_write[1]++;
		g_array_write[2]++;
		g_array_write[3]++;
	}
}
