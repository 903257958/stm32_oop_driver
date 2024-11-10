#include "main.h"

uint8_t gMID;										// 定义用于存放MID号的变量
uint16_t gDID;										// 定义用于存放DID号的变量
uint8_t gArrayWrite[] = {0x01, 0x02, 0x03, 0x04};	// 定义要写入数据的测试数组
uint8_t gArrayRead[4];								// 定义要读取数据的测试数组

LCDDev_t lcd;
W25QXDev_t w25q128 = {.info = {SPI1, GPIOC, GPIO_Pin_13}};

int main(void)
{
	lcd_init(&lcd);
	w25qx_init(&w25q128);
	
	lcd.clear(&lcd, BLACK);
	
	lcd.show_string(&lcd, 0, 0, "MID:     DID:", WHITE, BLACK, LCD_8X16, 0);
	lcd.show_string(&lcd, 0, 16, "W:", WHITE, BLACK, LCD_8X16, 0);
	lcd.show_string(&lcd, 0, 32, "R:", WHITE, BLACK, LCD_8X16, 0);
	
	/* 显示ID号 */
	w25q128.read_id(&w25q128, &gMID, &gDID);							// 获取W25Q128的ID号		
	lcd.show_num(&lcd, 32, 0, gMID, 3, WHITE, BLACK, LCD_8X16, 0);		// 显示MID
	lcd.show_num(&lcd, 104, 0, gDID, 5, WHITE, BLACK, LCD_8X16, 0);		// 显示DID	

	while (1)
	{
		/*W25Q64功能函数测试*/
		w25q128.sector_erase(&w25q128, 0x000000);						// 扇区擦除	
		//w25q128.page_program(&w25q128, 0x000000, gArrayWrite, 4);		// 将写入数据的测试数组写入到W25Q128中
		w25q128.write_data(&w25q128, 0x000000, gArrayWrite, 4);
		w25q128.read_data(&w25q128, 0x000000, gArrayRead, 4);			// 读取刚写入的测试数据到读取数据的测试数组中		
					
		/*显示数据*/
		lcd.show_num(&lcd, 16, 16, gArrayWrite[0], 2, WHITE, BLACK, LCD_8X16, 0);	// 显示写入数据的测试数组
		lcd.show_num(&lcd, 40, 16, gArrayWrite[1], 2, WHITE, BLACK, LCD_8X16, 0);
		lcd.show_num(&lcd, 64, 16, gArrayWrite[2], 2, WHITE, BLACK, LCD_8X16, 0);
		lcd.show_num(&lcd, 88, 16, gArrayWrite[3], 2, WHITE, BLACK, LCD_8X16, 0);
		
		delay_ms(500);
		
		lcd.show_num(&lcd, 16, 32, gArrayRead[0], 2, WHITE, BLACK, LCD_8X16, 0);	// 显示读取数据的测试数组
		lcd.show_num(&lcd, 40, 32, gArrayRead[1], 2, WHITE, BLACK, LCD_8X16, 0);
		lcd.show_num(&lcd, 64, 32, gArrayRead[2], 2, WHITE, BLACK, LCD_8X16, 0);
		lcd.show_num(&lcd, 88, 32, gArrayRead[3], 2, WHITE, BLACK, LCD_8X16, 0);
		
		delay_ms(500);
		
		gArrayWrite[0]++;
		gArrayWrite[1]++;
		gArrayWrite[2]++;
		gArrayWrite[3]++;
	}
}
