#include "main.h"

//LCDDev_t lcd = {.info = {
//	SPI1,
//	GPIOA, GPIO_Pin_5,
//	GPIOA, GPIO_Pin_7,
//	GPIOA, GPIO_Pin_9,
//	GPIOA, GPIO_Pin_8,
//	GPIOB, GPIO_Pin_5,
//	GPIOC, GPIO_Pin_7,
//	TIM3, 2,
//	VERTICAL_REVERSE
//}};
LCDDev_t lcd = {.info = {
	SPI1,
	GPIOA, GPIO_Pin_5,
	GPIOA, GPIO_Pin_7,
	GPIOA, GPIO_Pin_6,
	GPIOB, GPIO_Pin_4,
	GPIOB, GPIO_Pin_5,
	GPIOA, GPIO_Pin_1,
	TIM2, 2,
	VERTICAL_REVERSE
}};

CST816TDev_t touch = {.info = {
	GPIOB, GPIO_Pin_6,
	GPIOB, GPIO_Pin_7,
	GPIOD, GPIO_Pin_1,
	VERTICAL_REVERSE
}};

uint8_t id, fw_ver;

int main(void)
{
	delay_init(168);
	lcd_init(&lcd);
	cst816t_init(&touch);
	
	lcd.clear(&lcd, BLACK);

	touch.get_id(&touch, &id);
	lcd.show_string(&lcd, 20, 20, "ID:", WHITE, BLACK, LCD_12X24, 0);
	lcd.show_hex_num(&lcd, 56, 20, id, 2, WHITE, BLACK, LCD_12X24, 0);

	touch.get_firmware_ver(&touch, &fw_ver);
	lcd.show_string(&lcd, 20, 50, "Ver:", WHITE, BLACK, LCD_12X24, 0);
	lcd.show_hex_num(&lcd, 68, 50, fw_ver, 2, WHITE, BLACK, LCD_12X24, 0);

	lcd.show_string(&lcd, 20, 80, "Ges:", WHITE, BLACK, LCD_12X24, 0);
	lcd.show_string(&lcd, 20, 110, "X:", WHITE, BLACK, LCD_12X24, 0);
	lcd.show_string(&lcd, 20, 140, "Y:", WHITE, BLACK, LCD_12X24, 0);

	while (1)
	{
		touch.get_action(&touch);
		lcd.show_hex_num(&lcd, 68, 80, touch.gesture, 2, WHITE, BLACK, LCD_12X24, 0);
		lcd.show_num(&lcd, 44, 110, touch.x, 3, WHITE, BLACK, LCD_12X24, 0);
		lcd.show_num(&lcd, 44, 140, touch.y, 3, WHITE, BLACK, LCD_12X24, 0);

		lcd.show_num(&lcd, 44, 170, touch.get_finger_num(&touch), 3, WHITE, BLACK, LCD_12X24, 0);
	}
}
