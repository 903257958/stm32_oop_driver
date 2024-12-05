#include "main.h"

LCDDev_t lcd = {.info = {
	SPI1,
	GPIOA, GPIO_Pin_9,
	GPIOA, GPIO_Pin_8,
	GPIOB, GPIO_Pin_5,
	GPIOC, GPIO_Pin_7,
	TIM3, 2,
	VERTICAL_REVERSE
}};

CST816TDev_t touch = {.info = {
	GPIOB, GPIO_Pin_6,
	GPIOB, GPIO_Pin_7,
	GPIOD, GPIO_Pin_1,
	VERTICAL_REVERSE
}};

uint16_t x = 0;
uint16_t y = 0;
uint8_t gesture = 0;

uint8_t id, fwVer;
uint32_t i = 0;

int main(void)
{
	lcd_init(&lcd);
	cst816t_init(&touch);
	
	lcd.clear(&lcd, YELLOW);

	touch.get_id(&touch, &id);
	lcd.show_string(&lcd, 20, 20, "ID:", BLACK, YELLOW, LCD_12X24, 0);
	lcd.show_hex_num(&lcd, 56, 20, id, 2, BLACK, YELLOW, LCD_12X24, 0);

	touch.get_firmware_ver(&touch, &fwVer);
	lcd.show_string(&lcd, 20, 50, "Ver:", BLACK, YELLOW, LCD_12X24, 0);
	lcd.show_hex_num(&lcd, 68, 50, fwVer, 2, BLACK, YELLOW, LCD_12X24, 0);

	lcd.show_string(&lcd, 20, 80, "Ges:", BLACK, YELLOW, LCD_12X24, 0);
	lcd.show_string(&lcd, 20, 110, "X:", BLACK, YELLOW, LCD_12X24, 0);
	lcd.show_string(&lcd, 20, 140, "Y:", BLACK, YELLOW, LCD_12X24, 0);

	while (1)
	{
		touch.get_action(&touch);
		lcd.show_hex_num(&lcd, 68, 80, touch.gesture, 2, BLACK, YELLOW, LCD_12X24, 0);
		lcd.show_num(&lcd, 44, 110, touch.x, 3, BLACK, YELLOW, LCD_12X24, 0);
		lcd.show_num(&lcd, 44, 140, touch.y, 3, BLACK, YELLOW, LCD_12X24, 0);

		lcd.show_num(&lcd, 44, 170, touch.get_finger_num(&touch), 3, BLACK, YELLOW, LCD_12X24, 0);
	}
}
