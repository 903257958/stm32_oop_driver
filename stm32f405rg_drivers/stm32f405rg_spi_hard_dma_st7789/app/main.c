#include "main.h"

lcd_dev_t lcd = {.config = {
	SPI1,
	GPIOA, GPIO_Pin_5,
	GPIOA, GPIO_Pin_7,
	GPIOC, GPIO_Pin_5,
	GPIOA, GPIO_Pin_4,
	GPIOC, GPIO_Pin_4,
	GPIOA, GPIO_Pin_1,
	TIM2, 2,
	VERTICAL_FORWARD
}};

cst816t_dev_t touch = {.config = {
	GPIOB, GPIO_Pin_0,
	GPIOB, GPIO_Pin_1,
    GPIOB, GPIO_Pin_11,
	VERTICAL_FORWARD
}};

int main(void)
{
	uint8_t id, fw_ver;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

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
		touch.get_finger_num(&touch);

		lcd.show_hex_num(&lcd, 68, 80, touch.data.gesture, 2, WHITE, BLACK, LCD_12X24, 0);
		lcd.show_num(&lcd, 44, 110, touch.data.x, 3, WHITE, BLACK, LCD_12X24, 0);
		lcd.show_num(&lcd, 44, 140, touch.data.y, 3, WHITE, BLACK, LCD_12X24, 0);
		lcd.show_num(&lcd, 44, 170, touch.data.finger_num, 3, WHITE, BLACK, LCD_12X24, 0);
	}
}
