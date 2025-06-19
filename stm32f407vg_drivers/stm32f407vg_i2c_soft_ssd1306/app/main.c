#include "main.h"

oled_dev_t oled = {.config = {GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7}};

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	oled_init(&oled);

	oled.show_char(&oled, 0, 0, 'A', OLED_8X16);
	oled.show_string(&oled, 16, 0, "Hello World!", OLED_8X16);
	oled.show_char(&oled, 0, 18, 'A', OLED_6X8);
	oled.show_string(&oled, 16, 18, "Hello World!", OLED_6X8);
	oled.show_num(&oled, 0, 28, 12345, 5, OLED_6X8);
	oled.show_signed_num(&oled, 40, 28, -66, 2, OLED_6X8);
	oled.show_hex_num(&oled, 70, 28, 0xA5A5, 4, OLED_6X8);
	oled.show_bin_num(&oled, 0, 38, 0xA5, 8, OLED_6X8);
	oled.show_float_num(&oled, 60, 38, 123.45, 3, 2, OLED_6X8);
	oled.show_chinese(&oled, 0, 48, "你好，世界。");
	oled.show_image(&oled, 96, 48, 16, 16, diode);
	oled.printf(&oled, 96, 18, OLED_6X8, "[%02d]", 6);
	oled.update(&oled);
	
	while (1)
	{
		
	}
}
