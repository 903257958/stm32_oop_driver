#include "main.h"

OLEDDev_t oled = {.config = {GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7}};

int main(void)
{
    delay_init(72);
    oled_init(&oled);
    
    oled.show_string(&oled, 0, 0, "Hello", OLED_8X16);
    oled.update(&oled);
	
	while(1)
	{

	}
}
