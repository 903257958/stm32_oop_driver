#include "main.h"

LEDDev_t led = {.info = {GPIOB, GPIO_Pin_2, GPIO_LEVEL_LOW}};

int main(void)
{
	led_init(&led);
	
	while(1)
	{
		led.toggle(&led);
		delay_ms(500);
	}
}
