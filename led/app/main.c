#include "main.h"

LEDDev_t led = {.info = {GPIOC, GPIO_Pin_13, GPIO_LEVEL_LOW}};

int main(void)
{
	led_init(&led);
	
	while(1)
	{
		led.toggle(&led);
		delay_ms(500);
	}
}
