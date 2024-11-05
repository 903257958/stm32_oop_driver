#include "main.h"

LEDDev_t led1 = {.info = {GPIOB, GPIO_Pin_2, GPIO_LEVEL_LOW}};
LEDDev_t led2 = {.info = {GPIOB, GPIO_Pin_1, GPIO_LEVEL_LOW}};

KeyDev_t key1 = {.info = {GPIOA, GPIO_Pin_1, GPIO_LEVEL_LOW}};
KeyDev_t key2 = {.info = {GPIOA, GPIO_Pin_4, GPIO_LEVEL_LOW}};

int main(void)
{
	led_init(&led1);
	led_init(&led2);
	
	key_init(&key1);
	key_init(&key2);
	
	while(1)
	{
		if(key1.is_press(&key1))
		{
			led1.toggle(&led1);
		}
		if(key2.is_press(&key2))
		{
			led1.toggle(&led2);
		}
	}
}
