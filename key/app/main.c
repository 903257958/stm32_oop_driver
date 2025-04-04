#include "main.h"

LEDDev_t led1 = {.config = {GPIOB, GPIO_Pin_2, GPIO_LEVEL_LOW}};
LEDDev_t led2 = {.config = {GPIOB, GPIO_Pin_1, GPIO_LEVEL_LOW}};

KeyDev_t key1 = {.config = {GPIOA, GPIO_Pin_1, GPIO_LEVEL_LOW}};
KeyDev_t key2 = {.config = {GPIOA, GPIO_Pin_4, GPIO_LEVEL_LOW}};

int main(void)
{
    delay_init(168);
    
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
