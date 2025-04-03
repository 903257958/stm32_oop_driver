#include "main.h"

PassiveBuzzerDev_t passive_buzzer = {.config = {TIM4, 4, GPIOB, GPIO_Pin_9}};

int main(void)
{
    delay_init(168);
    
	passive_buzzer_init(&passive_buzzer);

	passive_buzzer.play_note(&passive_buzzer, L1, 1, 5);
	
	passive_buzzer.play_music(&passive_buzzer, music_test, 42);
	
	while (1)
	{
		
	}
}
