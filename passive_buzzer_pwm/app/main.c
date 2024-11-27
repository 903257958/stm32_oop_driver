#include "main.h"

PassiveBuzzerDev_t passiveBuzzer = {.info = {TIM4, 4, GPIOB, GPIO_Pin_9}};

int main(void)
{
	passive_buzzer_init(&passiveBuzzer);

	passiveBuzzer.play_note(&passiveBuzzer, L1, 1, 5);
	
	// passiveBuzzer.play_music(&passiveBuzzer, music_test, 42);
	
	while (1)
	{
		
	}
}
