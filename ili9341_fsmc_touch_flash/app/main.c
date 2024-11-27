#include "main.h"

KeyDev_t key = {.info = {GPIOA, GPIO_Pin_1, GPIO_LEVEL_LOW, 1}};
TimerDev_t timerKeyTick = {.info.timx = TIM2};
W25QXDev_t flash = {.info = {SPI1, GPIOC, GPIO_Pin_13}};
LCDDev_t lcd;
XPT2046Dev_t xpt2046;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	key_init(&key);
	w25qx_init(&flash);
	lcd_init(&lcd);
	xpt2046_init(&xpt2046);
	
	lcd.clear(&lcd, BLACK);

	while (1)
	{
		uint8_t keyVal = key_get_val();
		
		xpt2046.draw(&xpt2046, YELLOW, BLACK);
		
        if (keyVal == 1)
		{
			xpt2046.recalibration(&xpt2046);
		}
	}
}
