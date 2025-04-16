#include "main.h"

PWMDev_t led = {.config = {TIM4, 3, 168 - 1, 10000 - 1, GPIOB, GPIO_Pin_8}};

int main(void)
{
    delay_init(168);
    
	pwm_init(&led);

    uint16_t i = 0;
    
	while (1)
	{
        led.set_compare(&led, i);
        i++;
        if (i >= 5000)
        {
            i = 0;
        }
        
        
        delay_ms(1);
	}
}
