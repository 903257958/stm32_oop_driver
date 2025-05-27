#include "main.h"

ServoDev_t sg90 = {.config = {TIM4, 3, GPIOB, GPIO_Pin_8}};

int main(void)
{
    delay_init(168);
    
	servo_init(&sg90);

    int i;
    
	while (1)
	{
        for (i = 0; i < 180; i++)
        {
            sg90.set_angle(&sg90, i);
            delay_ms(10);
        }   
	}
}
