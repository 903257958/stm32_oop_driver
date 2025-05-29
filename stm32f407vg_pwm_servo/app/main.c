#include "main.h"

servo_dev_t sg90 = {.config = {TIM4, 3, GPIOB, GPIO_Pin_8}};

int main(void)
{
    uint8_t i;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(168);
	servo_init(&sg90);
    
	while (1)
	{
        for (i = 0; i < 180; i++)
        {
            sg90.set_angle(&sg90, i);
            delay_ms(10);
        }   
	}
}
