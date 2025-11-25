#include "main.h"

tb6612_dev_t tb6612 = {
    .config = {TIM4, 3, GPIOB, GPIO_Pin_8, GPIOB, GPIO_Pin_0, GPIOB, GPIO_Pin_1, NULL}
};

int main(void)
{
    int8_t i;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	tb6612_init(&tb6612);
    
	while (1)
	{
        for (i = -100; i <= 100; i += 50)
        {
            tb6612.set_speed(&tb6612, MOTOR_2, i);
            delay_s(2);
        }
        for (i = 50; i >= -100; i -= 50)
        {
            tb6612.set_speed(&tb6612, MOTOR_2, i);
            delay_s(2);
        }
	}
}
