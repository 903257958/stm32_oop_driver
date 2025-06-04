#include "main.h"

vibration_motor_dev_t vibration_motor = {.config = {GPIOB, GPIO_Pin_5}};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(168);
	vibration_motor_init(&vibration_motor);
	
    vibration_motor.on(&vibration_motor);
	delay_ms(500);
    vibration_motor.off(&vibration_motor);
    
	while (1)
	{
	
	}
}
