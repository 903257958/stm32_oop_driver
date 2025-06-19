#include "main.h"

stepper_motor_dev_t stepper_motor = {.config = {
	GPIOB, GPIO_Pin_0,
	GPIOB, GPIO_Pin_1,
	GPIOB, GPIO_Pin_10,
	GPIOB, GPIO_Pin_11
}};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	stepper_motor_init(&stepper_motor);
	
	stepper_motor.control(&stepper_motor, 4096, 1);	// 顺时针旋转360°

	while (1)
	{
		
	}
}
