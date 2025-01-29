#include "main.h"

StepperMotorDev_t stepper_motor = {.info = {
	GPIOA, GPIO_Pin_4,
	GPIOA, GPIO_Pin_5,
	GPIOA, GPIO_Pin_6,
	GPIOA, GPIO_Pin_7
}};

int main(void)
{
	delay_init(168);
	stepper_motor_init(&stepper_motor);
	
	stepper_motor.control(&stepper_motor, 4096, 1);	// 顺时针旋转360°

	while(1)
	{
		
	}
}
