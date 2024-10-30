#include "main.h"

StepperMotorDev_t stepperMotor = {.info = {
	GPIOA, GPIO_Pin_4,
	GPIOA, GPIO_Pin_5,
	GPIOA, GPIO_Pin_6,
	GPIOA, GPIO_Pin_7
}};

int main(void)
{
	stepper_motor_init(&stepperMotor);
	
	stepperMotor.control(&stepperMotor, 4096, 1);	// 顺时针旋转360°

	while(1)
	{
		
	}
}
