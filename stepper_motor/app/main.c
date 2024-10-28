#include "main.h"

StepperMotorDev_t StepperMotor = {.info = {
	GPIOA, GPIO_Pin_4,
	GPIOA, GPIO_Pin_5,
	GPIOA, GPIO_Pin_6,
	GPIOA, GPIO_Pin_7
}};

int main(void)
{
	stepper_motor_init(&StepperMotor);
	
	StepperMotor.control(&StepperMotor, 4096, 1);	// 顺时针初始化360°

	while(1)
	{
		
	}
}
