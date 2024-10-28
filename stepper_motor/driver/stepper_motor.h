#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef GPIO_TypeDef*	StepperMotor_GPIO_Port;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	StepperMotor_GPIO_Port;

#else
    #error stepper_motor.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

#ifndef FREERTOS
	#define FREERTOS	0
#endif

#if FREERTOS
	#include "FreeRTOS.h"
	#include "task.h"
#endif

#ifndef stepper_motor_log
    #define stepper_motor_log(x) 
#endif

typedef struct {
	StepperMotor_GPIO_Port in1Port;				// IN1端口
	uint32_t in1Pin;							// IN1引脚
	StepperMotor_GPIO_Port in2Port;				// IN2端口
	uint32_t in2Pin;							// IN2引脚
	StepperMotor_GPIO_Port in3Port;				// IN3端口
	uint32_t in3Pin;							// IN3引脚
	StepperMotor_GPIO_Port in4Port;				// IN4端口
	uint32_t in4Pin;							// IN4引脚
}StepperMotorInfo_t;

typedef struct StepperMotorDev {
	StepperMotorInfo_t info;
	bool initFlag;															// 初始化标志
	void (*control)(struct StepperMotorDev *pDev, int count, int timeMs);	// 控制
	void (*disable)(struct StepperMotorDev *pDev);							// 关闭
	int (*deinit)(struct StepperMotorDev *pDev);							// 去初始化
}StepperMotorDev_t;

int stepper_motor_init(StepperMotorDev_t *pDev);

#endif
