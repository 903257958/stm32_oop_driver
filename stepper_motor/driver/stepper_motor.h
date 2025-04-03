#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	StepperMotorGPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	StepperMotorGPIOPort_t;

#else
    #error stepper_motor.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

typedef struct {
	StepperMotorGPIOPort_t in1_port;			// IN1端口
	uint32_t in1_pin;							// IN1引脚
	StepperMotorGPIOPort_t in2_port;			// IN2端口
	uint32_t in2_pin;							// IN2引脚
	StepperMotorGPIOPort_t in3_port;			// IN3端口
	uint32_t in3_pin;							// IN3引脚
	StepperMotorGPIOPort_t in4_port;			// IN4端口
	uint32_t in4_pin;							// IN4引脚
}StepperMotorConfig_t;

typedef struct StepperMotorDev {
	StepperMotorConfig_t config;
	bool init_flag;														// 初始化标志
	void (*control)(struct StepperMotorDev *dev, int cnt, int time_ms);	// 控制
	void (*disable)(struct StepperMotorDev *dev);						// 关闭
	int (*deinit)(struct StepperMotorDev *dev);							// 去初始化
}StepperMotorDev_t;

int stepper_motor_init(StepperMotorDev_t *dev);

#endif
