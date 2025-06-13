#ifndef STEPPER_MOTOR_DRV_H
#define STEPPER_MOTOR_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	stepper_motor_gpio_port_t;
	typedef uint32_t		stepper_motor_gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	stepper_motor_gpio_port_t;
	typedef uint32_t		stepper_motor_gpio_pin_t;

#else
    #error stepper_motor.h: No processor defined!
#endif

#include "delay.h"

#ifndef STEPPER_MOTOR_DELAY_MS
	#define STEPPER_MOTOR_DELAY_MS(ms)	delay_ms(ms)
#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

typedef struct {
	stepper_motor_gpio_port_t in1_port;	// IN1端口
	stepper_motor_gpio_pin_t in1_pin;	// IN1引脚
	stepper_motor_gpio_port_t in2_port;	// IN2端口
	stepper_motor_gpio_pin_t in2_pin;	// IN2引脚
	stepper_motor_gpio_port_t in3_port;	// IN3端口
	stepper_motor_gpio_pin_t in3_pin;	// IN3引脚
	stepper_motor_gpio_port_t in4_port;	// IN4端口
	stepper_motor_gpio_pin_t in4_pin;	// IN4引脚
} stepper_motor_config_t;

typedef struct stepper_motor_dev {
	stepper_motor_config_t config;
	bool init_flag;																// 初始化标志
	void (*control)(struct stepper_motor_dev *dev, int cnt, uint16_t time_ms);	// 控制
	void (*disable)(struct stepper_motor_dev *dev);								// 关闭
	int8_t (*deinit)(struct stepper_motor_dev *dev);							// 去初始化
} stepper_motor_dev_t;

int8_t stepper_motor_init(stepper_motor_dev_t *dev);

#endif
