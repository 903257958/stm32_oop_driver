#ifndef __PASSIVE_BUZZER_H
#define __PASSIVE_BUZZER_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "pwm.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef TIM_TypeDef*	TIMx;
	typedef GPIO_TypeDef*	PassiveBuzzer_GPIO_Port;
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef TIM_TypeDef*	TIMx;
	typedef GPIO_TypeDef*	PassiveBuzzer_GPIO_Port;
#else
	#error passive_buzzer.h: No processor defined!
#endif

#ifndef FREERTOS
	#define FREERTOS	0
#endif

#if FREERTOS
	#include "FreeRTOS.h"
	#include "task.h"
#endif

#ifndef passive_buzzer_log
	#define passive_buzzer_log(x) 
#endif

/* 音符与频率对应关系 */
typedef enum  {
    L1 = 262,
    L2 = 294,
    L3 = 330,
    L4 = 349,
    L5 = 392,
    L6 = 440,
    L7 = 494,
    M1 = 523,
    M2 = 587,
    M3 = 659,
    M4 = 698,
    M5 = 784,
    M6 = 880,
    M7 = 988,
    H1 = 1046,
    H2 = 1175,
    H3 = 1318,
    H4 = 1397,
    H5 = 1568,
    H6 = 1760,
    H7 = 1976,
    MUTE = 0
}NoteFrequency;

/* 单个音符结构体 */
typedef struct {
	NoteFrequency note;
	uint8_t beat;
	uint16_t sound;
}PassiveBuzzerNote_t;

/* 测试音乐 */
extern PassiveBuzzerNote_t music_test[42];

typedef struct {
	TIMx timx;						// 定时器外设
	uint8_t OCChannel;				// 输出比较通道
	PassiveBuzzer_GPIO_Port	port;	// 无源蜂鸣器端口
	uint32_t pin;					// 无源蜂鸣器引脚
}PassiveBuzzerInfo_t;

typedef struct PassiveBuzzerDev {
	PassiveBuzzerInfo_t info;
	bool initFlag;																// 初始化标志
	void *pPrivData;															// 私有数据指针
	void (*set_frequency)(struct PassiveBuzzerDev *pDev, uint16_t frequency);	// 设置无源蜂鸣器频率
	void (*set_sound)(struct PassiveBuzzerDev *pDev, uint16_t compare);			// 设置无源蜂鸣器音量
	void (*play_note)(struct PassiveBuzzerDev *pDev, NoteFrequency note, uint8_t beat, uint16_t sound);	// 无源蜂鸣器播放一个音符
	void (*play_music)(struct PassiveBuzzerDev *pDev, PassiveBuzzerNote_t *music, uint16_t noteNum);	// 无源蜂鸣器播放一段音乐
	int (*deinit)(struct PassiveBuzzerDev *pDev);								// 去初始化
}PassiveBuzzerDev_t;

int passive_buzzer_init(PassiveBuzzerDev_t *pDev);

#endif
