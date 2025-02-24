#ifndef __PASSIVE_BUZZER_H
#define __PASSIVE_BUZZER_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "pwm.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef*	TimerPER_t;
	typedef GPIO_TypeDef*	PassiveBuzzerGPIOPort_t;

#elif defined(STM32F40_41xxx) || defined(STM32F411xE)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef*	TimerPER_t;
	typedef GPIO_TypeDef*	PassiveBuzzerGPIOPort_t;

#else
	#error passive_buzzer.h: No processor defined!
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
	TimerPER_t timx;				// 定时器外设
	uint8_t oc_channel;				// 输出比较通道
	PassiveBuzzerGPIOPort_t	port;	// 无源蜂鸣器端口
	uint32_t pin;					// 无源蜂鸣器引脚
}PassiveBuzzerInfo_t;

typedef struct PassiveBuzzerDev {
	PassiveBuzzerInfo_t info;
	bool init_flag;																// 初始化标志
	void *priv_data;															// 私有数据指针
	int (*set_frequency)(struct PassiveBuzzerDev *dev, uint16_t frequency);	// 设置无源蜂鸣器频率
	int (*set_sound)(struct PassiveBuzzerDev *dev, uint16_t compare);			// 设置无源蜂鸣器音量
	int (*play_note)(struct PassiveBuzzerDev *dev, NoteFrequency note, uint8_t beat, uint16_t sound);	// 无源蜂鸣器播放一个音符
	int (*play_music)(struct PassiveBuzzerDev *dev, PassiveBuzzerNote_t *music, uint16_t note_num);	// 无源蜂鸣器播放一段音乐
	int (*deinit)(struct PassiveBuzzerDev *dev);								// 去初始化
}PassiveBuzzerDev_t;

int passive_buzzer_init(PassiveBuzzerDev_t *dev);

#endif
