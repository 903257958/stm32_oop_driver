#include "passive_buzzer.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

#define TIMER_FREQ	72000000

	#if !FREERTOS
	static void __passive_buzzer_delay_ms(uint32_t ms)
	{
		while(ms--)
		{
			SysTick->LOAD = 72 * 1000;				// 设置定时器重装值
			SysTick->VAL = 0x00;					// 清空当前计数值
			SysTick->CTRL = 0x00000005;				// 设置时钟源为HCLK，启动定时器
			while(!(SysTick->CTRL & 0x00010000));	// 等待计数到0
			SysTick->CTRL = 0x00000004;				// 关闭定时器
		}
	}
	#else		
	static void __passive_buzzer_delay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif

#elif defined(STM32F40_41xxx)

#define TIMER_FREQ	84000000
	
	#if !FREERTOS
	static void __passive_buzzer_delay_ms(uint32_t ms)
	{
		while(ms--)
		{
			uint32_t temp;	    	 
			SysTick->LOAD = 1000 * 21; 					// 时间加载	  		 
			SysTick->VAL = 0x00;        				// 清空计数器
			SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ; 	// 开始倒数 	 
			do
			{
				temp = SysTick->CTRL;
			}while((temp&0x01) && !(temp&(1<<16)));		// 等待时间到达   
			SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; 	// 关闭计数器
			SysTick->VAL = 0X00;       					// 清空计数器 
		}
	}
	#else		
	static void __passive_buzzer_delay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif

#endif

/* PassiveBuzzer私有数据结构体 */
typedef struct {
	PWMDev_t passiveBuzzer;		// PWM设备
}PassiveBuzzerPrivData_t;

static void __passive_buzzer_set_frequency(PassiveBuzzerDev_t *pDev, uint16_t frequency);
static void __passive_buzzer_set_sound(PassiveBuzzerDev_t *pDev, uint16_t compare);
static void __passive_buzzer_play_note(PassiveBuzzerDev_t *pDev, NoteFrequency note, uint8_t beat, uint16_t sound);
static void __passive_buzzer_play_music(PassiveBuzzerDev_t *pDev, PassiveBuzzerNote_t *music, uint16_t noteNum);
static int __passive_buzzer_deinit(PassiveBuzzerDev_t *pDev);

/******************************************************************************
 * @brief	初始化无源蜂鸣器
			以STM32F1为例，PSC固定配置为72 - 1，通过修改ARR的值来产生不同频率的PWM波
			ARR的范围为0~65535，则PWM的频率范围约为>15Hz
			音符频率的范围为262~1976Hz，对应ARR的范围为3817~506
 * @param	pDev	:	PassiveBuzzerDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int passive_buzzer_init(PassiveBuzzerDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 保存私有数据 */
	pDev->pPrivData = (PassiveBuzzerPrivData_t *)malloc(sizeof(PassiveBuzzerPrivData_t));
	if (!pDev->pPrivData)
		return -1;
	
	PassiveBuzzerPrivData_t *pPrivData = (PassiveBuzzerPrivData_t *)pDev->pPrivData;
	
	pPrivData->passiveBuzzer.info.timx = pDev->info.timx;
	pPrivData->passiveBuzzer.info.OCChannel = pDev->info.OCChannel;
	pPrivData->passiveBuzzer.info.psc = TIMER_FREQ / 1000000 - 1;
	pPrivData->passiveBuzzer.info.arr = 3817 - 1;
	pPrivData->passiveBuzzer.info.port = pDev->info.port;
	pPrivData->passiveBuzzer.info.pin = pDev->info.pin;
	
	/* 配置PWM */
	pwm_init(&pPrivData->passiveBuzzer);
	
	/* 函数指针赋值 */
	pDev->set_frequency = __passive_buzzer_set_frequency;
	pDev->set_sound = __passive_buzzer_set_sound;
	pDev->play_note = __passive_buzzer_play_note;
	pDev->play_music = __passive_buzzer_play_music;
	pDev->deinit = __passive_buzzer_deinit;
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	设置无源蜂鸣器频率
 * @param	pDev		：	PassiveBuzzerDev_t结构体指针
 * @param	frequency	：	要设置的频率，范围：262~1976
 * @return	无
 ******************************************************************************/
static void __passive_buzzer_set_frequency(PassiveBuzzerDev_t *pDev, uint16_t frequency)
{
	PassiveBuzzerPrivData_t *pPrivData = (PassiveBuzzerPrivData_t *)pDev->pPrivData;
	
	pPrivData->passiveBuzzer.set_arr(&pPrivData->passiveBuzzer, 1000000 / frequency);
}

/******************************************************************************
 * @brief	设置无源蜂鸣器占空比
 * @param	pDev	：	PassiveBuzzerDev_t结构体指针
 * @param	compare	：	要设置的占空比
 * @return	无
 ******************************************************************************/
static void __passive_buzzer_set_sound(PassiveBuzzerDev_t *pDev, uint16_t compare)
{
	PassiveBuzzerPrivData_t *pPrivData = (PassiveBuzzerPrivData_t *)pDev->pPrivData;
	
	pPrivData->passiveBuzzer.set_compare(&pPrivData->passiveBuzzer, compare);
}

/******************************************************************************
 * @brief	无源蜂鸣器播放一个音符
 * @param	pDev	：	PassiveBuzzerDev_t结构体指针
 * @param	note	：	要播放的音符
 * @param	beat	：	节拍数（一个节拍为250ms）
 * @param	sound	：	音量
 * @return	无
 ******************************************************************************/
static void __passive_buzzer_play_note(PassiveBuzzerDev_t *pDev, NoteFrequency note, uint8_t beat, uint16_t sound)
{
	__passive_buzzer_set_sound(pDev, sound);
	__passive_buzzer_set_frequency(pDev, note);
	while(beat--)
	{
		__passive_buzzer_delay_ms(200);
	}
	__passive_buzzer_set_sound(pDev, 0);
}

/******************************************************************************
 * @brief	无源蜂鸣器播放一段音乐
 * @param	pDev	：	PassiveBuzzerDev_t结构体指针
 * @param	music	：	要播放的音乐的结构体数组首地址
 * @param	noteNum	：	要播放的音乐的音符数
 * @return	无
 ******************************************************************************/
static void __passive_buzzer_play_music(PassiveBuzzerDev_t *pDev, PassiveBuzzerNote_t *music, uint16_t noteNum)
{
	for(uint16_t i = 0; i < noteNum; i++)
	{
		__passive_buzzer_play_note(pDev, music[i].note, music[i].beat, music[i].sound);
	}
}

/******************************************************************************
 * @brief	去初始化PassiveBuzzer
 * @param	pDev   :  PassiveBuzzerDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __passive_buzzer_deinit(PassiveBuzzerDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	PassiveBuzzerPrivData_t *pPrivData = (PassiveBuzzerPrivData_t *)pDev->pPrivData;
	
	/* 去初始化PWM */
	pPrivData->passiveBuzzer.deinit(&pPrivData->passiveBuzzer);
	
	/* 释放私有数据内存 */
	free(pDev->pPrivData);
    pDev->pPrivData = NULL;
	
	pDev->initFlag = false;	// 修改初始化标志
	
	return 0;
}

/*测试音乐*/
PassiveBuzzerNote_t music_test[42] = {
	{L1, 1, 5},{MUTE, 1, 5},
	{L2, 1, 5},{MUTE, 1, 5},
	{L3, 1, 5},{MUTE, 1, 5},
	{L4, 1, 5},{MUTE, 1, 5},
	{L5, 1, 5},{MUTE, 1, 5},
	{L6, 1, 5},{MUTE, 1, 5},
	{L7, 1, 5},{MUTE, 1, 5},
	{M1, 1, 5},{MUTE, 1, 5},
	{M2, 1, 5},{MUTE, 1, 5},
	{M3, 1, 5},{MUTE, 1, 5},
	{M4, 1, 5},{MUTE, 1, 5},
	{M5, 1, 5},{MUTE, 1, 5},
	{M6, 1, 5},{MUTE, 1, 5},
	{M7, 1, 5},{MUTE, 1, 5},
	{H1, 1, 5},{MUTE, 1, 5},
	{H2, 1, 5},{MUTE, 1, 5},
	{H3, 1, 5},{MUTE, 1, 5},
	{H4, 1, 5},{MUTE, 1, 5},
	{H5, 1, 5},{MUTE, 1, 5},
	{H6, 1, 5},{MUTE, 1, 5},
	{H7, 1, 5},{MUTE, 1, 5}
};
