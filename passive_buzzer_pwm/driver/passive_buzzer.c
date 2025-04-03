#include "delay.h"
#include "passive_buzzer.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define TIMER_FREQ	72000000

#elif defined(STM32F40_41xxx)
#define TIMER_FREQ	84000000

#elif defined(STM32F411xE)
#define TIMER_FREQ	100000000

#elif defined(STM32F429_439xx)
#define TIMER_FREQ	90000000

#endif

/* PassiveBuzzer私有数据结构体 */
typedef struct {
	PWMDev_t pwm;		// PWM设备
}PassiveBuzzerPrivData_t;

/* 函数声明 */
static int __passive_buzzer_set_frequency(PassiveBuzzerDev_t *dev, uint16_t frequency);
static int __passive_buzzer_set_sound(PassiveBuzzerDev_t *dev, uint16_t compare);
static int __passive_buzzer_play_note(PassiveBuzzerDev_t *dev, NoteFrequency note, uint8_t beat, uint16_t sound);
static int __passive_buzzer_play_music(PassiveBuzzerDev_t *dev, PassiveBuzzerNote_t *music, uint16_t note_num);
static int __passive_buzzer_deinit(PassiveBuzzerDev_t *dev);

/******************************************************************************
 * @brief	初始化无源蜂鸣器
			以STM32F1为例，PSC固定配置为72 - 1，通过修改ARR的值来产生不同频率的PWM波
			ARR的范围为0~65535，则PWM的频率范围约为>15Hz
			音符频率的范围为262~1976Hz，对应ARR的范围为3817~506
 * @param	dev	:	PassiveBuzzerDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int passive_buzzer_init(PassiveBuzzerDev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 保存私有数据 */
	dev->priv_data = (PassiveBuzzerPrivData_t *)malloc(sizeof(PassiveBuzzerPrivData_t));
	if (!dev->priv_data)
		return -1;
	
	PassiveBuzzerPrivData_t *priv_data = (PassiveBuzzerPrivData_t *)dev->priv_data;
	
	priv_data->pwm.config.timx = dev->config.timx;
	priv_data->pwm.config.oc_channel = dev->config.oc_channel;
	priv_data->pwm.config.psc = TIMER_FREQ / 1000000 - 1;
	priv_data->pwm.config.arr = 3817 - 1;
	priv_data->pwm.config.port = dev->config.port;
	priv_data->pwm.config.pin = dev->config.pin;
	
	/* 配置PWM */
	pwm_init(&priv_data->pwm);
	
	/* 函数指针赋值 */
	dev->set_frequency = __passive_buzzer_set_frequency;
	dev->set_sound = __passive_buzzer_set_sound;
	dev->play_note = __passive_buzzer_play_note;
	dev->play_music = __passive_buzzer_play_music;
	dev->deinit = __passive_buzzer_deinit;
	
	dev->init_flag = true;
    __passive_buzzer_set_sound(dev, 0);
	return 0;
}

/******************************************************************************
 * @brief	设置无源蜂鸣器频率
 * @param	dev			：	PassiveBuzzerDev_t 结构体指针
 * @param	frequency	：	要设置的频率，范围：262~1976
 * @return	无
 ******************************************************************************/
static int __passive_buzzer_set_frequency(PassiveBuzzerDev_t *dev, uint16_t frequency)
{
    if (!dev || !dev->init_flag)
		return -1;
    
	PassiveBuzzerPrivData_t *priv_data = (PassiveBuzzerPrivData_t *)dev->priv_data;
	
	priv_data->pwm.set_arr(&priv_data->pwm, 1000000 / frequency);
    
    return 0;
}

/******************************************************************************
 * @brief	设置无源蜂鸣器占空比
 * @param	dev		：	PassiveBuzzerDev_t结构体指针
 * @param	compare	：	要设置的占空比
 * @return	无
 ******************************************************************************/
static int __passive_buzzer_set_sound(PassiveBuzzerDev_t *dev, uint16_t compare)
{
    if (!dev || !dev->init_flag)
		return -1;
    
	PassiveBuzzerPrivData_t *priv_data = (PassiveBuzzerPrivData_t *)dev->priv_data;
	
	priv_data->pwm.set_compare(&priv_data->pwm, compare);
    
    return 0;
}

/******************************************************************************
 * @brief	无源蜂鸣器播放一个音符
 * @param	dev		：	PassiveBuzzerDev_t结构体指针
 * @param	note	：	要播放的音符
 * @param	beat	：	节拍数（一个节拍为250ms）
 * @param	sound	：	音量
 * @return	无
 ******************************************************************************/
static int __passive_buzzer_play_note(PassiveBuzzerDev_t *dev, NoteFrequency note, uint8_t beat, uint16_t sound)
{
    if (!dev || !dev->init_flag)
		return -1;
    
	__passive_buzzer_set_sound(dev, sound);
	__passive_buzzer_set_frequency(dev, note);
	while(beat--)
	{
		delay_ms(200);
	}
	__passive_buzzer_set_sound(dev, 0);
    
    return 0;
}

/******************************************************************************
 * @brief	无源蜂鸣器播放一段音乐
 * @param	dev			：	PassiveBuzzerDev_t结构体指针
 * @param	music		：	要播放的音乐的结构体数组首地址
 * @param	note_num	：	要播放的音乐的音符数
 * @return	无
 ******************************************************************************/
static int __passive_buzzer_play_music(PassiveBuzzerDev_t *dev, PassiveBuzzerNote_t *music, uint16_t note_num)
{
    if (!dev || !dev->init_flag)
		return -1;
    
	for(uint16_t i = 0; i < note_num; i++)
	{
		__passive_buzzer_play_note(dev, music[i].note, music[i].beat, music[i].sound);
	}
    
    return 0;
}

/******************************************************************************
 * @brief	去初始化无源蜂鸣器
 * @param	dev   :  PassiveBuzzerDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __passive_buzzer_deinit(PassiveBuzzerDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	PassiveBuzzerPrivData_t *priv_data = (PassiveBuzzerPrivData_t *)dev->priv_data;
	
	/* 去初始化PWM */
	priv_data->pwm.deinit(&priv_data->pwm);
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
    dev->priv_data = NULL;
	
	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}

/* 测试音乐 */
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
