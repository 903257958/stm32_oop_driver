#include "main.h"

LEDDev_t led[] = {
	{.info = {GPIOC, GPIO_Pin_13, GPIO_LEVEL_LOW}},
	{.info = {GPIOB, GPIO_Pin_8, GPIO_LEVEL_LOW}}
};

KeyDev_t key[] = {
	{.info = {GPIOC, GPIO_Pin_1, GPIO_LEVEL_LOW, 1}},
	{.info = {GPIOC, GPIO_Pin_2, GPIO_LEVEL_LOW, 2}},
	{.info = {GPIOC, GPIO_Pin_3, GPIO_LEVEL_LOW, 3}}
};

int main(void)
{
	int i;
	int ledDevCount = sizeof(led)/sizeof(led[0]);
	int keyDevCount = sizeof(key)/sizeof(key[0]);
	int keyValue;

	for (i = 0; i < ledDevCount; i++)
	{
		led_init(&led[i]);
	}

	for (i = 0; i < keyDevCount; i++)
	{
		key_init(&key[i]);
	}
	
#if 1

	/* 检测按键并翻转LED */
	while (1)
	{
		key_scan(key, keyDevCount);
		keyValue = key_get_value();

		if (keyValue == 1)
		{
			led[0].toggle(&led[0]);
		}
		else if (keyValue == 2)
		{
			led[1].toggle(&led[1]);
		}
		else if (keyValue == 3)
		{
			led[0].toggle(&led[0]);
			led[1].toggle(&led[1]);
		}
	}
	
#else

	/* 检测到五次按键变化后，停止检测，开始从缓冲区读数据并且翻转LED灯 */
	int t = 5;
	while (t > 0)
	{
		if (key_scan(key, keyDevCount) == 0)
		{
			t--;
		}
	}

	while (1)
	{
		keyValue = key_get_value();

		if (keyValue == 1)
		{
			led[0].toggle(&led[0]);
			delay_ms(500);
		}
		else if (keyValue == 2)
		{
			led[1].toggle(&led[1]);
			delay_ms(500);
		}
		else if (keyValue == 3)
		{
			led[0].toggle(&led[0]);
			led[1].toggle(&led[1]);
			delay_ms(500);
		}
	}

#endif
}
