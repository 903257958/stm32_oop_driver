#include "main.h"

/* 硬件设备定义 */
uart_dev_t debug = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
key_dev_t  key1  = {.config = {TIM2, GPIOB, GPIO_Pin_12, GPIO_LEVEL_LOW, KEY_DOUBLE_CLICK_DISABLE}};
key_dev_t  key2  = {.config = {TIM2, GPIOA, GPIO_Pin_8, GPIO_LEVEL_LOW, KEY_DOUBLE_CLICK_ENABLE}};
key_dev_t  key3  = {.config = {TIM2, GPIOB, GPIO_Pin_5, GPIO_LEVEL_LOW, KEY_DOUBLE_CLICK_ENABLE}};

#if 1
/* ========== 用法1：回调函数 + 事件映射表 ========== */

/* 回调函数 */
void key_down_cb(void *param)
{
    const char *name = (const char *)param;
    debug.printf("%s down!\r\n", name);
}

void key_up_cb(void *param)
{
    const char *name = (const char *)param;
    debug.printf("%s up!\r\n", name);
}

void key_click_cb(void *param)
{
    const char *name = (const char *)param;
    debug.printf("%s click!\r\n", name);
}

void key_double_click_cb(void *param)
{
    const char *name = (const char *)param;
    debug.printf("%s double click!\r\n", name);
}

void key_long_cb(void *param)
{
    const char *name = (const char *)param;
    debug.printf("%s long!\r\n", name);
}

void key_repeat_cb(void *param)
{
    const char *name = (const char *)param;
    debug.printf("%s repeat!\r\n", name);
}

/* 事件映射表 */
key_event_table_t key1_event_table = {
    .events[KEY_EVENT_DOWN] 		= { .func = key_down_cb, 			.param = "key1" },
    .events[KEY_EVENT_UP]   		= { .func = key_up_cb,   			.param = "key1" },
    .events[KEY_EVENT_CLICK]   		= { .func = key_click_cb,   		.param = "key1" },
    .events[KEY_EVENT_DOUBLE_CLICK]	= { .func = key_double_click_cb,	.param = "key1" },
    .events[KEY_EVENT_LONG]			= { .func = key_long_cb,			.param = "key1" },
    .events[KEY_EVENT_REPEAT]		= { .func = key_repeat_cb,			.param = "key1" },
};

key_event_table_t key2_event_table = {
    .events[KEY_EVENT_DOWN] 		= { .func = key_down_cb, 			.param = "key2" },
    .events[KEY_EVENT_UP]   		= { .func = key_up_cb,   			.param = "key2" },
    .events[KEY_EVENT_CLICK]   		= { .func = key_click_cb,   		.param = "key2" },
    .events[KEY_EVENT_DOUBLE_CLICK]	= { .func = key_double_click_cb,	.param = "key2" },
    .events[KEY_EVENT_LONG]			= { .func = key_long_cb,			.param = "key2" },
    .events[KEY_EVENT_REPEAT]		= { .func = key_repeat_cb,			.param = "key2" },
};

key_event_table_t key3_event_table = {
    .events[KEY_EVENT_DOWN] 		= { .func = key_down_cb, 			.param = "key3" },
    .events[KEY_EVENT_UP]   		= { .func = key_up_cb,   			.param = "key3" },
    .events[KEY_EVENT_CLICK]   		= { .func = key_click_cb,   		.param = "key3" },
    .events[KEY_EVENT_DOUBLE_CLICK]	= { .func = key_double_click_cb,	.param = "key3" },
    .events[KEY_EVENT_LONG]			= { .func = key_long_cb,			.param = "key3" },
    .events[KEY_EVENT_REPEAT]		= { .func = key_repeat_cb,			.param = "key3" },
};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    delay_init(72);
	uart_init(&debug);
	key_init(&key1);
	key_init(&key2);
    key_init(&key3);

	while (1)
	{
		key1.event_handler(&key1, &key1_event_table);
        key2.event_handler(&key2, &key2_event_table);
        key3.event_handler(&key3, &key3_event_table);
	}
}

#else
/* ========== 用法2：直接获取事件标志位 ========== */

int main(void)
{
	uint8_t flag;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    delay_init(72);
	uart_init(&debug);
	key_init(&key1);
	key_init(&key2);
    key_init(&key3);

	while (1)
	{
		while ((flag = key1.get_flag(&key1)) != 0)
		{
			if (flag & KEY_DOWN)
			{
				debug.printf("key1 down!\r\n");
			}
			if (flag & KEY_UP)
			{
				debug.printf("key1 up!\r\n");
			}
			if (flag & KEY_CLICK)
			{
				debug.printf("key1 click!\r\n");
			}
			if (flag & KEY_DOUBLE_CLICK)
			{
				debug.printf("key1 double click!\r\n");
			}
			if (flag & KEY_LONG)
			{
				debug.printf("key1 long!\r\n");
			}
			if (flag & KEY_REPEAT)
			{
				debug.printf("key1 repeat!\r\n");
			}
		}

		while ((flag = key2.get_flag(&key2)) != 0)
		{
			if (flag & KEY_DOWN)
			{
				debug.printf("key2 down!\r\n");
			}
			if (flag & KEY_UP)
			{
				debug.printf("key2 up!\r\n");
			}
			if (flag & KEY_CLICK)
			{
				debug.printf("key2 click!\r\n");
			}
			if (flag & KEY_DOUBLE_CLICK)
			{
				debug.printf("key2 double click!\r\n");
			}
			if (flag & KEY_LONG)
			{
				debug.printf("key2 long!\r\n");
			}
			if (flag & KEY_REPEAT)
			{
				debug.printf("key2 repeat!\r\n");
			}
		}

		while ((flag = key3.get_flag(&key3)) != 0)
		{
			if (flag & KEY_DOWN)
			{
				debug.printf("key3 down!\r\n");
			}
			if (flag & KEY_UP)
			{
				debug.printf("key3 up!\r\n");
			}
			if (flag & KEY_CLICK)
			{
				debug.printf("key3 click!\r\n");
			}
			if (flag & KEY_DOUBLE_CLICK)
			{
				debug.printf("key3 double click!\r\n");
			}
			if (flag & KEY_LONG)
			{
				debug.printf("key3 long!\r\n");
			}
			if (flag & KEY_REPEAT)
			{
				debug.printf("key3 repeat!\r\n");
			}
		}
	}
}
#endif
