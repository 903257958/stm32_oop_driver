#include "key.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

#define TIMER_FREQ	72000000

#define	__key_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
										else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
										else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
										else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
										else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
										else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
										else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
									}

#define	__key_config_io_in_pd(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}
											
#define	__key_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __key_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

	#if defined(STM32F40_41xxx)
	#define TIMER_FREQ	84000000

	#elif defined(STM32F411xE)
	#define TIMER_FREQ	100000000

	#elif defined(STM32F429_439xx)
	#define TIMER_FREQ	90000000
	
	#endif

#define	__key_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
										else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
										else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
										else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
										else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
										else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
										else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
									}

#define	__key_config_io_in_pd(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}
											
#define	__key_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __key_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#endif
                                            
#endif

/* 按键状态 */
#define KEY_PRESSED		1
#define KEY_RELEASED	0

/* 环形缓冲区 */
#define BUF_LEN 10
#define NEXT_POS(x) ((x + 1) % BUF_LEN)

typedef struct {
	uint8_t flag[BUF_LEN];
	int read;
	int write;
} key_buf_t;

/* 全局变量定义 */
static uint8_t g_key_dev_num = 0; 			    // 已注册按键设备数
static timer_dev_t g_timer_key_tick;		    // 用于提供tick的定时器设备
static key_dev_t *g_key_dev[MAX_KEY_DEV_NUM];	// 存储已注册按键设备地址
static key_buf_t g_key_buf[MAX_KEY_DEV_NUM];	// 已注册按键设备的环形缓冲区

/* 函数声明 */
static bool __key_buf_is_empty(key_buf_t *buf);
static bool __key_buf_is_full(key_buf_t *buf);
static void __key_buf_write(key_buf_t *buf, int flag);
static uint8_t __key_buf_read(key_buf_t *buf);
static void __key_tick(void *param);
static uint8_t __key_get_flag(key_dev_t *dev);
static void __key_clear_flag(key_dev_t *dev);
static bool __key_is_pressed(key_dev_t *dev);
static void __key_event_handler(key_dev_t *dev, key_event_table_t *event_table);
static int8_t __key_deinit(key_dev_t *dev);

/******************************************************************************
 * @brief	初始化按键
 * @param	dev	:	key_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t key_init(key_dev_t *dev)
{
	if (!dev || g_key_dev_num >= MAX_KEY_DEV_NUM)
		return -1;
	
	/* 配置时钟与GPIO */
	__key_io_clock_enable(dev->config.port);
	
	if(dev->config.press_level == GPIO_LEVEL_HIGH)	// 根据press_level配置为上拉或下拉输入
	{
		__key_config_io_in_pd(dev->config.port, dev->config.pin);
	}
	else
	{
		__key_config_io_in_pu(dev->config.port, dev->config.pin);
	}
	
	/* 配置定时器 */
    if (g_key_dev_num == 0)
    {
        g_timer_key_tick.config.timx = dev->config.timx;
        g_timer_key_tick.config.psc = TIMER_FREQ / 1000000 - 1;	// 计数周期1us
        g_timer_key_tick.config.arr = 999;						// 定时周期1ms
        g_timer_key_tick.config.irq_callback = __key_tick;		// 注册回调函数
        g_timer_key_tick.config.irq_callback_param = NULL;
        g_timer_key_tick.config.preemption_priority = 0;
        g_timer_key_tick.config.sub_priority = 0;
        timer_init(&g_timer_key_tick);
    }

	/* 函数指针赋值 */
	dev->get_flag = __key_get_flag;
	dev->clear_flag = __key_clear_flag;
	dev->is_pressed = __key_is_pressed;
	dev->event_handler = __key_event_handler;
	dev->deinit = __key_deinit;
	
	dev->init_flag = true;
	dev->val = g_key_dev_num;			// 保存本次注册的按键值
	g_key_dev[g_key_dev_num] = dev;		// 保存本次注册的按键设备地址
	g_key_buf[g_key_dev_num].read = 0;
	g_key_buf[g_key_dev_num].write = 0;
	g_key_dev_num++;					// 按键设备数加1

	return 0;
}

/******************************************************************************
 * @brief	判断环形缓冲区空
 * @param	buf	:	key_buf_t 结构体指针
 * @return	true 表示空, false 表示不空
 ******************************************************************************/
static bool __key_buf_is_empty(key_buf_t *buf)
{
	return (buf->read == buf->write);
}

/******************************************************************************
 * @brief	判断环形缓冲区满
 * @param	buf	:	key_buf_t 结构体指针
 * @return	true 表示满, false 表示不满
 ******************************************************************************/
static bool __key_buf_is_full(key_buf_t *buf)
{
	return (buf->read == NEXT_POS(buf->write));
}

/******************************************************************************
 * @brief	环形缓冲区写数据
 * @param	buf		:	key_buf_t 结构体指针
 * @param	flag	:	按键状态标志位
 * @return	无
 ******************************************************************************/
static void __key_buf_write(key_buf_t *buf, int flag)
{
	if (!__key_buf_is_full(buf))
	{
		buf->flag[buf->write] = flag;
		buf->write = NEXT_POS(buf->write);
	}
}

/******************************************************************************
 * @brief	环形缓冲区读数据
 * @param	buf	:	key_buf_t 结构体指针
 * @return	读取到的按键状态标志位
 ******************************************************************************/
static uint8_t __key_buf_read(key_buf_t *buf)
{
	uint8_t flag = 0x00;

	if (!__key_buf_is_empty(buf))
	{
		flag = buf->flag[buf->read];
		buf->read = NEXT_POS(buf->read);
	}

	return flag;
}

/******************************************************************************
 * @brief	按键定时器中断回调函数，扫描已注册的按键，若有按键按下，放入环形缓冲区
 * @param	param
 * @return	无
 ******************************************************************************/
static void __key_tick(void *param)
{
    static uint8_t cnt = 0;
    static int8_t prev_status[MAX_KEY_DEV_NUM] = {0};	// 上次状态
    static int8_t curr_status[MAX_KEY_DEV_NUM] = {0};	// 本次状态
	static uint8_t state[MAX_KEY_DEV_NUM] = {0};		// 状态机
	static uint16_t timer[MAX_KEY_DEV_NUM] = {0};		// 状态机计时器
    uint8_t i;
	uint8_t flag;
	
	for (i = 0; i < g_key_dev_num; i++)
	{
		if (timer[i] > 0)
		{
			timer[i]--;
		}
	}

    /* 每20ms检测一次，消抖 */
    if (++cnt < 20)
    {
        return;
    }
    cnt = 0;

    for (i = 0; i < g_key_dev_num; i++)
    {
        prev_status[i] = curr_status[i];
		curr_status[i] = __key_is_pressed(g_key_dev[i]);

        flag = 0;

        if (prev_status[i] == KEY_RELEASED && curr_status[i] == KEY_PRESSED)
        {
			/* 按键按下（前一次是释放状态，当前是按下状态） */ 
            flag |= KEY_DOWN;
        }
        else if (prev_status[i] == KEY_PRESSED && curr_status[i] == KEY_RELEASED)
        {
			/* 按键释放（前一次是按下状态，当前是释放状态） */ 
            flag |= KEY_UP;
        }

		switch (state[i])
		{
			case 0:
			{
				if (curr_status[i] == KEY_PRESSED)
				{
					timer[i] = g_key_dev[i]->config.time_ms_long;
					state[i] = 1;
				}
				break;
			}
			case 1:
			{
				if (curr_status[i] == KEY_RELEASED)
				{
					timer[i] = g_key_dev[i]->config.time_ms_double;
					state[i] = 2;
				}
				else if (timer[i] == 0)
				{
					timer[i] = g_key_dev[i]->config.time_ms_repeat;
					flag |= KEY_LONG;
					state[i] = 4;
				}
				break;
			}
			case 2:
			{
				if (curr_status[i] == KEY_PRESSED)
				{
					flag |= KEY_DOUBLE_CLICK;
					state[i] = 3;
				}
				else if (timer[i] == 0)
				{
					flag |= KEY_CLICK;
					state[i] = 0;
				}
				break;
			}
			case 3:
			{
				if (curr_status[i] == KEY_RELEASED)
				{
					state[i] = 0;
				}
				break;
			}
			case 4:
			{
				if (curr_status[i] == KEY_RELEASED)
				{
					state[i] = 0;
				}
				else if (timer[i] == 0)
				{
					timer[i] = g_key_dev[i]->config.time_ms_repeat;
					flag |= KEY_REPEAT;
					state[i] = 4;
				}
				break;
			}
			default:
				break;
		}

        /* 仅当状态有变化时，才写入缓冲区 */
        if (flag != 0)
        {
            __key_buf_write(&g_key_buf[i], flag);
        }
    }
}

/******************************************************************************
 * @brief	获取按键状态标志位
 * @param	dev	:	key_dev_t 结构体指针
 * @return	读取到的按键状态标志位
 ******************************************************************************/
static uint8_t __key_get_flag(key_dev_t *dev)
{
	return __key_buf_read(&g_key_buf[dev->val]);
}

/******************************************************************************
 * @brief	清除缓冲区中所有按键状态标志位，用于进入新模式时清除历史事件
 * @param	dev	:	key_dev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __key_clear_flag(key_dev_t *dev)
{
	g_key_buf[dev->val].read = 0;
	g_key_buf[dev->val].write = 0;
}

/******************************************************************************
 * @brief	按键是否按下
 * @param	dev		:	key_dev_t 结构体指针
 * @return	true, 表示按下, false, 表示释放
 ******************************************************************************/
static bool __key_is_pressed(key_dev_t *dev)
{
	uint8_t io_status;

	io_status = __key_io_read(g_key_dev[dev->val]->config.port, g_key_dev[dev->val]->config.pin);

	if (io_status == g_key_dev[dev->val]->config.press_level)
	{
		return KEY_PRESSED;
	}
	else
	{
		return KEY_RELEASED;
	}
}

/******************************************************************************
 * @brief	按键事件处理
 * @param	dev		:	key_dev_t 结构体指针
 * @param	handler	:	key_event_table_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __key_event_handler(key_dev_t *dev, key_event_table_t *event_table)
{
    uint8_t flag;

	/* 从环形缓冲区中读取出标志位，针对不同事件类型调用回调函数 */
    while ((flag = __key_get_flag(dev)) != 0)
    {
		/* 按键按下 */
        if ((flag & KEY_DOWN) && event_table->events[KEY_EVENT_DOWN].func)
        {
            event_table->events[KEY_EVENT_DOWN].func(event_table->events[KEY_EVENT_DOWN].param);
        }

		/* 按键释放 */
        if ((flag & KEY_UP) && event_table->events[KEY_EVENT_UP].func)
        {
            event_table->events[KEY_EVENT_UP].func(event_table->events[KEY_EVENT_UP].param);
        }

		/* 按键单击 */
		if ((flag & KEY_CLICK) && event_table->events[KEY_EVENT_CLICK].func)
        {
            event_table->events[KEY_EVENT_CLICK].func(event_table->events[KEY_EVENT_CLICK].param);
        }

		/* 按键双击 */
		if ((flag & KEY_DOUBLE_CLICK) && event_table->events[KEY_EVENT_DOUBLE_CLICK].func)
        {
            event_table->events[KEY_EVENT_DOUBLE_CLICK].func(event_table->events[KEY_EVENT_DOUBLE_CLICK].param);
        }

		/* 按键长按 */
		if ((flag & KEY_LONG) && event_table->events[KEY_EVENT_LONG].func)
        {
            event_table->events[KEY_EVENT_LONG].func(event_table->events[KEY_EVENT_LONG].param);
        }

		/* 按键重复 */
        if ((flag & KEY_REPEAT) && event_table->events[KEY_EVENT_REPEAT].func)
        {
            event_table->events[KEY_EVENT_REPEAT].func(event_table->events[KEY_EVENT_REPEAT].param);
        }
    }
}

/******************************************************************************
 * @brief	去初始化按键
 * @param	dev	:	key_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __key_deinit(key_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	uint8_t i, index = 0xFF;

	/* 查找该设备在设备数组中的索引 */
	for (i = 0; i < g_key_dev_num; i++)
	{
		if (g_key_dev[i] == dev)
		{
			index = i;
			break;
		}
	}

	if (index == 0xFF)
		return -2;  // 没找到该设备，可能已被删除

	/* 停止定时器（仅当最后一个设备被移除时） */
	if (g_key_dev_num == 1 && g_timer_key_tick.deinit)
	{
		g_timer_key_tick.deinit(&g_timer_key_tick);
	}

	/* 将设备数组中的后续元素向前移动 */
	for (i = index; i < g_key_dev_num - 1; i++)
	{
		g_key_dev[i] = g_key_dev[i + 1];
		g_key_buf[i] = g_key_buf[i + 1];

		/* 更新 val 索引（避免主循环中使用了旧索引） */
		if (g_key_dev[i])
		{
			g_key_dev[i]->val = i;
		}
	}

	/* 清除最后一个元素 */
	g_key_dev[g_key_dev_num - 1] = NULL;
	g_key_buf[g_key_dev_num - 1].read = 0;
	g_key_buf[g_key_dev_num - 1].write = 0;

	/* 设备数减一 */
	g_key_dev_num--;

	/* 清除设备状态 */
	dev->init_flag = false;
	dev->val = 0xFF;
	dev->is_pressed = NULL;
	dev->event_handler = NULL;
	dev->deinit = NULL;

	return 0;
}
