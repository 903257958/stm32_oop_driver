#include "key.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

#define TIMER_FREQ	72000000

#define	__key_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
													else					{key_log("key clock no enable\r\n");} \
												}

#define	__key_config_io_in_pd(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}
											
#define	__key_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __key_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#elif defined(STM32F40_41xxx) || defined(STM32F411xE)

	#if defined(STM32F40_41xxx)
	#define TIMER_FREQ	84000000

	#elif defined(STM32F411xE)
	#define TIMER_FREQ	100000000
	
	#endif

#define	__key_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
													else					{key_log("gpio clock no enable\r\n");} \
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

/* 环形缓冲区 */
#define BUF_LEN 10
#define NEXT_POS(x) ((x+1) % BUF_LEN)

typedef struct {
	int val[BUF_LEN];
	int read;
	int write;
}KeyBuf_t;

#define MAX_KEY_NUM	10						// 最大按键数量

static KeyDev_t *g_key_dev[MAX_KEY_NUM];	// 存储已注册按键设备，用于中断回调函数传参
static uint8_t g_key_dev_num = 0; 			// 已注册按键设备数
static TimerDev_t g_timer_key_tick;			// 用于提供tick的定时器设备
static KeyBuf_t g_key_buf = {
	.read = 0,
	.write = 0
};

/* 函数声明 */			
static int __key_deinit(KeyDev_t *dev);

/******************************************************************************
 * @brief	判断环形缓冲区空
 * @param	buf	:  KeyBuf_t 结构体指针
 * @return	true 表示空, false 表示不空
 ******************************************************************************/
static bool __key_buf_is_empty(KeyBuf_t *buf)
{
	return (buf->read == buf->write);
}

/******************************************************************************
 * @brief	判断环形缓冲区满
 * @param	buf	:  KeyBuf_t 结构体指针
 * @return	true 表示满, false 表示不满
 ******************************************************************************/
static bool __key_buf_is_full(KeyBuf_t *buf)
{
	return (buf->read == NEXT_POS(buf->write));
}

/******************************************************************************
 * @brief	环形缓冲区写数据
 * @param	buf	:  KeyBuf_t 结构体指针
 * @param	val	:  按键数据
 * @return	无
 ******************************************************************************/
static void __key_buf_write(KeyBuf_t *buf, int val)
{
	if (!__key_buf_is_full(buf))
	{
		buf->val[buf->write] = val;
		buf->write = NEXT_POS(buf->write);
	}
}

/******************************************************************************
 * @brief	环形缓冲区读数据
 * @param	buf	:  KeyBuf_t 结构体指针
 * @param	val	:  按键数据
 * @return	读取到的按键值
 ******************************************************************************/
static int __key_buf_read(KeyBuf_t *buf)
{
	int val = 0;

	if (!__key_buf_is_empty(buf))
	{
		val = buf->val[buf->read];
		buf->read = NEXT_POS(buf->read);
	}

	return val;
}

/******************************************************************************
 * @brief	获取按键状态
 * @param	无
 * @return	若有按键被按下，则返回此按键的按键值，否则返回-1
 ******************************************************************************/
static int __key_get_state(void)
{
	uint8_t i;

	/* 遍历所有按键设备，找出第一个已按下的按键 */
	for (i = 0; i < g_key_dev_num; i++)
	{
		if (__key_io_read(g_key_dev[i]->info.port, g_key_dev[i]->info.pin) == g_key_dev[i]->info.press_level)
		{
			return g_key_dev[i]->info.val;
		}
	}

	return -1;
}

/******************************************************************************
 * @brief	按键定时器中断回调函数，扫描已注册的按键，若有按键按下，放入环形缓冲区
 * @param	无
 * @return	无
 ******************************************************************************/
static void __key_tick(void)
{
	static uint8_t cnt;
	static int8_t curr_state, prev_state;
	uint8_t key_val;

	/* 定时中断为1ms，每20ms检测一次按键状态，同时可以软件消抖 */
	if (++cnt >= 20)
	{
		cnt = 0;
		
		prev_state = curr_state;			// 上次按键状态
		curr_state = __key_get_state();		// 本次按键状态
		
		/* 若本次查询无按键按下，上次有按键按下，则说明此按键已松开，记录按键值 */
		if (curr_state == -1 && prev_state != -1)
		{
			key_val = prev_state;
			__key_buf_write(&g_key_buf, key_val);	// 写入环形缓冲区
		}
	}
}

/******************************************************************************
 * @brief	获取按键值
 * @param	无
 * @return	读取到的按键值
 ******************************************************************************/
int key_get_val(void)
{
	return __key_buf_read(&g_key_buf);
}
										
/******************************************************************************
 * @brief	初始化按键
 * @param	dev	:  KeyDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int key_init(KeyDev_t *dev)
{
	if (!dev || g_key_dev_num >= MAX_KEY_NUM)
		return -1;
	
	/* 配置时钟与GPIO */
	__key_config_gpio_clock_enable(dev->info.port);
	
	if(dev->info.press_level == GPIO_LEVEL_HIGH)			// 根据press_level配置为上拉或下拉输入
	{
		__key_config_io_in_pd(dev->info.port, dev->info.pin);
	}
	else
	{
		__key_config_io_in_pu(dev->info.port, dev->info.pin);
	}
	
	/* 配置定时器 */
	g_timer_key_tick.info.timx = dev->info.timx;
	g_timer_key_tick.info.psc = TIMER_FREQ / 1000000 - 1;	// 计数周期1us
	g_timer_key_tick.info.arr = 999;						// 定时周期1ms
	g_timer_key_tick.info.irq_callback = __key_tick;		// 注册回调函数
	timer_init(&g_timer_key_tick);

	/* 函数指针赋值 */
	dev->deinit = __key_deinit;
	
	dev->init_flag = true;
	g_key_dev[g_key_dev_num] = dev;		// 保存本次注册的按键设备地址
	g_key_dev_num++;					// 按键设备数加1
	return 0;
}

/******************************************************************************
 * @brief	去初始化按键
 * @param	dev   :  KeyDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __key_deinit(KeyDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	uint8_t i;
	
	g_timer_key_tick.deinit(&g_timer_key_tick);

	dev->init_flag = false;	// 修改初始化标志

	/* 从按键设备数组中移除该设备 */
    for (i = 0; i < g_key_dev_num; i++)
    {
        if (g_key_dev[i] == dev)
        {
            g_key_dev[i] = NULL;
            break;
        }
    }
	
	return 0;
}
