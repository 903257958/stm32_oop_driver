#include "key.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
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

#elif defined(STM32F40_41xxx)

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

#define MAX_KEY_NUM	10

static KeyDev_t *gKeyDev[MAX_KEY_NUM];	// 存储已注册按键设备，用于中断回调函数传参
static uint8_t gKeyNum = 0; 			// 已注册按键设备数

/* 函数声明 */			
static int __key_deinit(KeyDev_t *pDev);

/* 环形缓冲区 */
#define BUF_LEN 10
#define NEXT_POS(x) ((x+1) % BUF_LEN)

typedef struct {
	int value[BUF_LEN];
	int read;
	int write;
}KeyBuf_t;

static KeyBuf_t gKeyBuf = {
	.read = 0,
	.write = 0
};

/******************************************************************************
 * @brief	判断环形缓冲区空
 * @param	pKeyBuf	:  KeyBuf_t结构体指针
 * @return	true 表示空, false 表示不空
 ******************************************************************************/
static bool __key_buf_is_empty(KeyBuf_t *pKeyBuf)
{
	return (pKeyBuf->read == pKeyBuf->write);
}

/******************************************************************************
 * @brief	判断环形缓冲区满
 * @param	pKeyBuf	:  KeyBuf_t结构体指针
 * @return	true 表示满, false 表示不满
 ******************************************************************************/
static bool __key_buf_is_full(KeyBuf_t *pKeyBuf)
{
	return (pKeyBuf->read == NEXT_POS(pKeyBuf->write));
}

/******************************************************************************
 * @brief	环形缓冲区写数据
 * @param	pKeyBuf	:  KeyBuf_t结构体指针
 * @param	value	:  按键数据
 * @return	无
 ******************************************************************************/
static void __key_buf_write(KeyBuf_t *pKeyBuf, int value)
{
	if (!__key_buf_is_full(pKeyBuf))
	{
		pKeyBuf->value[pKeyBuf->write] = value;
		pKeyBuf->write = NEXT_POS(pKeyBuf->write);
	}
}

/******************************************************************************
 * @brief	环形缓冲区读数据
 * @param	pKeyBuf	:  KeyBuf_t结构体指针
 * @param	value	:  按键数据
 * @return	读取到的按键值
 ******************************************************************************/
static int __key_buf_read(KeyBuf_t *pKeyBuf)
{
	int value = 0;

	if (!__key_buf_is_empty(pKeyBuf))
	{
		value = pKeyBuf->value[pKeyBuf->read];
		pKeyBuf->read = NEXT_POS(pKeyBuf->read);
	}

	return value;
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
	for (i = 0; i < gKeyNum; i++)
	{
		if (__key_io_read(gKeyDev[i]->info.port, gKeyDev[i]->info.pin) == gKeyDev[i]->info.pressLevel)
		{
			return gKeyDev[i]->info.value;
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
	static int8_t currState, prevState;
	uint8_t keyValue;

	/* 定时中断为1ms，每20ms检测一次按键状态，同时可以软件消抖 */
	if (++cnt >= 20)
	{
		cnt = 0;
		
		prevState = currState;			// 上次按键状态
		currState = __key_get_state();	// 本次按键状态
		
		/* 若本次查询无按键按下，上次有按键按下，则说明此按键已松开，记录按键值 */
		if (currState == -1 && prevState != -1)
		{
			keyValue = prevState;
			__key_buf_write(&gKeyBuf, keyValue);	// 写入环形缓冲区
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
	return __key_buf_read(&gKeyBuf);
}
										
/******************************************************************************
 * @brief	初始化按键
 * @param	pDev	:  KeyDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int key_init(KeyDev_t *pDev)
{
	if (!pDev || gKeyNum >= MAX_KEY_NUM)
		return -1;
	
	/* 配置时钟与GPIO */
	__key_config_gpio_clock_enable(pDev->info.port);
	
	if(pDev->info.pressLevel == GPIO_LEVEL_HIGH)			// 根据pressLevel配置为上拉或下拉输入
	{
		__key_config_io_in_pd(pDev->info.port, pDev->info.pin);
	}
	else
	{
		__key_config_io_in_pu(pDev->info.port, pDev->info.pin);
	}
	
	/* 配置定时器 */
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	timerKeyTick.info.psc = 71;
	#elif defined(STM32F40_41xxx)
	timerKeyTick.info.psc = 83;		// 计数周期1us
	#endif

	timerKeyTick.info.arr = 999;	// 定时周期1ms

	timerKeyTick.info.irq_callback = __key_tick;	// 注册回调函数

	timer_init(&timerKeyTick);

	/* 函数指针赋值 */
	pDev->deinit = __key_deinit;
	
	pDev->initFlag = true;
	gKeyDev[gKeyNum] = pDev;	// 保存本次注册的按键设备地址
	gKeyNum++;					// 按键设备数加1
	return 0;
}

/******************************************************************************
 * @brief	去初始化按键
 * @param	pDev   :  KeyDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __key_deinit(KeyDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;

	int i;
	
	timerKeyTick.deinit(&timerKeyTick);

	pDev->initFlag = false;	// 修改初始化标志

	/* 从按键设备数组中移除该设备 */
    for (i = 0; i < gKeyNum; i++)
    {
        if (gKeyDev[i] == pDev)
        {
            gKeyDev[i] = NULL;
            break;
        }
    }
	
	return 0;
}
