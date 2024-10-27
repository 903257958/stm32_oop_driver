#include "key.h"
#include "delay.h"

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

	#if !FREERTOS
	static void __key_delay_ms(uint32_t ms)
	{
		while(ms--)
		{
			SysTick->LOAD = 72 * 1000;				//设置定时器重装值
			SysTick->VAL = 0x00;					//清空当前计数值
			SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
			while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
			SysTick->CTRL = 0x00000004;				//关闭定时器
		}
	}
	#else
	static void __key_delay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif

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

	#if !FREERTOS
	static void __key_delay_ms(uint32_t ms)
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
			}while((temp & 0x01) && !(temp & (1<<16)));	// 等待时间到达   
			SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; 	// 关闭计数器
			SysTick->VAL = 0X00;       					// 清空计数器
		}
	}
	#else		
	static void __key_delay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif

#endif

/* 函数声明 */			
static bool __key_is_press(KeyDev_t *pDev);
static int __key_deinit(KeyDev_t *pDev);

/* 环形缓冲区 */
#define BUF_LEN 10
#define NEXT_POS(x) ((x+1) % BUF_LEN)

typedef struct {
	int value[BUF_LEN];
	int read;
	int write;
}KeyBuf_t;

KeyBuf_t gKeyBuf = {
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
 * @brief	初始化按键
 * @param	pDev		:  KeyDev_t结构体指针
 * @param	port		:  端口
 * @param	pin			:  引脚
 * @param	pressLevel	:  按键按下的时候IO口的电平
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int key_init(KeyDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/*配置时钟与GPIO*/
	__key_config_gpio_clock_enable(pDev->info.port);
	
	if(pDev->info.pressLevel == GPIO_LEVEL_HIGH)			// 根据pressLevel配置为上拉或下拉输入
	{
		__key_config_io_in_pd(pDev->info.port, pDev->info.pin);
	}
	else
	{
		__key_config_io_in_pu(pDev->info.port, pDev->info.pin);
	}
	
	/*函数指针赋值*/
	pDev->is_press = __key_is_press;
	pDev->deinit = __key_deinit;
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	扫描已注册的按键，若有按键按下，放入环形缓冲区，此函数需定时调用
 * @param	dev		:  KeyDev_t结构体数组
 * @param	count   :  已注册按键设备的数量
 * @return	0，表示检测到按键并写入缓冲区，-1，表示未检测到按键
 ******************************************************************************/
int key_scan(KeyDev_t *dev, int count)
{
    int i;

	/* 遍历所有已注册的按键设备 */
    for (i = 0; i < count; i++)
	{
        if (__key_is_press(&dev[i]))	// 读到按键
		{
            __key_buf_write(&gKeyBuf, dev[i].info.value);	// 写入环形缓冲区
			return 0;
        }
    }

	return -1;
}

/******************************************************************************
 * @brief	读取环形缓冲区，得到按键数据
 * @param	无
 * @return	环形缓冲区中的按键数据
 ******************************************************************************/
int key_get_value(void)
{
	return __key_buf_read(&gKeyBuf);	// 读取环形缓冲区
}

/******************************************************************************
 * @brief	判断按键是否被按下
 * @param	pDev   :  KeyDev_t结构体指针
 * @return	true, 表示按键被按下； false，表示按键未被按下。
 ******************************************************************************/
static bool __key_is_press(KeyDev_t *pDev)
{
	if (__key_io_read(pDev->info.port, pDev->info.pin) == pDev->info.pressLevel)
	{
		__key_delay_ms(20);
		while((__key_io_read(pDev->info.port, pDev->info.pin) == pDev->info.pressLevel));
		
		return true;
	}
	
	return false;
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
	
	pDev->initFlag = false;	//修改初始化标志
	
	return 0;
}
