#include "i2c.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__i2c_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
													else					{i2c_log("i2c gpio clock no enable\r\n");} \
												}
													
#define	__i2c_config_io_out_od(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}
											
#define	__i2c_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)
												
#define __i2c_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

	#if !FREERTOS		
	static void __i2c_delay_us(uint32_t us)
	{
		if(!us)	return;
		SysTick->LOAD = 72 * us;				// 设置定时器重装值
		SysTick->VAL = 0x00;					// 清空当前计数值
		SysTick->CTRL = 0x00000005;				// 设置时钟源为HCLK，启动定时器
		while(!(SysTick->CTRL & 0x00010000));	// 等待计数到0
		SysTick->CTRL = 0x00000004;				// 关闭定时器
	}
	#else		
	/*	FreeRTOS的SysTick被用于任务调度，故改为硬件定时器配置毫秒级延时
		硬件定时器定义在该文件外部，示例如下：
		TIMDev_t tim_delay = {.info = {TIM2, 71, 49999, NULL}};
		TIM_Init(&tim_delay);	*/
	static void __i2c_delay_us(uint32_t us)
	{
		timerDelay.delay_us(&timerDelay, us);
	}							  
	#endif

#elif defined(STM32F40_41xxx) || defined(STM32F411xE)

#define	__i2c_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
													else					{i2c_log("i2c gpio clock no enable\r\n");} \
												}
													
#define	__i2c_config_io_out_od(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}
											
#define	__i2c_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)
												
#define __i2c_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

	#if !FREERTOS		
	static void __i2c_delay_us(uint32_t us)
	{
		#if defined(STM32F40_41xxx)
		uint32_t temp;	    	 
		SysTick->LOAD = us * 21; 					// 时间加载	  		 
		SysTick->VAL=0x00;        					// 清空计数器
		SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ; 	// 开始倒数 	 
		do
		{
			temp = SysTick->CTRL;
		}while((temp&0x01) && !(temp&(1<<16)));		// 等待时间到达   
		SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	// 关闭计数器
		SysTick->VAL = 0X00;       					// 清空计数器 
		#elif defined(STM32F411xE)
		uint32_t temp;
		SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // 使用系统时钟（100MHz）
		SysTick->LOAD = us * 100;                   // 100MHz时钟，每微秒100个时钟周期
		SysTick->VAL = 0x00;                         // 清空计数器
		SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;    // 开始倒数
		do
		{
			temp = SysTick->CTRL;
		} while ((temp & 0x01) && !(temp & (1 << 16))); // 等待时间到达
		SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;   // 关闭计数器
		SysTick->VAL = 0x00;                         // 清空计数器
		#endif
	}
	#else		
	/*	FreeRTOS的SysTick被用于任务调度，故改为硬件定时器配置毫秒级延时
		硬件定时器定义在该文件外部，示例如下：
		TIMDev_t tim_delay = {.info = {TIM2, 71, 49999, NULL}};
		TIM_Init(&tim_delay);	*/
	static void __i2c_delay_us(uint32_t us)
	{
		timerDelay.delay_us(&timerDelay, us);
	}							  
	#endif

#endif
		
static int __i2c_scl_write(I2CDev_t *pDev, uint8_t level);
static int __i2c_sda_write(I2CDev_t *pDev, uint8_t level);
static uint8_t __i2c_sda_read(I2CDev_t *pDev);
static int __i2c_start(I2CDev_t *pDev);
static int __i2c_stop(I2CDev_t *pDev);
static int __i2c_send_byte(I2CDev_t *pDev, uint8_t byte);
static uint8_t __i2c_recv_byte(I2CDev_t *pDev);
static int __i2c_send_ack(I2CDev_t *pDev, uint8_t ack);
static uint8_t __i2c_recv_ack(I2CDev_t *pDev);
static uint8_t __i2c_check(I2CDev_t *pDev, uint8_t addr);
static int __i2c_deinit(I2CDev_t *pDev);
	
/******************************************************************************
 * @brief	初始化软件I2C
 * @param	pDev	:  I2CDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int i2c_init(I2CDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 配置时钟与GPIO */
	__i2c_config_gpio_clock_enable(pDev->info.SCLPort);
	__i2c_config_gpio_clock_enable(pDev->info.SDAPort);
	
	__i2c_config_io_out_od(pDev->info.SCLPort, pDev->info.SCLPin);	// SCL开漏输出
	__i2c_config_io_out_od(pDev->info.SDAPort, pDev->info.SDAPin);	// SDA开漏输出
	
	/* 函数指针赋值 */
	pDev->start = __i2c_start;
	pDev->stop = __i2c_stop;
	pDev->send_byte = __i2c_send_byte;
	pDev->recv_byte = __i2c_recv_byte;
	pDev->send_ack = __i2c_send_ack;
	pDev->recv_ack = __i2c_recv_ack;
	pDev->check_dev = __i2c_check;
	pDev->deinit = __i2c_deinit;
	
	/* 起始SCL与SDA均置高电平 */
	__i2c_scl_write(pDev, 1);
	__i2c_sda_write(pDev, 1);
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	软件I2C SCL线置高/低电平
 * @param	pDev	:  I2CDev_t结构体指针
 * @param	level	:  要写入的电平值，取1/0
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __i2c_scl_write(I2CDev_t *pDev, uint8_t level)
{
	if (!pDev->initFlag)
		return -1;
	
	__i2c_io_write(pDev->info.SCLPort, pDev->info.SCLPin, level);
	__i2c_delay_us(1);
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C SDA线置高/低电平
 * @param	pDev	:  I2CDev_t结构体指针
 * @param	level	:  要写入的电平值，取1/0
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __i2c_sda_write(I2CDev_t *pDev, uint8_t level)
{
	if (!pDev->initFlag)
		return -1;
	
	__i2c_io_write(pDev->info.SDAPort, pDev->info.SDAPin, level);
	__i2c_delay_us(1);
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C SDA线读高/低电平
 * @param	pDev	:  I2CDev_t结构体指针
 * @return	SDA的电平值，为1/0
 ******************************************************************************/
static uint8_t __i2c_sda_read(I2CDev_t *pDev)
{
	uint8_t level;
	level = __i2c_io_read(pDev->info.SDAPort, pDev->info.SDAPin);
	__i2c_delay_us(1);
	
	return level;
}

/******************************************************************************
 * @brief	软件I2C起始
 * @param	pDev	:  I2CDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __i2c_start(I2CDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	__i2c_sda_write(pDev, 1);
	__i2c_scl_write(pDev, 1);
	__i2c_sda_write(pDev, 0);
	__i2c_scl_write(pDev, 0);
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C停止
 * @param	pDev	:  I2CDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __i2c_stop(I2CDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	__i2c_scl_write(pDev, 0);
	__i2c_sda_write(pDev, 0);
	__i2c_scl_write(pDev, 1);
	__i2c_sda_write(pDev, 1);
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C发送一个字节
 * @param	pDev	:  I2CDev_t结构体指针
 * @param	byte	:  要发送的字节
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __i2c_send_byte(I2CDev_t *pDev, uint8_t byte)
{
	if (!pDev)
		return -1;
	
	for(uint8_t i = 0;i < 8;i++)
	{
		__i2c_sda_write(pDev, (byte & (0x80 >> i)));	// 高位先行
		__i2c_scl_write(pDev, 1);						// SCL原来是低电平，放完数据拉高SCL，等待从机读取
		__i2c_scl_write(pDev, 0);						// 从机读取完毕，SCL置低电平，完成一位发送
	}
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C接收一个字节
 * @param	pDev	:  I2CDev_t结构体指针
 * @return	接收到的字节
 ******************************************************************************/
static uint8_t __i2c_recv_byte(I2CDev_t *pDev)
{
	uint8_t data = 0x00;
	
	__i2c_sda_write(pDev, 1);					// 主机释放SDA，等待从机放入数据
	for(int i = 7;i >= 0;i--)
	{
		__i2c_scl_write(pDev, 1);				// SCL原来是低电平，从机放完数据SCL置高电平，主机开始读取
		data |= (__i2c_sda_read(pDev) << i);	// 高位先行
		__i2c_scl_write(pDev, 0);				// 主机读取完毕，SCL置低电平，完成一位接收
	}
	
	return data;
}

/******************************************************************************
 * @brief	软件I2C发送应答位
 * @param	pDev	:  I2CDev_t结构体指针
 * @param	ack		:  要发送的应答位
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __i2c_send_ack(I2CDev_t *pDev, uint8_t ack)
{
	if (!pDev)
		return -1;
	
	__i2c_sda_write(pDev, ack);
	__i2c_scl_write(pDev, 1);		// SCL原来是低电平，放完数据拉高SCL，等待从机读取
	__i2c_scl_write(pDev, 0);		// 从机读取完毕，SCL置低电平，完成发送
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C接收应答位
 * @param	pDev	:  I2CDev_t结构体指针
 * @return	接收到的应答位，0为应答，1为非应答
 ******************************************************************************/
static uint8_t __i2c_recv_ack(I2CDev_t *pDev)
{
	uint8_t ack;
	
	__i2c_sda_write(pDev, 1);		// 主机释放SDA，等待从机放入数据
	__i2c_scl_write(pDev, 1);		// SCL原来是低电平，从机放完数据SCL置高电平，主机开始读取
	ack = __i2c_sda_read(pDev);
	__i2c_scl_write(pDev, 0);		// 主机读取完毕，SCL置低电平，完成接收
	
	return ack;
}

/******************************************************************************
 * @brief	软件I2C检查已连接的设备
 * @param	pDev	:  I2CDev_t结构体指针
 * @param	addr	:  要检查设备的地址		//MPU6050:0xD0
 * @return	接收到的应答位，0为应答，1为非应答
 ******************************************************************************/
static uint8_t __i2c_check(I2CDev_t *pDev, uint8_t addr)
{
	uint8_t ack;
	
	__i2c_start(pDev);
	__i2c_send_byte(pDev, addr);	
	ack = __i2c_recv_ack(pDev);
	__i2c_stop(pDev);
	
	return ack;
}

/******************************************************************************
 * @brief	去初始化软件I2C
 * @param	pDev   :  I2CDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __i2c_deinit(I2CDev_t *pDev)
{    
    if (!pDev || !pDev->initFlag)
        return -1;
	
	pDev->initFlag = false;	// 修改初始化标志
    
    return 0;
}
