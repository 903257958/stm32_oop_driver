#include "mpu6050.h"
#include "exti.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__mpu6050_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
														else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
														else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
														else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
														else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
														else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
														else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
														else					{mpu6050_log("mpu6050 gpio clock no enable\r\n");} \
													}
														
#define	__mpu6050_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_InitStructure.GPIO_Pin = pin ; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

#define	__mpu6050_exti_get_port_source(port)	(	port == GPIOA ? GPIO_PortSourceGPIOA : \
													port == GPIOB ? GPIO_PortSourceGPIOB : \
													port == GPIOC ? GPIO_PortSourceGPIOC : \
													port == GPIOD ? GPIO_PortSourceGPIOD : \
													port == GPIOE ? GPIO_PortSourceGPIOE : \
													port == GPIOF ? GPIO_PortSourceGPIOF : \
													port == GPIOG ? GPIO_PortSourceGPIOG : \
													(int)0	)
															
#define	__mpu6050_exti_get_pin_source(pin)	(	pin == GPIO_Pin_0 ? GPIO_PinSource0 : \
												pin == GPIO_Pin_1 ? GPIO_PinSource1 : \
												pin == GPIO_Pin_2 ? GPIO_PinSource2 : \
												pin == GPIO_Pin_3 ? GPIO_PinSource3 : \
												pin == GPIO_Pin_4 ? GPIO_PinSource4 : \
												pin == GPIO_Pin_5 ? GPIO_PinSource5 : \
												pin == GPIO_Pin_6 ? GPIO_PinSource6 : \
												pin == GPIO_Pin_7 ? GPIO_PinSource7 : \
												pin == GPIO_Pin_8 ? GPIO_PinSource8 : \
												pin == GPIO_Pin_9 ? GPIO_PinSource9 : \
												pin == GPIO_Pin_10 ? GPIO_PinSource10 : \
												pin == GPIO_Pin_11 ? GPIO_PinSource11 : \
												pin == GPIO_Pin_12 ? GPIO_PinSource12 : \
												pin == GPIO_Pin_13 ? GPIO_PinSource13 : \
												pin == GPIO_Pin_14 ? GPIO_PinSource14 : \
												pin == GPIO_Pin_15 ? GPIO_PinSource15 : \
												(int)0	)
													
#define	__mpu6050_get_exti_line(pin)	(	pin == GPIO_Pin_0 ? EXTI_Line0 : \
											pin == GPIO_Pin_1 ? EXTI_Line1 : \
											pin == GPIO_Pin_2 ? EXTI_Line2 : \
											pin == GPIO_Pin_3 ? EXTI_Line3 : \
											pin == GPIO_Pin_4 ? EXTI_Line4 : \
											pin == GPIO_Pin_5 ? EXTI_Line5 : \
											pin == GPIO_Pin_6 ? EXTI_Line6 : \
											pin == GPIO_Pin_7 ? EXTI_Line7 : \
											pin == GPIO_Pin_8 ? EXTI_Line8 : \
											pin == GPIO_Pin_9 ? EXTI_Line9 : \
											pin == GPIO_Pin_10 ? EXTI_Line10 : \
											pin == GPIO_Pin_11 ? EXTI_Line11 : \
											pin == GPIO_Pin_12 ? EXTI_Line12 : \
											pin == GPIO_Pin_13 ? EXTI_Line13 : \
											pin == GPIO_Pin_14 ? EXTI_Line14 : \
											pin == GPIO_Pin_15 ? EXTI_Line15 : \
											(int)0	)
												
#define	__mpu6050_get_exti_irqn(pin)	(	pin == GPIO_Pin_0 ? EXTI0_IRQn : \
											pin == GPIO_Pin_1 ? EXTI1_IRQn : \
											pin == GPIO_Pin_2 ? EXTI2_IRQn : \
											pin == GPIO_Pin_3 ? EXTI3_IRQn : \
											pin == GPIO_Pin_4 ? EXTI4_IRQn : \
											pin == GPIO_Pin_5 ? EXTI9_5_IRQn : \
											pin == GPIO_Pin_6 ? EXTI9_5_IRQn : \
											pin == GPIO_Pin_7 ? EXTI9_5_IRQn : \
											pin == GPIO_Pin_8 ? EXTI9_5_IRQn : \
											pin == GPIO_Pin_9 ? EXTI9_5_IRQn : \
											pin == GPIO_Pin_10 ? EXTI15_10_IRQn : \
											pin == GPIO_Pin_11 ? EXTI15_10_IRQn : \
											pin == GPIO_Pin_12 ? EXTI15_10_IRQn : \
											pin == GPIO_Pin_13 ? EXTI15_10_IRQn : \
											pin == GPIO_Pin_14 ? EXTI15_10_IRQn : \
											pin == GPIO_Pin_15 ? EXTI15_10_IRQn : \
											(int)0	)
											
#elif defined(STM32F40_41xxx) || defined(STM32F411xE)

#define	__mpu6050_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
														else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
														else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
														else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
														else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
														else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
														else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
														else					{mpu6050_log("mpu6050 gpio clock no enable\r\n");} \
													}
														
#define	__mpu6050_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
													GPIO_InitStructure.GPIO_Pin = pin; \
													GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}
												
#define	__mpu6050_exti_get_port_source(port)	(	port == GPIOA ? EXTI_PortSourceGPIOA : \
													port == GPIOB ? EXTI_PortSourceGPIOB : \
													port == GPIOC ? EXTI_PortSourceGPIOC : \
													port == GPIOD ? EXTI_PortSourceGPIOD : \
													port == GPIOE ? EXTI_PortSourceGPIOE : \
													port == GPIOF ? EXTI_PortSourceGPIOF : \
													port == GPIOG ? EXTI_PortSourceGPIOG : \
													(int)0	)
															
#define	__mpu6050_exti_get_pin_source(pin)	(	pin == GPIO_Pin_0 ? EXTI_PinSource0 : \
												pin == GPIO_Pin_1 ? EXTI_PinSource1 : \
												pin == GPIO_Pin_2 ? EXTI_PinSource2 : \
												pin == GPIO_Pin_3 ? EXTI_PinSource3 : \
												pin == GPIO_Pin_4 ? EXTI_PinSource4 : \
												pin == GPIO_Pin_5 ? EXTI_PinSource5 : \
												pin == GPIO_Pin_6 ? EXTI_PinSource6 : \
												pin == GPIO_Pin_7 ? EXTI_PinSource7 : \
												pin == GPIO_Pin_8 ? EXTI_PinSource8 : \
												pin == GPIO_Pin_9 ? EXTI_PinSource9 : \
												pin == GPIO_Pin_10 ? EXTI_PinSource10 : \
												pin == GPIO_Pin_11 ? EXTI_PinSource11 : \
												pin == GPIO_Pin_12 ? EXTI_PinSource12 : \
												pin == GPIO_Pin_13 ? EXTI_PinSource13 : \
												pin == GPIO_Pin_14 ? EXTI_PinSource14 : \
												pin == GPIO_Pin_15 ? EXTI_PinSource15 : \
												(int)0	)
												
#define	__mpu6050_get_exti_line(pin)	(	pin == GPIO_Pin_0 ? EXTI_Line0 : \
											pin == GPIO_Pin_1 ? EXTI_Line1 : \
											pin == GPIO_Pin_2 ? EXTI_Line2 : \
											pin == GPIO_Pin_3 ? EXTI_Line3 : \
											pin == GPIO_Pin_4 ? EXTI_Line4 : \
											pin == GPIO_Pin_5 ? EXTI_Line5 : \
											pin == GPIO_Pin_6 ? EXTI_Line6 : \
											pin == GPIO_Pin_7 ? EXTI_Line7 : \
											pin == GPIO_Pin_8 ? EXTI_Line8 : \
											pin == GPIO_Pin_9 ? EXTI_Line9 : \
											pin == GPIO_Pin_10 ? EXTI_Line10 : \
											pin == GPIO_Pin_11 ? EXTI_Line11 : \
											pin == GPIO_Pin_12 ? EXTI_Line12 : \
											pin == GPIO_Pin_13 ? EXTI_Line13 : \
											pin == GPIO_Pin_14 ? EXTI_Line14 : \
											pin == GPIO_Pin_15 ? EXTI_Line15 : \
											(int)0	)
											
#define	__mpu6050_get_exti_irqn(pin)	(	pin == GPIO_Pin_0 ? EXTI0_IRQn : \
											pin == GPIO_Pin_1 ? EXTI1_IRQn : \
											pin == GPIO_Pin_2 ? EXTI2_IRQn : \
											pin == GPIO_Pin_3 ? EXTI3_IRQn : \
											pin == GPIO_Pin_4 ? EXTI4_IRQn : \
											pin == GPIO_Pin_5 ? EXTI9_5_IRQn : \
											pin == GPIO_Pin_6 ? EXTI9_5_IRQn : \
											pin == GPIO_Pin_7 ? EXTI9_5_IRQn : \
											pin == GPIO_Pin_8 ? EXTI9_5_IRQn : \
											pin == GPIO_Pin_9 ? EXTI9_5_IRQn : \
											pin == GPIO_Pin_10 ? EXTI15_10_IRQn : \
											pin == GPIO_Pin_11 ? EXTI15_10_IRQn : \
											pin == GPIO_Pin_12 ? EXTI15_10_IRQn : \
											pin == GPIO_Pin_13 ? EXTI15_10_IRQn : \
											pin == GPIO_Pin_14 ? EXTI15_10_IRQn : \
											pin == GPIO_Pin_15 ? EXTI15_10_IRQn : \
											(int)0	) 
											
#endif

/* MPU6050私有数据结构体 */
typedef struct {
	I2CDev_t i2c;		// 软件I2C设备
}MPU6050PrivData_t;

/* 配置中断 */
static void __mpu6050_irq_init(MPU6050Dev_t *dev);

/* 功能函数 */
static uint8_t __mpu6050_get_id(MPU6050Dev_t *dev);
static int __mpu6050_get_data(MPU6050Dev_t *dev, MPU6050Data_t *data);
static int __mpu6050_deinit(MPU6050Dev_t *dev);

/******************************************************************************
 * @brief	初始化MPU6050
 * @param	dev		:	MPU6050Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int mpu6050_init(MPU6050Dev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 保存私有数据 */
	dev->priv_data = (MPU6050PrivData_t *)malloc(sizeof(MPU6050PrivData_t));
	if (!dev->priv_data)
		return -1;
	
	MPU6050PrivData_t *priv_data = (MPU6050PrivData_t *)dev->priv_data;
	
	priv_data->i2c.info.scl_port = dev->info.scl_port;
	priv_data->i2c.info.scl_pin = dev->info.scl_pin;
	priv_data->i2c.info.sda_port = dev->info.sda_port;
	priv_data->i2c.info.sda_pin = dev->info.sda_pin;
	
	/* 配置软件I2C */
	i2c_init(&priv_data->i2c);

	/* 配置中断 */
	__mpu6050_irq_init(dev);
	
	/* 函数指针赋值 */
	dev->get_id = __mpu6050_get_id;
	dev->get_data = __mpu6050_get_data;
	dev->deinit = __mpu6050_deinit;
	
	dev->init_flag = true;
	
	/* MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器 */
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01);		// 电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_PWR_MGMT_2, 0x00);		// 电源管理寄存器2，保持默认值0，所有轴均不待机
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x09);		// 采样率分频寄存器，配置采样率
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_CONFIG, 0x06);			// 配置寄存器，配置DLPF
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x18);		// 陀螺仪配置寄存器，选择满量程为±2000°/s
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x18);		// 加速度计配置寄存器，选择满量程为±16g
	
	return 0;
}

/******************************************************************************
 * @brief	MPU6050配置中断
 * @param	dev	:	MPU6050Dev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __mpu6050_irq_init(MPU6050Dev_t *dev)
{
	MPU6050PrivData_t *priv_data = (MPU6050PrivData_t *)dev->priv_data;

	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
	/* 配置时钟与GPIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	// 开启AFIO时钟
	__mpu6050_config_gpio_clock_enable(dev->info.int_port);	// 开启中断GPIO口时钟
	
	__mpu6050_config_io_in_pu(dev->info.int_port, dev->info.int_pin);	// 上拉输入
	
	/* 配置AFIO */
	GPIO_EXTILineConfig(	__mpu6050_exti_get_port_source(dev->info.int_port), 
							__mpu6050_exti_get_pin_source(dev->info.int_pin)	);
	
	#elif defined(STM32F40_41xxx) || defined(STM32F411xE)
	
	/*配置时钟与GPIO*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	// 开启SYSCFG时钟
	__mpu6050_config_gpio_clock_enable(dev->info.int_port);	// 开启中断GPIO口时钟
	
	__mpu6050_config_io_in_pu(dev->info.int_port, dev->info.int_pin);	// 上拉输入
							
	/* 配置SYSCFG */
	SYSCFG_EXTILineConfig(	__mpu6050_exti_get_port_source(dev->info.int_port), 
							__mpu6050_exti_get_pin_source(dev->info.int_pin)	);
							
	#endif
							
	/* 配置EXTI */
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = __mpu6050_get_exti_line(dev->info.int_pin);
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);
	
	/* 配置NVIC */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = __mpu6050_get_exti_irqn(dev->info.int_pin);
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	/* 注册外部中断回调函数 */
	irq_handler_register(	__mpu6050_get_exti_line(dev->info.int_pin), 
							dev->info.irq_callback	);

	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, 0x00);	// 配置中断引脚
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_INT_ENABLE, 0xFF);	// 使能所有中断
}

/******************************************************************************
 * @brief	MPU6050获取ID号
 * @param	dev		:	MPU6050Dev_t 结构体指针
 * @return	MPU6050的ID号
 ******************************************************************************/
static uint8_t __mpu6050_get_id(MPU6050Dev_t *dev)
{
	MPU6050PrivData_t *priv_data = (MPU6050PrivData_t *)dev->priv_data;
	uint8_t id;

	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_WHO_AM_I, &id);

	return id;
}

/******************************************************************************
 * @brief	MPU6050获取数据
 * @param	dev		:	MPU6050Dev_t 结构体指针
 * @param	data		:	MPU6050Data_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __mpu6050_get_data(MPU6050Dev_t *dev, MPU6050Data_t *data)
{
	if (!dev || !dev->init_flag)
		return -1;

	MPU6050PrivData_t *priv_data = (MPU6050PrivData_t *)dev->priv_data;

	int16_t temp_val;
	uint8_t data_h, data_l;									// 定义数据高8位和低8位的变量
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, &data_h);	// 读取加速度计X轴的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_L, &data_l);	// 读取加速度计X轴的低8位数据
	data->accx = (data_h << 8) | data_l;					// 数据拼接
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_H, &data_h);	// 读取加速度计Y轴的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_L, &data_l);	// 读取加速度计Y轴的低8位数据
	data->accy = (data_h << 8) | data_l;					// 数据拼接
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_H, &data_h);	// 读取加速度计Z轴的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_L, &data_l);	// 读取加速度计Z轴的低8位数据
	data->accz = (data_h << 8) | data_l;					// 数据拼接
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, &data_h);	// 读取陀螺仪X轴的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_L, &data_l);	// 读取陀螺仪X轴的低8位数据
	data->gyrox = (data_h << 8) | data_l;					// 数据拼接
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_YOUT_H, &data_h);	// 读取陀螺仪Y轴的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_YOUT_L, &data_l);	// 读取陀螺仪Y轴的低8位数据
	data->gyroy = (data_h << 8) | data_l;					// 数据拼接
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_H, &data_h);	// 读取陀螺仪Z轴的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_L, &data_l);	// 读取陀螺仪Z轴的低8位数据
	data->gyroz = (data_h << 8) | data_l;					// 数据拼接
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_TEMP_OUT_H, &data_h);		// 读取温度值的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_TEMP_OUT_L, &data_l);		// 读取温度值的低8位数据
	temp_val = ((data_h << 8) | data_l);					// 数据拼接
	data->temp = (float)(temp_val)/340.0f + 36.53f;			// 计算温度值

	return 0;
}

/******************************************************************************
 * @brief	去初始化MPU6050
 * @param	dev   :  MPU6050Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __mpu6050_deinit(MPU6050Dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;
    
    MPU6050PrivData_t *priv_data = (MPU6050PrivData_t *)dev->priv_data;
	
	/* 去初始化软件I2C */
	priv_data->i2c.deinit(&priv_data->i2c);
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
	dev->priv_data = NULL;
	
	dev->init_flag = false;	// 修改初始化标志
    
    return 0;
}
