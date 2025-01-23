#include <stdlib.h>
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
											
#elif defined(STM32F40_41xxx)

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
	I2CDev_t mpu6050;		// 软件I2C设备
}MPU6050PrivData_t;

/* 配置中断 */
static void __mpu6050_irq_init(MPU6050Dev_t *pDev);

/* 通信协议 */
static void __mpu6050_write_reg(MPU6050Dev_t *pDev, uint8_t regAddr, uint8_t data);
static uint8_t __mpu6050_read_reg(MPU6050Dev_t *pDev, uint8_t regAddr);

/* 功能函数 */
static uint8_t __mpu6050_get_id(MPU6050Dev_t *pDev);
static void __mpu6050_get_data(MPU6050Dev_t *pDev, MPU6050Data_t *data);
static int __mpu6050_deinit(MPU6050Dev_t *pDev);

/******************************************************************************
 * @brief	初始化MPU6050
 * @param	pDev		:	MPU6050Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int mpu6050_init(MPU6050Dev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 保存私有数据 */
	pDev->pPrivData = (MPU6050PrivData_t *)malloc(sizeof(MPU6050PrivData_t));
	if (!pDev->pPrivData)
		return -1;
	
	MPU6050PrivData_t *pPrivData = (MPU6050PrivData_t *)pDev->pPrivData;
	
	pPrivData->mpu6050.info.SCLPort = pDev->info.SCLPort;
	pPrivData->mpu6050.info.SCLPin = pDev->info.SCLPin;
	pPrivData->mpu6050.info.SDAPort = pDev->info.SDAPort;
	pPrivData->mpu6050.info.SDAPin = pDev->info.SDAPin;
	
	/* 配置软件I2C */
	i2c_init(&pPrivData->mpu6050);

	/* 配置中断 */
	__mpu6050_irq_init(pDev);
	
	/* 函数指针赋值 */
	pDev->get_id = __mpu6050_get_id;
	pDev->get_data = __mpu6050_get_data;
	pDev->deinit = __mpu6050_deinit;
	
	pDev->initFlag = true;
	
	/* MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器 */
	__mpu6050_write_reg(pDev, MPU6050_PWR_MGMT_1, 0x01);		// 电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	__mpu6050_write_reg(pDev, MPU6050_PWR_MGMT_2, 0x00);		// 电源管理寄存器2，保持默认值0，所有轴均不待机
	__mpu6050_write_reg(pDev, MPU6050_SMPLRT_DIV, 0x09);		// 采样率分频寄存器，配置采样率
	__mpu6050_write_reg(pDev, MPU6050_CONFIG, 0x06);			// 配置寄存器，配置DLPF
	__mpu6050_write_reg(pDev, MPU6050_GYRO_CONFIG, 0x18);		// 陀螺仪配置寄存器，选择满量程为±2000°/s
	__mpu6050_write_reg(pDev, MPU6050_ACCEL_CONFIG, 0x18);		// 加速度计配置寄存器，选择满量程为±16g
	
	return 0;
}

/******************************************************************************
 * @brief	MPU6050配置中断
 * @param	pDev	:	MPU6050Dev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __mpu6050_irq_init(MPU6050Dev_t *pDev)
{
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
	/* 配置时钟与GPIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	// 开启AFIO时钟
	__mpu6050_config_gpio_clock_enable(pDev->info.INTPort);	// 开启中断GPIO口时钟
	
	__mpu6050_config_io_in_pu(pDev->info.INTPort, pDev->info.INTPin);	// 上拉输入
	
	/* 配置AFIO */
	GPIO_EXTILineConfig(	__mpu6050_exti_get_port_source(pDev->info.INTPort), 
							__mpu6050_exti_get_pin_source(pDev->info.INTPin)	);
	
	#elif defined(STM32F40_41xxx)
	
	/*配置时钟与GPIO*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	// 开启SYSCFG时钟
	__mpu6050_config_gpio_clock_enable(pDev->info.INTPort);	// 开启中断GPIO口时钟
	
	__mpu6050_config_io_in_pu(pDev->info.INTPort, pDev->info.INTPin);	// 上拉输入
							
	/* 配置SYSCFG */
	SYSCFG_EXTILineConfig(	__mpu6050_exti_get_port_source(pDev->info.INTPort), 
							__mpu6050_exti_get_pin_source(pDev->info.INTPin)	);
							
	#endif
							
	/* 配置EXTI */
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = __mpu6050_get_exti_line(pDev->info.INTPin);
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);
	
	/* 配置NVIC */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = __mpu6050_get_exti_irqn(pDev->info.INTPin);
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	/* 注册外部中断回调函数 */
	irq_handler_register(	__mpu6050_get_exti_line(pDev->info.INTPin), 
							pDev->info.irq_callback	);
	
	__mpu6050_write_reg(pDev, MPU6050_INT_PIN_CFG, 0x00);		// 配置中断引脚
	__mpu6050_write_reg(pDev, MPU6050_INT_ENABLE, 0xFF);		// 使能所有中断
}

/******************************************************************************
 * @brief	MPU6050写寄存器
 * @param	pDev		:	MPU6050Dev_t 结构体指针
 * @param	regAddr		:	寄存器地址，范围：参考MPU6050手册的寄存器描述
 * @param	data		:	要写入寄存器的数据，范围：0x00~0xFF
 * @return	无
 ******************************************************************************/
static void __mpu6050_write_reg(MPU6050Dev_t *pDev, uint8_t regAddr, uint8_t data)
{
	MPU6050PrivData_t *pPrivData = (MPU6050PrivData_t *)pDev->pPrivData;
	
	pPrivData->mpu6050.start(&pPrivData->mpu6050);						// I2C起始
	pPrivData->mpu6050.send_byte(&pPrivData->mpu6050, MPU6050_ADDRESS);	// 发送从机地址，读写位为0，表示即将写入
	pPrivData->mpu6050.recv_ack(&pPrivData->mpu6050);					// 接收应答
	pPrivData->mpu6050.send_byte(&pPrivData->mpu6050, regAddr);			// 发送寄存器地址
	pPrivData->mpu6050.recv_ack(&pPrivData->mpu6050);					// 接收应答
	pPrivData->mpu6050.send_byte(&pPrivData->mpu6050, data);			// 发送要写入寄存器的数据
	pPrivData->mpu6050.recv_ack(&pPrivData->mpu6050);					// 接收应答
	pPrivData->mpu6050.stop(&pPrivData->mpu6050);						// I2C终止					
}

/******************************************************************************
 * @brief	MPU6050读寄存器
 * @param	pDev		:	MPU6050Dev_t 结构体指针
 * @param	regAddr		:	寄存器地址，范围：参考MPU6050手册的寄存器描述
 * @return	读取寄存器的数据，范围：0x00~0xFF
 ******************************************************************************/
static uint8_t __mpu6050_read_reg(MPU6050Dev_t *pDev, uint8_t regAddr)
{
	MPU6050PrivData_t *pPrivData = (MPU6050PrivData_t *)pDev->pPrivData;
	
	uint8_t data;
	
	pPrivData->mpu6050.start(&pPrivData->mpu6050);								// I2C起始
	pPrivData->mpu6050.send_byte(&pPrivData->mpu6050, MPU6050_ADDRESS);			// 发送从机地址，读写位为0，表示即将写入
	pPrivData->mpu6050.recv_ack(&pPrivData->mpu6050);							// 接收应答
	pPrivData->mpu6050.send_byte(&pPrivData->mpu6050, regAddr);					// 发送寄存器地址
	pPrivData->mpu6050.recv_ack(&pPrivData->mpu6050);							// 接收应答
	
	pPrivData->mpu6050.start(&pPrivData->mpu6050);								// I2C重复起始
	pPrivData->mpu6050.send_byte(&pPrivData->mpu6050, MPU6050_ADDRESS | 0x01);	// 发送从机地址，读写位为1，表示即将读取
	pPrivData->mpu6050.recv_ack(&pPrivData->mpu6050);							// 接收应答
	data = pPrivData->mpu6050.recv_byte(&pPrivData->mpu6050);					// 接收指定寄存器的数据
	pPrivData->mpu6050.send_ack(&pPrivData->mpu6050, 1);						// 发送应答，给从机非应答，终止从机的数据输出
	pPrivData->mpu6050.stop(&pPrivData->mpu6050);								// I2C终止					
	
	return data;
}

/******************************************************************************
 * @brief	MPU6050获取ID号
 * @param	pDev		:	MPU6050Dev_t 结构体指针
 * @return	MPU6050的ID号
 ******************************************************************************/
static uint8_t __mpu6050_get_id(MPU6050Dev_t *pDev)
{
	return __mpu6050_read_reg(pDev, MPU6050_WHO_AM_I);		// 返回WHO_AM_I寄存器的值
}

/******************************************************************************
 * @brief	MPU6050获取数据
 * @param	pDev		:	MPU6050Dev_t 结构体指针
 * @param	data		:	MPU6050Data_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __mpu6050_get_data(MPU6050Dev_t *pDev, MPU6050Data_t *data)
{
	int16_t tempVal;
	uint8_t dataH, dataL;									// 定义数据高8位和低8位的变量
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_ACCEL_XOUT_H);	// 读取加速度计X轴的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_ACCEL_XOUT_L);	// 读取加速度计X轴的低8位数据
	data->accX = (dataH << 8) | dataL;						// 数据拼接
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_ACCEL_YOUT_H);	// 读取加速度计Y轴的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_ACCEL_YOUT_L);	// 读取加速度计Y轴的低8位数据
	data->accY = (dataH << 8) | dataL;						// 数据拼接
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_ACCEL_ZOUT_H);	// 读取加速度计Z轴的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_ACCEL_ZOUT_L);	// 读取加速度计Z轴的低8位数据
	data->accZ = (dataH << 8) | dataL;						// 数据拼接
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_GYRO_XOUT_H);	// 读取陀螺仪X轴的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_GYRO_XOUT_L);	// 读取陀螺仪X轴的低8位数据
	data->gyroX = (dataH << 8) | dataL;						// 数据拼接
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_GYRO_YOUT_H);	// 读取陀螺仪Y轴的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_GYRO_YOUT_L);	// 读取陀螺仪Y轴的低8位数据
	data->gyroY = (dataH << 8) | dataL;						// 数据拼接
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_GYRO_ZOUT_H);	// 读取陀螺仪Z轴的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_GYRO_ZOUT_L);	// 读取陀螺仪Z轴的低8位数据
	data->gyroZ = (dataH << 8) | dataL;						// 数据拼接
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_TEMP_OUT_H);	// 读取温度值的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_TEMP_OUT_L);	// 读取温度值的低8位数据
	tempVal = ((dataH << 8) | dataL);						// 数据拼接
	data->temp = (float)(tempVal)/340.0f + 36.53f;			// 计算温度值
}

/******************************************************************************
 * @brief	去初始化MPU6050
 * @param	pDev   :  MPU6050Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __mpu6050_deinit(MPU6050Dev_t *pDev)
{    
    if (!pDev || !pDev->initFlag)
        return -1;
    
    MPU6050PrivData_t *pPrivData = (MPU6050PrivData_t *)pDev->pPrivData;
	
	/* 去初始化软件I2C */
	pPrivData->mpu6050.deinit(&pPrivData->mpu6050);
	
	/* 释放私有数据内存 */
	free(pDev->pPrivData);
	pDev->pPrivData = NULL;
	
	pDev->initFlag = false;	// 修改初始化标志
    
    return 0;
}
