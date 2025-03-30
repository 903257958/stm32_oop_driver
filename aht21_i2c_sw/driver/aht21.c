#include "delay.h"
#include "aht21.h"

/* AHT21私有数据结构体 */
typedef struct {
	I2CDev_t i2c;	// 软件I2C设备
}AHT21PrivData_t;

/* 函数声明 */
static int __aht21_write_regs(AHT21Dev_t *dev, uint8_t addr, uint8_t num, uint8_t data[]);
static int __aht21_read_reg(AHT21Dev_t *dev, uint8_t addr, uint8_t *data);
static int __aht21_read_curr_addr_regs(AHT21Dev_t *dev, uint8_t num, uint8_t data[]);
static int __aht21_get_data(AHT21Dev_t *dev);
static int __aht21_deinit(AHT21Dev_t *dev);

/******************************************************************************
 * @brief	初始化AHT21
 * @param	dev	:  AHT21Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int aht21_init(AHT21Dev_t *dev)
{
    if (!dev)
		return -1;

	uint8_t bits;
	uint8_t datas_init[2] = {0x08, 0x00};

    /* 保存私有数据 */
	dev->priv_data = (AHT21PrivData_t *)malloc(sizeof(AHT21PrivData_t));
	if (!dev->priv_data)
		return -1;
	
	AHT21PrivData_t *priv_data = (AHT21PrivData_t *)dev->priv_data;
	
	priv_data->i2c.info.scl_port = dev->info.scl_port;
	priv_data->i2c.info.scl_pin = dev->info.scl_pin;
	priv_data->i2c.info.sda_port = dev->info.sda_port;
	priv_data->i2c.info.sda_pin = dev->info.sda_pin;
	
	/* 配置软件I2C */
	i2c_init(&priv_data->i2c);

	dev->init_flag = true;

	/* AHT21硬件初始化 */
	delay_ms(40);
	__aht21_read_reg(dev, AHT21_GET_STATUS, &bits);
	if (!((bits >> 3) & 0x01))
	{
		__aht21_write_regs(dev, AHT21_INIT, 2, datas_init);
		delay_ms(10);
	}

    /* 函数指针赋值 */
	dev->get_data = __aht21_get_data;
	dev->deinit = __aht21_deinit;
    
	return 0;
}

/******************************************************************************
 * @brief	AHT21指定地址写多个寄存器
 * @param	dev		:  AHT21Dev_t 结构体指针
 * @param	addr	:  要写入的寄存器首地址
 * @param	num		:  要写入的寄存器个数
 * @param	data	:  要写入的寄存器数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __aht21_write_regs(AHT21Dev_t *dev, uint8_t addr, uint8_t num, uint8_t data[])
{
	if (!dev || !dev->init_flag)
		return -1;

	uint8_t i;

    AHT21PrivData_t *priv_data = (AHT21PrivData_t *)dev->priv_data;

    priv_data->i2c.start(&priv_data->i2c);						// I2C起始
	priv_data->i2c.send_byte(&priv_data->i2c, AHT21_ADDRESS << 1);	// 发送从机地址，读写位为0，表示即将写入
	priv_data->i2c.recv_ack(&priv_data->i2c);					// 接收应答
	priv_data->i2c.send_byte(&priv_data->i2c, addr);	        // 发送寄存器地址
	priv_data->i2c.recv_ack(&priv_data->i2c);					// 接收应答

	for (i = 0; i < num; i++)
	{
		priv_data->i2c.send_byte(&priv_data->i2c, data[i]);			// 发送要写入寄存器的数据
		priv_data->i2c.recv_ack(&priv_data->i2c);					// 接收应答
	}
	
	priv_data->i2c.stop(&priv_data->i2c);						// I2C终止

	return 0;
}

/******************************************************************************
 * @brief	AHT21指定地址读寄存器
 * @param	dev		:   AHT21Dev_t 结构体指针
 * @param	addr    :   要读的寄存器地址
 * @param	data    :   要读的寄存器数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __aht21_read_reg(AHT21Dev_t *dev, uint8_t addr, uint8_t *data)
{
	if (!dev || !dev->init_flag)
		return -1;
	
    AHT21PrivData_t *priv_data = (AHT21PrivData_t *)dev->priv_data;
	
	priv_data->i2c.start(&priv_data->i2c);								// I2C起始
	priv_data->i2c.send_byte(&priv_data->i2c, AHT21_ADDRESS << 1);		// 发送从机地址，读写位为0，表示即将写入
	priv_data->i2c.recv_ack(&priv_data->i2c);							// 接收应答
	priv_data->i2c.send_byte(&priv_data->i2c, addr);					// 发送寄存器地址
	priv_data->i2c.recv_ack(&priv_data->i2c);							// 接收应答
	
	priv_data->i2c.start(&priv_data->i2c);								// I2C重复起始
	priv_data->i2c.send_byte(&priv_data->i2c, (AHT21_ADDRESS << 1) | 0x01);	// 发送从机地址，读写位为1，表示即将读取
	priv_data->i2c.recv_ack(&priv_data->i2c);							// 接收应答
	*data = priv_data->i2c.recv_byte(&priv_data->i2c);					// 接收指定寄存器的数据
	priv_data->i2c.send_ack(&priv_data->i2c, 1);						// 发送应答，给从机非应答，终止从机的数据输出
	priv_data->i2c.stop(&priv_data->i2c);								// I2C终止		
	
	return 0;
}

/******************************************************************************
 * @brief	AHT21当前地址读多个寄存器
 * @param	dev    	:   AHT21Dev_t 结构体指针
 * @param	num		:   要读的寄存器个数
 * @param	data    :   要读的寄存器数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __aht21_read_curr_addr_regs(AHT21Dev_t *dev, uint8_t num, uint8_t data[])
{
	if (!dev || !dev->init_flag)
		return -1;
	
	uint8_t i;

    AHT21PrivData_t *priv_data = (AHT21PrivData_t *)dev->priv_data;
	
	priv_data->i2c.start(&priv_data->i2c);								// I2C起始
	priv_data->i2c.send_byte(&priv_data->i2c, (AHT21_ADDRESS << 1) | 0x01);	// 发送从机地址，读写位为1，表示即将读取
	priv_data->i2c.recv_ack(&priv_data->i2c);							// 接收应答

	for (i = 0; i < num; i++)
    {
        data[i] = priv_data->i2c.recv_byte(&priv_data->i2c);			// 接收数据
        if (i == num - 1)
        {
            priv_data->i2c.send_ack(&priv_data->i2c, 1);				// 发送非应答信号
        }
        else
        {
            priv_data->i2c.send_ack(&priv_data->i2c, 0);				// 发送应答信号
        }
    }

	priv_data->i2c.stop(&priv_data->i2c);								// I2C终止
	
	return 0;
}

/******************************************************************************
 * @brief	AHT21获取温湿度
 * @param	dev    	:   AHT21Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __aht21_get_data(AHT21Dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	uint8_t bits, timeout = 100;
	uint8_t datas_measure[2] = {0x33, 0x00};
	uint8_t datas[6] = {0};
	uint32_t temp_t, humi_t;

	/* 触发测量 */
	__aht21_write_regs(dev, AHT21_MEASURE, 2, datas_measure);

	/* 等待测量完成 */
	while (timeout > 0)
	{
		__aht21_read_reg(dev, AHT21_GET_STATUS, &bits);
		if ((bits & 0x80) == 0)
		{
			break;
		}

		timeout -= 10;
		delay_ms(10);
	}
	if (timeout == 0)
	{
		return -2;
	}
	
	/* 读取数据 */
	__aht21_read_curr_addr_regs(dev, 6, datas);

	/* 数据转换 */
	humi_t = (datas[1] << 12) + (datas[2] << 4) + (datas[3] >> 4);		// 拼接完成的20位湿度数据
	dev->humi = (humi_t * 1000 >> 20);									// 湿度数据百分比乘10
	dev->humi = dev->humi / 10;											// 湿度数据百分比
	temp_t = ((datas[3] & 0x0F) << 16) + (datas[4] << 8) + datas[5];	// 拼接完成的20位温度数据
	dev->temp = (temp_t * 2000 >> 20) - 500;							// 温度数据乘10
	dev->temp = dev->temp / 10;											// 温度数据

	return 0;
}

/******************************************************************************
 * @brief	去初始化AHT21
 * @param	dev   :  AHT21Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __aht21_deinit(AHT21Dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
    dev->priv_data = NULL;

	dev->init_flag = false;	// 修改初始化标志

    return 0;
}
