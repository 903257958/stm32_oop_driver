#include "main.h"

UARTDev_t debug = {.info = {
	USART1, 115200,
	GPIOA, GPIO_Pin_9,
	GPIOA, GPIO_Pin_10
}};
MPU6050Dev_t mpu6050 = {.info = {
	GPIOB, GPIO_Pin_6, 
	GPIOB, GPIO_Pin_7, 
}};

int main(void)
{
    delay_init(168);
	uart_init(&debug);
	mpu6050_init(&mpu6050);
	
	while (1)
	{
		/* 获取数据 */
		mpu6050.get_data(&mpu6050, &mpu6050.data);
		
		/* 发送数据 */
		debug.printf(&debug, "\r\nid = 0x%x, temperature = %f\r\n", mpu6050.get_id(&mpu6050), mpu6050.data.temp);
		debug.printf(&debug, "accx = %d, accy = %d, accz = %d\r\n", mpu6050.data.accx, mpu6050.data.accy, mpu6050.data.accz);
		debug.printf(&debug, "gyrox = %d, gyroy = %d, gyroz = %d\r\n", mpu6050.data.gyrox, mpu6050.data.gyroy, mpu6050.data.gyroz);

		delay_ms(500);
	}
}
