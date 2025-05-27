#include "main.h"

UARTDev_t debug = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
BMP280Dev_t bmp280 = {.config = {GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7,}};

int main(void)
{
    uint8_t id;

    delay_init(168);
	uart_init(&debug);
	bmp280_init(&bmp280);

    bmp280.get_id(&bmp280, &id);
    debug.printf("ID: 0x%x\r\n", id);
    
	while (1)
	{
        bmp280.get_data(&bmp280);

        debug.printf("temperature: %.2fC, pressure: %.2fhPa\r\n", bmp280.data.temperature, bmp280.data.pressure);

        delay_ms(1000);
	}
}
