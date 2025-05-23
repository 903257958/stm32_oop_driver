#include "main.h"

GPIODev_t gpio1 = {.config = {GPIOA, GPIO_Pin_1, GPIO_MODE_IN_PU}};
GPIODev_t gpio2 = {.config = {GPIOB, GPIO_Pin_1, GPIO_MODE_OUT_PP}};

uint8_t status;

int main(void)
{
	gpio_init(&gpio1);
	gpio_init(&gpio2);

	while(1)
	{
		gpio1.read(&gpio1, &status);
	
        if (status == GPIO_LEVEL_LOW)
        {
            gpio2.set(&gpio2);
        }
        else
        {
            gpio2.reset(&gpio2);
        }
	}
}
