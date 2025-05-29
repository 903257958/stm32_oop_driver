#include "main.h"

gpio_dev_t gpio = {.config = {GPIOC, GPIO_Pin_13, GPIO_MODE_OUT_PP}};

int main(void)
{
	gpio_init(&gpio);

	gpio.set(&gpio);

	while (1)
	{
		
	}
}
