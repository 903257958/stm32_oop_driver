#include "main.h"

oled_dev_t oled = {.config = {
	SPI1,
	GPIOA, GPIO_Pin_5,
	GPIOA, GPIO_Pin_7,
	GPIOB, GPIO_Pin_0,
	GPIOA, GPIO_Pin_6,
	GPIOB, GPIO_Pin_4,
	2,
	SPI_MODE_0
}};

int main(void)
{
	int i;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* 解除JTAG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	DBGMCU->CR &= ~((uint32_t)1 << 5);
    
    delay_init(72);
	oled_init(&oled);
	
	while (1)
	{
		i++;
		oled.show_num(&oled, 0, 0, i, 8, OLED_8X16);
		oled.update(&oled);
	}
}
