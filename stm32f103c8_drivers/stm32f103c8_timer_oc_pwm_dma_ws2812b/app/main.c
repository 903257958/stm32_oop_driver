#include "main.h"

ws2812b_dev_t ws2812b = {
    .config = {TIM2, 1, GPIOA, GPIO_Pin_0, 60}
};

int main(void)
{
    uint32_t color_grb_buf[60] = {0};
    uint8_t i;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(72);
	ws2812b_init(&ws2812b);

    for (i = 0; i < ws2812b.config.led_num; i++)
    {
        color_grb_buf[i] = 0xFFD9A0;
    }

    ws2812b.off(&ws2812b);
    delay_s(1);
    ws2812b.set_color(&ws2812b, color_grb_buf);

	while (1)
	{

	}
}
