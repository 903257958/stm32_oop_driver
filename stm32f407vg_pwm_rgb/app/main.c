#include "main.h"

RGBDev_t rgb = {.config = {
    TIM3, 3, GPIOB, GPIO_Pin_0,
    TIM3, 4, GPIOB, GPIO_Pin_1,
    TIM4, 3, GPIOB, GPIO_Pin_8,
    GPIO_LEVEL_HIGH
}};

int main(void)
{
    uint8_t r, g, b;

    delay_init(168);
	rgb_init(&rgb);
    
    rgb.red(&rgb);
    delay_ms(500);
    rgb.yellow(&rgb);
    delay_ms(500);
    rgb.green(&rgb);
    delay_ms(500);
    rgb.blue(&rgb);
    delay_ms(500);
    rgb.white(&rgb);
    delay_ms(500);
    rgb.off(&rgb);
    delay_ms(500);

	while (1)
	{
        rgb.next_rainbow_color(&rgb, &r, &g, &b);
        rgb.set_color(&rgb, r, g, b);
        delay_ms(5);
	}
}
