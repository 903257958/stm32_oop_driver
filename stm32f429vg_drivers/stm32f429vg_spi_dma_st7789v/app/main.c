#include "drv_delay.h"
#include "drv_spi.h"
#include "drv_st7789v.h"
#include "drv_i2c_soft.h"
#include "drv_cst816t.h"
#include "drv_pwm.h"

/* 硬件设备定义 */
static i2c_soft_dev_t i2c_soft;
static const i2c_soft_cfg_t i2c_soft_cfg = {
    .scl_port     = GPIOD,
    .scl_pin      = GPIO_Pin_2,
    .sda_port     = GPIOD,
    .sda_pin      = GPIO_Pin_1,
    .delay_us     = delay_us,
    .bit_delay_us = 1
};

static int i2c_soft_read_reg_adapter(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    return i2c_soft.ops->read_reg(&i2c_soft, dev_addr, reg_addr, data);
}
static int i2c_soft_read_regs_adapter(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data)
{
    return i2c_soft.ops->read_regs(&i2c_soft, dev_addr, reg_addr, num, data);
}
static cst816t_i2c_ops_t cst816t_i2c_ops = {
    .read_reg  = i2c_soft_read_reg_adapter, 
    .read_regs = i2c_soft_read_regs_adapter
};

static cst816t_dev_t cst816t;
static const cst816t_cfg_t cst816t_cfg = {
	.i2c_ops 	   = &cst816t_i2c_ops,
	.delay_ms 	   = delay_ms,
	.rst_port 	   = GPIOD,
	.rst_pin  	   = GPIO_Pin_0,
	.screen_x_max  = 240,
	.screen_y_max  = 280,
	.is_vertical   = true,
	.is_forward    = false
};

static spi_dev_t spi1;
static const spi_cfg_t spi1_cfg = {
	.spi_periph = SPI1,
	.sck_port   = GPIOA,
	.sck_pin    = GPIO_Pin_5,
	.miso_port  = GPIOA,
	.miso_pin   = GPIO_Pin_6,
	.mosi_port  = GPIOA,
	.mosi_pin   = GPIO_Pin_7,
	.prescaler  = SPI_BaudRatePrescaler_2,
	.mode       = SPI_MODE_3,
};

static int spi1_start(gpio_port_t cs_port, gpio_pin_t cs_pin)
{
	return spi1.ops->start(&spi1, cs_port, cs_pin);
}
static int spi1_swap_byte(uint8_t send, uint8_t *recv)
{
	return spi1.ops->swap_byte(&spi1, send, recv);
}
static int spi1_stop(gpio_port_t cs_port, gpio_pin_t cs_pin)
{
	return spi1.ops->stop(&spi1, cs_port, cs_pin);
}
static st7789v_spi_ops_t st7789v_spi_ops = {
	.start = spi1_start,
	.swap_byte = spi1_swap_byte,
	.stop = spi1_stop	
};

static pwm_dev_t pwm;
static const pwm_cfg_t pwm_cfg = {
	.timer_periph = TIM2,
	.oc_channel   = 2,
	.port		  = GPIOB,
	.pin		  = GPIO_Pin_3
};

static int pwm_set_psc(uint16_t psc)
{
	return pwm.ops->set_psc(&pwm, psc);
}
static int pwm_set_arr(uint16_t arr)
{
	return pwm.ops->set_arr(&pwm, arr);
}
static int pwm_set_compare(uint16_t compare)
{
	return pwm.ops->set_compare(&pwm, compare);
}
static st7789v_pwm_ops_t st7789v_pwm_ops = {
	.set_psc     = pwm_set_psc,
	.set_arr     = pwm_set_arr,
	.set_compare = pwm_set_compare
};

static st7789v_dev_t st7789v;
static const st7789v_cfg_t st7789v_cfg = {
	.spi_ops      = &st7789v_spi_ops,
	.pwm_ops      = &st7789v_pwm_ops,
	.delay_ms     = delay_ms,
	.spi_periph   = SPI1,
	.cs_port      = GPIOC,
	.cs_pin       = GPIO_Pin_9,
	.res_port     = GPIOA,
	.res_pin      = GPIO_Pin_8,
	.dc_port      = GPIOC,
	.dc_pin       = GPIO_Pin_8,
	.x_max        = 240,
	.y_max        = 280,
	.is_vertical  = true,
	.is_forward   = false
};

int main(void)
{
	uint8_t id, fw_ver, finger_num;
	cst816t_data_t data;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	drv_spi_init(&spi1, &spi1_cfg);
	drv_pwm_init(&pwm, &pwm_cfg);
    drv_i2c_soft_init(&i2c_soft, &i2c_soft_cfg);
	drv_cst816t_init(&cst816t, &cst816t_cfg);
    drv_st7789v_init(&st7789v, &st7789v_cfg, NULL);
	
	st7789v.ops->clear(&st7789v, GREEN);
	st7789v.ops->set_backlight(&st7789v, 50);

	cst816t.ops->get_id(&cst816t, &id);
	st7789v.ops->show_str(&st7789v, 20, 20, "ID:", BLACK, GREEN, LCD_12X24, 0);
	st7789v.ops->show_hex_num(&st7789v, 56, 20, id, 2, BLACK, GREEN, LCD_12X24, 0);

	cst816t.ops->get_firmware_ver(&cst816t, &fw_ver);
	st7789v.ops->show_str(&st7789v, 20, 50, "Ver:", BLACK, GREEN, LCD_12X24, 0);
	st7789v.ops->show_hex_num(&st7789v, 68, 50, fw_ver, 2, BLACK, GREEN, LCD_12X24, 0);

	st7789v.ops->show_str(&st7789v, 20, 80, "Ges:", BLACK, GREEN, LCD_12X24, 0);
	st7789v.ops->show_str(&st7789v, 20, 110, "X:", BLACK, GREEN, LCD_12X24, 0);
	st7789v.ops->show_str(&st7789v, 20, 140, "Y:", BLACK, GREEN, LCD_12X24, 0);

	st7789v.ops->show_chinese(&st7789v, 20, 190, "你好", BLACK, GREEN, LCD_32X32, false);
	st7789v.ops->show_chinese(&st7789v, 90, 190, "你好", BLACK, GREEN, LCD_24X24, false);

	while (1) {
		cst816t.ops->get_data(&cst816t, &data);
		cst816t.ops->get_finger_num(&cst816t, &finger_num);

		st7789v.ops->show_hex_num(&st7789v, 68, 80, data.gesture, 2, BLACK, GREEN, LCD_12X24, 0);
		st7789v.ops->show_num(&st7789v, 44, 110, data.x, 3, BLACK, GREEN, LCD_12X24, 0);
		st7789v.ops->show_num(&st7789v, 44, 140, data.y, 3, BLACK, GREEN, LCD_12X24, 0);
		st7789v.ops->show_num(&st7789v, 44, 170, finger_num, 3, BLACK, GREEN, LCD_12X24, 0);
	}
}
