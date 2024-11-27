# STM32外设驱动

- 个人编写的一些STM32外设驱动，使用结构体+函数指针实现C语言的面向对象编程；
- 此仓库会不断更新。

## 适配情况

- 这些是STM32F407VGT6工程；
- 目前适配STM32F103与STM32F407的标准库，驱动文件可不经过修改直接移植；
- 你可以通过修改每个驱动文件的宏定义部分的代码，使其适配其他的MCU型号，以及适配HAL库；
- 在移植驱动到不同型号的芯片后，参考我的main.h与main.c的代码，进行外设的定义与初始化，你可以自己定义外设的引脚与通信接口，使其适配你的开发板。

## 驱动列表

- **led_reg**: 寄存器点灯
- **led**: 点灯
- **key**: 按键 + 延时消抖
- **key_fifo_timer**: 按键 + 环形缓冲区 + 定时器中断扫描 + 定时器消抖
- **timer_irq**: 定时器中断
- **usart_dma**: 串口 + 空闲中断 + DMA
- **esp8266**: WiFi模块通过串口实现TCP透传
- **w25qx_spi_hw**: 硬件SPI读写外部Flash
- **w25qx_spi_sw**: 软件SPI读写外部Flash
- **ssd1306_spi_hw_dma**: 硬件SPI + DMA驱动 SSD1306主控0.96寸OLED屏幕
- **st7735_spi_hw_dma**: 硬件SPI + DMA驱动 ST7735主控1.8寸LCD屏幕
- **ili9341_fsmc**: FSMC驱动 ILI9341主控2.8寸LCD屏幕
- **ili9341_fsmc_touch_flash**: FSMC驱动 ILI9341主控2.8寸LCD屏幕 + XPT2046触摸屏，校准数据保存在Flash中
- **stepper_motor**: 步进电机
- **passive_buzzer_pwm**: PWM控制无源蜂鸣器
