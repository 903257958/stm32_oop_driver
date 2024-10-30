# STM32外设驱动

- 个人编写的一些STM32外设驱动，使用结构体+函数指针实现C语言的面向对象编程。
- 此仓库会不断更新。

## 适配情况

- 目前适配STM32F103与STM32F407的标准库，驱动文件可不经过修改直接移植。
- 你可以通过修改每个驱动文件开始的宏定义部分的代码，使其适配其他的MCU型号，以及适配HAL库。

## 驱动列表

- **led_reg**: 基于寄存器点灯
- **led**: 面向对象点灯
- **key**: 按键
- **key_fifo**: 按键 + 环形缓冲区
- **usart_dma**: 串口 + 空闲中断 + DMA
- **esp8266**: WiFi模块通过串口实现TCP透传
- **st7735_spi_hw_dma**: ST7735主控1.8寸LCD屏幕，使用硬件SPI + DMA
- **stepper_motor**: 步进电机
