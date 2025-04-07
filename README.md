# STM32面向对象外设驱动

- 个人编写的一些STM32外设驱动，使用结构体+函数指针实现C语言的面向对象编程；
- 此仓库会不断更新。

## 注意事项

- 测试例程使用STM32F407VGT6；
- 目前主要适配STM32F407的标准库，部分适配STM32F103、STM32F405、STM32F411标准库，驱动文件可不经过修改直接移植；
- 你可以通过修改每个驱动文件的宏定义部分的代码，使其适配其他的MCU型号，以及适配HAL库；
- 在移植驱动到不同型号的芯片后，参考我的main.h与main.c的代码，进行外设的定义与初始化，你可以自己定义外设的引脚与通信接口，使其适配你的开发板；
- 适配VSCode插件EIDE，如果不使用VSCode开发，"eide"文件夹可直接删除。

## 驱动列表

- **gpio**: 基本GPIO读写操作
- **led_and_delay**: 点亮LED与delay函数(基于systick/定时器)
- **key**: 按键 + 延时消抖
- **key_fifo_timer**: 按键 + 环形缓冲区 + 定时器中断扫描 + 定时器消抖
- **timer_irq**: 定时器中断
- **adc**: ADC多通道采集
- **usart_dma**: 串口 + 空闲中断 + DMA
- **esp_at**: ESP32或ES8266通过AT命令获取天气与时间
- **w25qx_spi_hw**: 硬件SPI读写外部Flash
- **w25qx_spi_sw**: 软件SPI读写外部Flash
- **ssd1306_spi_hw_dma**: 硬件SPI + DMA驱动 SSD1306 主控 分辨率128*64 0.96寸OLED屏幕
- **st7735_spi_hw_dma**: 硬件SPI + DMA驱动 ST7735 主控 分辨率128*160 1.8寸LCD屏幕
- **st7789_spi_hw_dma**: 硬件SPI + DMA驱动 ST7789 主控 分辨率240*280 1.69寸LCD屏幕
- **st7789_spi_hw_dma_touch**: 硬件SPI + DMA驱动 ST7789 主控 分辨率240*280 1.69寸LCD屏幕 + CST816T 电容触摸
- **ili9341_fsmc**: FSMC驱动 ILI9341 主控 分辨率240*320 2.8寸LCD屏幕
- **stepper_motor**: 步进电机
- **passive_buzzer_pwm**: PWM控制无源蜂鸣器
- **rtc**: RTC实时时钟
- **mpu6050_i2c_sw**: 软件I2C + MPU6050
- **mpu6050_i2c_sw_irq**: 软件I2C + MPU6050 + 中断引脚
- **aht21_i2c_sw**: 软件I2C + AHT21 获取温湿度
- **max30102_i2c_sw**: 软件I2C + MAX30102 获取心率血氧
- **bmp280_i2c_sw**: 软件I2C + BMP280 获取气压与海拔




