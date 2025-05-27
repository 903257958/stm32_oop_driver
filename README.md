# STM32面向对象外设驱动

- 本项目为个人编写的一套STM32外设驱动集合，采用C语言实现“结构体+函数指针”的面向对象编程思想，可大幅度提高代码的模块化与可复用性；
- 当前仓库处于大规模重构阶段，部分驱动尚未完善，完成后将陆续更新。

## 项目特点

- 主要提供STM32F103C8T6与STM32F407VGT6标准库的测试例程；
- 驱动文件高度可移植：无需修改初始化函数内部代码，通过结构体在初始化阶段传入配置信息；
- 驱动兼容多型号MCU：例如F1与F4系列均使用相同代码，也可通过修改驱动文件中的宏定义适配其他型号MCU，甚至迁移至HAL库；
- 移植时只需参考我所提供例程的main.c，自定义外设引脚与通信接口，即可快速适配任意开发板；
- 适配VSCode插件EIDE，如果不使用VSCode开发，"eide"文件夹可直接删除。

## 驱动列表

| 驱动模块                   | 简要说明 |
|---------------------------|----------|
| `xxx_gpio`                | 基本GPIO输入输出操作 |
| `xxx_led_and_delay`       | LED 控制 + 基于SysTick/定时器的延时函数 |
| `xxx_timer_irq`           | 定时器中断 |
| `xxx_key`                 | 按键输入 + 延时消抖 |
| `xxx_key_fifo`            | 按键 + 环形缓冲区 + 定时器中断扫描 + 定时器消抖 |
| `xxx_pwm_servo`           | PWM驱动舵机 |
| `xxx_pwm_rgb`             | PWM驱动RGB灯 |
| `xxx_adc`                 | 多通道ADC采集 |
| `xxx_uart`                | 串口发送 + 空闲中断 + DMA 接收 |
| `xxx_esp8266`             | （待优化）ESP8266通过AT指令获取时间、天气等信息 |
| `xxx_i2c_sw_eeprom`       | 软件I2C读写EEPROM（AT24C02） |
| `xxx_i2c_sw_mpu6050`      | 软件I2C驱动MPU6050获取陀螺仪数据 |
| `xxx_i2c_sw_max30102`     | 软件I2C驱动MAX30102获取心率血氧 |
| `xxx_i2c_sw_ssd1306`      | 软件I2C驱动0.96寸OLED（SSD1306，128×64）|
| `xxx_spi_hw_w25qx`        | 硬件SPI读写外部Flash |
| `xxx_spi_hw_dma_ssd1306`  | 硬件SPI + DMA驱动0.96寸OLED（SSD1306，128×64）|
| `xxx_spi_hw_dma_st7735`   | 硬件SPI + DMA驱动1.8寸LCD（ST7735，128×160）|
| `xxx_fsmc_ili9341`        | FSMC驱动2.8寸LCD（ILI9341，240×320）|
| `xxx_fmc`                 | 内部Flash读写 |
| `xxx_rtc`                 | RTC实时时钟 |
| `xxx_dht11`               | DHT11测量温湿度 |
| `xxx_ds18b20`             | DS18B20测量温度 |
