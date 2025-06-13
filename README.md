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
| `xxx_key`                 | 按键 + 延时消抖 |
| `xxx_key_fifo`            | 按键 + 环形缓冲区 + 定时器中断扫描消抖 |
| `xxx_key_fifo_event`      | xxx_key_fifo 的基础上添加事件处理，支持双击、长按等复杂操作 |
| `xxx_pwm_servo`           | PWM驱动舵机 |
| `xxx_pwm_rgb`             | PWM驱动RGB灯 |
| `xxx_adc`                 | 多通道ADC采集 |
| `xxx_uart`                | 串口发送 + 空闲中断 + DMA 接收 |
| `xxx_esp8266`             | ESP8266通过AT指令获取时间天气信息、TCP透传 |
| `xxx_i2c_soft_aht21`      | 软件I2C驱动AHT21测量温湿度 |
| `xxx_i2c_soft_ap3216c`    | 软件I2C驱动AP3216C获取光照、距离与红外值 |
| `xxx_i2c_soft_bmp280`     | 软件I2C驱动BMP280获取温度与气压值 |
| `xxx_i2c_soft_eeprom`     | 软件I2C读写EEPROM（AT24C02） |
| `xxx_i2c_soft_mpu6050`    | 软件I2C驱动MPU6050获取陀螺仪数据 |
| `xxx_i2c_soft_max30102`   | 软件I2C驱动MAX30102获取心率血氧 |
| `xxx_i2c_soft_ssd1306`    | 软件I2C驱动0.96寸OLED（SSD1306，128×64）|
| `xxx_spi_hard_w25qx`      | 硬件SPI读写外部Flash |
| `xxx_spi_hard_dma_ssd1306`| 硬件SPI + DMA驱动0.96寸OLED（SSD1306，128×64）|
| `xxx_spi_hard_dma_st7735` | 硬件SPI + DMA驱动1.8寸LCD（ST7735，128×160）|
| `xxx_spi_hard_dma_st7789` | 硬件SPI + DMA驱动1.69寸LCD（ST7789，240×280）, 软件I2C驱动CST816T触摸屏|
| `xxx_fsmc_ili9341`        | FSMC驱动2.8寸LCD（ILI9341，240×320）|
| `xxx_flash`               | 读写内部Flash |
| `xxx_rtc`                 | RTC实时时钟 |
| `xxx_dht11`               | DHT11测量温湿度 |
| `xxx_ds18b20`             | DS18B20测量温度 |
| `xxx_stepper_motor`       | 步进电机 |
| `xxx_vibration_motor`     | 振动马达 |
