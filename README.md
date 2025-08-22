# STM32面向对象外设驱动

- 本项目为个人编写的一套STM32外设驱动集合，采用C语言实现“结构体+函数指针”的面向对象编程思想，可大幅度提高代码的模块化与可复用性。

## 项目特点

- 主要提供STM32F103C8T6与STM32F407VGT6标准库的测试例程；
- 驱动文件高度可移植：无需修改初始化函数内部代码，通过结构体在初始化阶段传入配置信息；
- 驱动兼容多型号MCU：例如F1与F4系列均使用相同代码，也可通过修改驱动文件中的宏定义适配其他型号MCU，甚至迁移至HAL库；
- 移植时只需参考我所提供例程的main.c，自定义外设引脚与通信接口，即可快速适配任意开发板；
- 适配VSCode插件EIDE，如果不使用VSCode开发，"eide"文件夹可直接删除。

## 驱动列表

| 驱动模块                       | 简要说明 |
|-------------------------------|----------|
| `xxx_gpio`                    | GPIO基础输入输出控制 |
| `xxx_led_and_delay`           | LED驱动与SysTick/定时器延时函数 |
| `xxx_exti`                    | 外部中断 |
| `xxx_exti_encoder`            | 外部中断控制旋转编码器 |
| `xxx_timer_irq`               | 定时器中断 |
| `xxx_timer_oc_pwm_servo`      | 定时器输出比较PWM驱动舵机 |
| `xxx_timer_oc_pwm_tb6612`     | 定时器输出比较PWM驱动TB6612（直流电机驱动） |
| `xxx_timer_oc_pwm_rgb`        | 定时器输出比较PWM驱动RGB灯 |
| `xxx_timer_oc_pwm_dma_ws2812b`| 定时器输出比较PWM+DMA驱动WS2812B灯带 |
| `xxx_timer_ic_sr04`           | 定时器输入捕获驱动SR04超声波测距模块 |
| `xxx_key`                     | 按键扫描+延时消抖 |
| `xxx_key_fifo`                | 按键环形缓冲区+定时器中断消抖 |
| `xxx_key_fifo_event`          | 支持双击/长按的按键事件处理（基于xxx_key_fifo） |
| `xxx_adc`                     | ADC采集 |
| `xxx_uart`                    | 串口发送+空闲中断+DMA接收 |
| `xxx_esp8266`                 | ESP8266 AT指令（时间天气获取/TCP透传） |
| `xxx_i2c_soft_aht21`          | 软件I2C驱动AHT21（温湿度） |
| `xxx_i2c_soft_ap3216c`        | 软件I2C驱动AP3216C（光照/距离/红外） |
| `xxx_i2c_soft_bmp280`         | 软件I2C驱动BMP280（温度/气压） |
| `xxx_i2c_soft_eeprom`         | 软件I2C读写EEPROM（AT24C02） |
| `xxx_i2c_soft_mpu6050`        | 软件I2C驱动MPU6050（陀螺仪） |
| `xxx_i2c_soft_max30102`       | 软件I2C驱动MAX30102（心率/血氧） |
| `xxx_i2c_soft_ssd1306`        | 软件I2C驱动0.96寸OLED（SSD1306, 128×64） |
| `xxx_spi_hard_dma_ssd1306`    | 硬件SPI+DMA驱动0.96寸OLED（SSD1306, 128×64） |
| `xxx_spi_hard_dma_st7735`     | 硬件SPI+DMA驱动1.8寸LCD（ST7735, 128×160） |
| `xxx_spi_hard_dma_st7789`     | 硬件SPI+DMA驱动1.69寸LCD（ST7789, 240×280）<br>（+软件I2C驱动CST816T触摸屏） |
| `xxx_spi_hard_w25qx`          | 硬件SPI读写外部Flash（W25QX） |
| `xxx_spi_soft_w25qx`          | 软件SPI读写外部Flash（W25QX） |
| `xxx_fsmc_ili9341`            | FSMC驱动2.8寸LCD（ILI9341, 240×320） |
| `xxx_can`                     | CAN通信 |
| `xxx_flash`                   | 内部Flash读写 |
| `xxx_rtc`                     | RTC实时时钟 |
| `xxx_dht11`                   | DHT11驱动（温湿度） |
| `xxx_ds18b20`                 | DS18B20驱动（温度） |
| `xxx_stepper_motor`           | 步进电机驱动 |
| `xxx_vibration_motor`         | 振动马达驱动 |
