#ifndef DRV_ESP8266_H
#define DRV_ESP8266_H

#include <stdint.h>
#include <stdbool.h>

#ifndef EIO
#define EIO 11
#endif
#ifndef ETIMEDOUT
#define ETIMEDOUT 110 
#endif

/* 串口操作接口结构体 */
typedef struct {
	/**
	 * @brief   串口发送原始二进制数据
	 * @param[in] data 待发送数据起始地址
	 * @param[in] len  数据长度（字节）
	 * @return	0 表示成功，其他值表示失败
	 */
	int (*send_data)(uint8_t *data, uint32_t len);

	/**
	 * @brief   串口接收原始数据（零拷贝，不添加 '\0'）
	 * @details 该函数不进行 memcpy，也不在数据后追加 '\0'，完全适用于二进制协议（如 Xmodem、IAP、文件传输）
	 *          返回的 data 指针指向环形缓冲区的原始内存区域。
	 * 			注意：该指针在下一次接收后内容可能被覆盖，调用方应当在使用前及时处理或复制。
	 * @param[out] data 输出参数，返回数据首地址（无需调用方分配缓冲区，只需传入指针的地址）
	 * @param[out] len  输出参数，返回数据长度
	 * @return	0 表示成功，其他值表示失败
	 */
	int (*recv_data)(uint8_t **data, uint32_t *len);
} esp8266_uart_ops_t;

/* 日志操作接口结构体 */
typedef struct {
	void (*vprintf)(const char *format, va_list args);
} esp8266_log_ops_t;

/* 配置结构体 */
typedef struct {
	const esp8266_uart_ops_t *uart_ops;
	const esp8266_log_ops_t *log_ops;
	void (*delay_ms)(uint32_t ms);
	uint8_t    *rx_buf;
    uint16_t    rx_buf_size;
    uint8_t    *tx_buf;
    uint16_t    tx_buf_size;
} esp8266_cfg_t;

typedef struct esp8266_dev esp8266_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*send_cmd)(esp8266_dev_t *dev,
					const char *cmd,
					const char *expect,
					char **out,
					uint32_t timeout_ms);
	int (*wifi_connect)(esp8266_dev_t *dev, const char *ssid, const char *pwd);
	int (*wifi_disconnect)(esp8266_dev_t *dev);
	int (*tcp_transmit_connect)(esp8266_dev_t *dev, const char *ip, uint16_t port);
	int (*tcp_transmit_disconnect)(esp8266_dev_t *dev);
    int (*tcp_send_data)(esp8266_dev_t *dev, uint8_t *data, uint32_t len);
    int (*tcp_recv_data)(esp8266_dev_t *dev, uint8_t **data, uint32_t *len, uint32_t timeout_ms);
	int (*deinit)(esp8266_dev_t *dev);
} esp8266_ops_t;

/* 设备结构体 */
struct esp8266_dev {
	esp8266_cfg_t cfg;
	const esp8266_ops_t *ops;
};

/**
 * @brief   初始化 ESP8266 驱动
 * @param[out] dev esp8266_dev_t 结构体指针
 * @param[in]  cfg esp8266_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_esp8266_init(esp8266_dev_t *dev, const esp8266_cfg_t *cfg);

#endif
