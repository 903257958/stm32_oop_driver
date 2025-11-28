#include <string.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <errno.h>
#include "drv_esp8266.h"

/* --------------------------------- 核心驱动层 --------------------------------- */

static void esp8266_log_raw(esp8266_dev_t *dev, const char *fmt, ...);
static void esp8266_delay_ms(esp8266_dev_t *dev, uint32_t ms);

#define esp_log(...)  esp8266_log_raw(dev, __VA_ARGS__)
#define esp_delay_ms(ms)  esp8266_delay_ms(dev, ms)

static void esp8266_clear(esp8266_dev_t *dev);

static int esp8266_send_cmd_impl(esp8266_dev_t *dev,
								const char *cmd,
								const char *expect,
								char **out,
                                uint32_t timeout_ms);
static int esp8266_wifi_connect(esp8266_dev_t *dev, const char *ssid, const char *pwd);
static int esp8266_wifi_disconnect(esp8266_dev_t *dev);
static int esp8266_tcp_transmit_connect_impl(esp8266_dev_t *dev, const char *ip, uint16_t port);
static int esp8266_tcp_transmit_disconnect_impl(esp8266_dev_t *dev);
static int esp8266_tcp_send_data_impl(esp8266_dev_t *dev, uint8_t *data, uint32_t len);
static int esp8266_tcp_recv_data_impl(esp8266_dev_t *dev, uint8_t **data, uint32_t *len, uint32_t timeout_ms);
static int esp8266_deinit_impl(esp8266_dev_t *dev);

/* 操作接口表 */
static const esp8266_ops_t esp8266_ops = {
	.send_cmd                = esp8266_send_cmd_impl,
    .wifi_connect            = esp8266_wifi_connect,
    .wifi_disconnect         = esp8266_wifi_disconnect,
	.tcp_transmit_connect    = esp8266_tcp_transmit_connect_impl,
	.tcp_transmit_disconnect = esp8266_tcp_transmit_disconnect_impl,
	.tcp_send_data           = esp8266_tcp_send_data_impl,
	.tcp_recv_data           = esp8266_tcp_recv_data_impl,
	.deinit                  = esp8266_deinit_impl
};

/**
 * @brief   初始化 ESP8266 驱动
 * @param[out] dev esp8266_dev_t 结构体指针
 * @param[in]  cfg esp8266_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_esp8266_init(esp8266_dev_t *dev, const esp8266_cfg_t *cfg)
{
	char *recv = NULL;

	if (!dev || !cfg)
        return -EINVAL;

	dev->cfg = *cfg;
	dev->ops = &esp8266_ops;

    esp_log("\r\n==================== ESP8266 INIT ====================\r\n");
	esp_delay_ms(500);  // 等待 ESP8266 启动

    /* 1. 关闭回显 */
    if (dev->ops->send_cmd(dev, "ATE0", "OK", &recv, 3000) != 0)
        return -EIO;

	/* 2. AT 测试 */
	if (dev->ops->send_cmd(dev, "AT", "OK", &recv, 3000) != 0)
        return -EIO;

	esp_log("================== ESP8266 INIT DONE =================\r\n");
	return 0;
}

/**
 * @brief	原始日志打印函数
 * @details 使用可变参数格式化字符串，并通过 esp8266_log_ops 的 printf 输出，
 *          不会清空缓冲区，也不处理任何日志前缀或级别。
 * @param[in] dev ESP8266 设备结构体指针
 * @param[in] fmt 格式化字符串（与 printf 相同）
 * @param[in] ... 可变参数
 */
static void esp8266_log_raw(esp8266_dev_t *dev, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    if (dev && dev->cfg.log_ops && dev->cfg.log_ops->vprintf)
        dev->cfg.log_ops->vprintf(fmt, args);
    va_end(args);
}

/**
 * @brief	ESP8266 毫秒级延时
 * @param[in] dev ESP8266 设备结构体指针
 * @param[in] ms  毫秒
 */
static void esp8266_delay_ms(esp8266_dev_t *dev, uint32_t ms)
{
    dev->cfg.delay_ms(ms);
}

/**
 * @brief   清空 ESP8266 串口接收缓冲区
 * @param[in] dev esp8266_dev_t 结构体指针
 */
static void esp8266_clear(esp8266_dev_t *dev)
{
    uint8_t *dummy_data = NULL;
    uint32_t dummy_len = 0;
    int no_data_count = 0;
	int ret;
    
	if (dev->cfg.rx_buf && dev->cfg.rx_buf_size > 0)
        memset(dev->cfg.rx_buf, 0, dev->cfg.rx_buf_size);

    /* 持续清空直到连续5次接收不到数据 */
    while (no_data_count < 5) {
        ret = dev->cfg.uart_ops->recv_data(&dummy_data, &dummy_len);
        if (ret == 0 && dummy_data && dummy_len > 0)
            no_data_count = 0;
        else
            no_data_count++;
        esp_delay_ms(1);
    }
}

/**
 * @brief	ESP8266 发送 AT 命令并等待期望响应
 * @details 发送流程：
 *  		1. 清空接收缓冲区
 *  		2. 构造发送内容：cmd + "\r\n"
 *  		3. 调用 UART 发送
 *  		4. 在超时时间内持续接收数据，并实时写入 rx_buf
 *  		5. 若 rx_buf 中出现 expect，则立即返回成功
 *  		6. 若超时或无新数据超过阈值，则重发命令
 * 			7. 连续 3 次重试均超时则返回失败
 * 
 * @param[in]  dev        esp8266_dev_t 结构体指针
 * @param[in]  cmd        要发送的 AT 命令（不包含 "\r\n"）
 * @param[in]  expect     期望的关键字，例如 "OK"、"ERROR"、">"
 * @param[out] out        若不为 NULL，当检测到 expect 时返回 rx_buf（整段接收内容）
 * @param[out] timeout_ms 超时时间（毫秒）
 * @return	0 表示成功，其他值表示失败
 */
static int esp8266_send_cmd_impl(struct esp8266_dev *dev,
                                 const char *cmd,
                                 const char *expect,
                                 char **out,
                                 uint32_t timeout_ms)
{
    uint8_t *rx_buf = dev->cfg.rx_buf;
    uint8_t *tx_buf = dev->cfg.tx_buf;
    uint16_t rx_size = dev->cfg.rx_buf_size;
    uint16_t tx_size = dev->cfg.tx_buf_size;

    uint8_t *recv_ptr = NULL;
    uint32_t recv_len = 0;
    uint32_t rx_pos;
    uint32_t no_new_data;

    uint8_t retry_cnt = 3;
    bool busy_flag = false;   /* 检测到 busy p... */

    while (retry_cnt--) {

        rx_pos = 0;
        no_new_data = 0;

        /*--------------------------------------------------------------------
         * 清空接收缓冲区
         *------------------------------------------------------------------*/
        esp8266_clear(dev);

        /*--------------------------------------------------------------------
         * 构造发送内容
         *------------------------------------------------------------------*/
        uint16_t cmd_len = strlen(cmd);
        if (cmd_len + 2 >= tx_size) {
            esp_log("!!![ESP8266] TX buf too small for command: %s\r\n", cmd);
            return -ENOMEM;
        }

        memcpy(tx_buf, cmd, cmd_len);
        tx_buf[cmd_len] = '\r';
        tx_buf[cmd_len + 1] = '\n';
        tx_buf[cmd_len + 2] = '\0';

        esp_log("\r\n>>> [ESP8266 SEND] %s", (char *)tx_buf);
        dev->cfg.uart_ops->send_data(tx_buf, cmd_len + 2);
        esp_delay_ms(100);

        /*--------------------------------------------------------------------
         * 接收循环（零拷贝 + 复制到本地 rx_buf 进行累积、搜索）
         *------------------------------------------------------------------*/
        while (timeout_ms--) {

            int ret = dev->cfg.uart_ops->recv_data(&recv_ptr, &recv_len);
            if (ret == 0 && recv_len > 0 && recv_ptr) {

                /* 防溢出保护 */
                if (rx_pos + recv_len >= rx_size) {
                    recv_len = rx_size - rx_pos - 1;
                    if (!recv_len)
                        break;
                }

                /* 拷贝入 ESP 的专属缓冲区，方便搜索 expect */
                memcpy(rx_buf + rx_pos, recv_ptr, recv_len);
                rx_pos += recv_len;
                rx_buf[rx_pos] = '\0';

                esp_log("<<< [ESP8266 RECV] %.*s", (int)recv_len, recv_ptr);

                /*------------ 状态检测 ----------------*/

                if (!busy_flag && strstr((char *)rx_buf, "busy p")) {
                    esp_log("!!! [ESP8266 BUSY] extending timeout +3000ms\r\n");
                    timeout_ms += 3000;
                    busy_flag = true;
                    no_new_data = 0;
                }

                if (strstr((char *)rx_buf, "FAIL") ||
                    strstr((char *)rx_buf, "ERROR")) {
                    esp_log("!!! [ESP8266 ERROR] command failed\r\n");
                    break;
                }

                if (strstr((char *)rx_buf, expect)) {
                    esp_log("=== [ESP8266 SUCCESS] Command matched: %s ===\r\nFull response:\r\n%s\r\n",
                            cmd, rx_buf);
                    if (out)
                        *out = (char *)rx_buf;
                    return 0;
                }

                no_new_data = 0;
            } else {
                /* 没有新数据 */
                no_new_data++;
                esp_delay_ms(1);
            }
        }

        /*--------------------------------------------------------------------
         * 本轮失败
         *------------------------------------------------------------------*/
        esp_log("!!! [ESP8266 TIMEOUT] \"%s\" (retry %d/3)", cmd, 3 - retry_cnt);
        if (rx_pos)
            esp_log("Partial response: %s\r\n", rx_buf);
        else
            esp_log("No data received.\r\n");

        esp_delay_ms(100);
    }

    esp_log("!!! [ESP8266 FAILED] \"%s\" after 3 retries", cmd);
    esp_log("Final response: %s\r\n", (rx_pos > 0) ? rx_buf : "<ESP8266 RX EMPTY>");

    return -ETIMEDOUT;
}

/**
 * @brief	ESP8266 连接 WiFi
 * @param[in] dev  esp8266_dev_t 结构体指针
 * @param[in] ssid WiFi 名称
 * @param[in] pwd  WiFi 密码
 * @return	0 表示成功，其他值表示失败
 */
static int esp8266_wifi_connect(esp8266_dev_t *dev, const char *ssid, const char *pwd)
{
    char *recv = NULL;
    char cmd[64];

    if (!dev || !ssid || !pwd)
        return -EINVAL;

    esp_log("\r\n================ ESP8266 WIFI CONNECT ================\r\n");

    /* 1. 设置客户端模式（仅当前有效，不写入 flash） */
	if (dev->ops->send_cmd(dev, "AT+CWMODE_CUR=1", "OK", &recv, 3000) != 0)
        return -EIO;

    /* 2. 断开现有WiFi连接 */
    if (dev->ops->send_cmd(dev, "AT+CWQAP", "OK", &recv, 3000) != 0)
        return -EIO;

	/* 3. 连接 WiFi */
	snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, pwd);
	if (dev->ops->send_cmd(dev, cmd, "WIFI GOT IP", &recv, 10000) != 0)
        return -EIO;
	esp_delay_ms(1000);    // 等待WiFi连接稳定

	/* 4. 再次 AT 确认模块状态正常 */
	if (dev->ops->send_cmd(dev, "AT", "OK", &recv, 3000) != 0)
        return -EIO;
    
    esp_log("============= ESP8266 WIFI CONNECT DONE ==============\r\n");
    return 0;
}

/**
 * @brief	ESP8266 断开 WiFi 连接
 * @param[in] dev esp8266_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int esp8266_wifi_disconnect(esp8266_dev_t *dev)
{
    char *recv = NULL;

    if (!dev)
        return -EINVAL;

    esp_log("\r\n============== ESP8266 WIFI DISCONNECT ===============\r\n");

    /* 1. 设置客户端模式（仅当前有效，不写入 flash） */
	if (dev->ops->send_cmd(dev, "AT+CWMODE_CUR=1", "OK", &recv, 3000) != 0)
        return -EIO;

    /* 2. 断开现有WiFi连接 */
    if (dev->ops->send_cmd(dev, "AT+CWQAP", "OK", &recv, 3000) != 0)
        return -EIO;

	/* 3. 再次 AT 确认模块状态正常 */
	if (dev->ops->send_cmd(dev, "AT", "OK", &recv, 3000) != 0)
        return -EIO;
    
    esp_log("============ ESP8266 WIFI DISCONNECT DONE ============\r\n");
    return 0;
}

/**
 * @brief	ESP8266 建立 TCP 连接 + 透传（作为客户端）
 * @param[in]  dev  esp8266_dev_t 结构体指针
 * @param[in]  ip   服务器IP地址字符串（如 "192.168.4.1"）
 * @param[in]  port 服务器端口（如 8080）
 * @return	0 表示成功，其他值表示失败
 */
static int esp8266_tcp_transmit_connect_impl(esp8266_dev_t *dev, const char *ip, uint16_t port)
{
	char cmd[64];
    char *recv = NULL;

    if (!dev || !ip)
        return -EINVAL;

    esp_log("\r\n================ ESP8266 TCP CONNECT =================\r\n");

    /* 设置为单连接模式 */
    if (dev->ops->send_cmd(dev, "AT+CIPMUX=0", "OK", &recv, 3000) != 0)
        return -EIO;

    /* 连接服务器 */
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d", ip, port);
    if (dev->ops->send_cmd(dev, cmd, "CONNECT", &recv, 3000) != 0)
        return -EIO;

    /* 进入透传模式 */
    if (dev->ops->send_cmd(dev, "AT+CIPMODE=1", "OK", &recv, 3000) != 0)
        return -EIO;

    if (dev->ops->send_cmd(dev, "AT+CIPSEND", ">", &recv, 3000) != 0)
        return -EIO;

    esp_log(">>> TCP connection established: %s:%u\r\n", ip, port);
    esp_log("============== ESP8266 TCP CONNECT DONE ==============\r\n");
    return 0;
}

/**
 * @brief	ESP8266 退出 TCP 透传模式
 * @param[in] dev esp8266_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int esp8266_tcp_transmit_disconnect_impl(esp8266_dev_t *dev)
{
    char *recv = NULL;

    if (!dev)
		return -EINVAL;

    esp_log("\r\n=============== ESP8266 TCP DISCONNECT ===============\r\n");

    /* 退出透传 */
    if (dev->cfg.uart_ops->send_data("+++", 3) != 0)
        return -EIO;
    esp_delay_ms(100);

    /* 断开TCP连接 */
    if (dev->ops->send_cmd(dev, "AT+CIPCLOSE", "OK", &recv, 3000) != 0)
        return -EIO;

    /* 再次 AT 确认模块状态正常 */
	if (dev->ops->send_cmd(dev, "AT", "OK", &recv, 3000) != 0)
        return -EIO;

    esp_log("============ ESP8266 TCP DISCONNECT DONE =============\r\n");
    return 0;
}

/**
 * @brief	ESP8266 TCP 透传发送数据
 * @param[in] dev  esp8266_dev_t 结构体指针
 * @param[in] data 发送的数据
 * @return	0 表示成功，其他值表示失败
 */
static int esp8266_tcp_send_data_impl(esp8266_dev_t *dev, uint8_t *data, uint32_t len)
{
	if (!dev || !data)
        return -EINVAL;

    if (len > dev->cfg.tx_buf_size) {
        esp_log("!!! TX buffer too small for TCP send (%u > %u)\n", len, dev->cfg.tx_buf_size);
        return -ENOMEM;
    }

    memcpy(dev->cfg.tx_buf, data, len);
    dev->cfg.uart_ops->send_data(dev->cfg.tx_buf, len);

    /* 打印日志，截断长数据避免刷屏 */
    if (len > 128)
        esp_log("\r\n>>> [ESP8266 TCP SEND] %.*s... (len=%u)\r\n", 128, data, len);
    else
        esp_log("\r\n>>> [ESP8266 TCP SEND] %.*s\r\n", (int)len, data);

    return 0;
}

/**
 * @brief	ESP8266 TCP 透传接收数据（零拷贝）
 * @param[in]  dev        esp8266_dev_t 结构体指针
 * @param[out] data       输出参数，接收数据首地址（无需调用方分配缓冲区，只需传入指针的地址）
 * @param[out] len        输出参数，接收数据长度
 * @param[in]  timeout_ms 超时时间（毫秒）
 * @return	0 表示成功，其他值表示失败
 */
static int esp8266_tcp_recv_data_impl(esp8266_dev_t *dev, uint8_t **data, uint32_t *len, uint32_t timeout_ms)
{
    if (!dev || !data || !len)
        return -EINVAL;

    uint32_t elapsed = 0;
    *data = NULL;
    *len = 0;

    while (elapsed < timeout_ms) {
        uint8_t *ptr = NULL;
        uint32_t rlen = 0;

        if (dev->cfg.uart_ops->recv_data(&ptr, &rlen) == 0 && rlen > 0) {
            *data = ptr;
            *len = rlen;
            return 0;
        }
        esp_delay_ms(1);
        elapsed++;
    }

    return -ETIMEDOUT;
}

/**
 * @brief	去初始化 ESP8266
 * @param[in] dev esp8266_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int esp8266_deinit_impl(esp8266_dev_t *dev)
{    
    if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}
