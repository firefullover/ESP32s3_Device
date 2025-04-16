#ifndef __MQTT_H__
#define __MQTT_H__

#include "mqtt_client.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 最大重连间隔（秒） */
#define MQTT_RECONNECT_INTERVAL  10
/* 接收缓冲区大小（字节） */
#define MQTT_RX_BUFFER_SIZE      8192

/**
 * @brief MQTT连接状态回调
 * @param connected 连接状态
 */
typedef void (*mqtt_conn_callback_t)(bool connected);

/**
 * @brief 初始化MQTT客户端
 * @param broker_uri 代理服务器地址（例：mqtt://192.168.1.100:1883）
 * @param client_id  客户端标识符
 * @param rx_callback 数据接收回调（NULL表示使用默认处理）
 */
void mqtt_comm_init(const char *broker_uri, const char *client_id, mqtt_conn_callback_t conn_cb);

/**
 * @brief 发布数据到指定主题
 * @param topic 目标主题
 * @param data  payload数据
 * @param len   数据长度
 * @param qos   服务质量等级（0-2）
 * @return 消息ID（失败返回-1）
 */
int mqtt_publish(const char *topic, const void *data, size_t len, int qos);

/**
 * @brief 订阅指定主题
 * @param topic 订阅主题（支持通配符）
 * @param qos   服务质量等级
 * @return 成功返回ESP_OK
 */
esp_err_t mqtt_subscribe(const char *topic, int qos);

/**
 * @brief 获取发送队列句柄
 */
QueueHandle_t mqtt_get_queue(void);

#ifdef __cplusplus
}
#endif

#endif //__MQTT_H__