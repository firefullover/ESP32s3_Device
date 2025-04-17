#ifndef __WIFI_H__
#define __WIFI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_wifi.h"
#include "esp_event.h"

#define WIFI_SSID_STA      "DragonG"
#define WIFI_PASS_STA      "lrt13729011089"

/**
 * @brief WiFi事件处理函数
 */
void wifi_event_handler(void *arg, esp_event_base_t event_base,int32_t event_id, void *event_data);

/**
 * @brief  初始化WiFi连接模块
 * @param  ssid      WiFi网络SSID
 * @param  password  WiFi密码
 */
void wifi_init(const char *ssid, const char *password);


/**
 * @brief 初始化并启动 WiFi STA 模式
 */
void wifi_init_sta(void);

#ifdef __cplusplus
}
#endif

#endif