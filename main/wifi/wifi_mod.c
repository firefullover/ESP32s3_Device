#include "wifi_mod.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "WiFi_STA";

// Wi-Fi 事件处理函数
static void wifi_event_handler(void *arg, esp_event_base_t event_base,int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {  // 如果是 Wi-Fi 事件
        if (event_id == WIFI_EVENT_STA_START) {   // Wi-Fi 站点启动事件
            ESP_LOGI(TAG, "STA 启动");
            esp_wifi_connect();   // 连接 Wi-Fi
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {   // Wi-Fi 断开连接事件
            ESP_LOGI(TAG, "已断开 Wi-Fi 连接，正在重新连接...");
            esp_wifi_connect();   // 尝试重新连接 Wi-Fi
        }
    } else if (event_base == IP_EVENT) {   // 如果是 IP 事件
        if (event_id == IP_EVENT_STA_GOT_IP) {   // 获取到 IP 地址事件
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;  // 获取 IP 信息
            ESP_LOGI(TAG, "Wifi连接，已获取到 IP 地址: " IPSTR, IP2STR(&event->ip_info.ip));  // 打印 IP 地址
        }
    }
}

// STA 模式初始化
void wifi_init(const char *ssid, const char *password) {
    // 参数检查
    if(!ssid || !password || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "无效的SSID或密码");
        return ESP_ERR_INVALID_ARG;
    }

    // 保存配置
    strncpy(configured_ssid, ssid, sizeof(configured_ssid)-1);
    strncpy(configured_pass, password, sizeof(configured_pass)-1);

    // 初始化 NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 仅创建 STA 的网络接口
    esp_netif_create_default_wifi_sta();

    // wifi 配置
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 注册事件处理器
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));
    
    // 配置 STA 参数
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ssid,
            .password = password,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        }
    };
    // 设置模式为仅 STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WIFI_STA 模式初始化完成");
}