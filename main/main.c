#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mpu6050.h"
#include "wifi.h"
#include "st7789.h"
#include "mqtt.h"

// 配置参数
#define WIFI_SSID      "DragonG"
#define WIFI_PASS      "lrt13729011089"
#define MQTT_URI       "mqtt://192.168.5.109:1883"
#define MQTT_CLIENT_ID "esp32s3_Client"
#define SAMPLING_MS    50
#define GYRO_SCALE     131.0f  // ±250dps灵敏度

static const char *TAG = "Main";
static mpu6050_data_t gyro_bias;

// WiFi连接状态回调
void wifi_callback(bool connected, const char *ip) {
    if(connected) {
        ESP_LOGI(TAG, "已连接WiFi，IP地址: %s", ip);
    } else {
        ESP_LOGI(TAG, "WiFi连接丢失");
    }
}

// MQTT图像接收处理
static void display_image_callback(const uint8_t *data, size_t len) {
    st7789_display_raw(data, len);
    ESP_LOGI(TAG, "已刷新显示屏图像");
}

void app_main(void)
{
    // 初始化各模块
    ESP_LOGI(TAG, "系统启动...");
    
    // WiFi初始化
    wifi_init(WIFI_SSID, WIFI_PASS, wifi_callback);
    vTaskDelay(pdMS_TO_TICKS(5000));  // 等待网络连接

    // 显示屏初始化
    st7789_init();
    st7789_fill_screen(0x0000);  // 初始黑屏
    ESP_LOGI(TAG, "显示屏初始化完成");

    // 传感器初始化
    mpu6050_init();
    mpu6050_calibrate_gyro(&gyro_bias, 100);  // 校准陀螺仪偏移

    // MQTT初始化
    mqtt_comm_init(MQTT_URI, MQTT_CLIENT_ID, display_image_callback);
    ESP_LOGI(TAG, "MQTT客户端已启动");

    // 主循环变量
    float angles[3] = {0};  // X/Y/Z轴角度
    int64_t last_time = esp_timer_get_time();

    while (1) {
        // 读取传感器数据
        mpu6050_data_t raw;
        mpu6050_read_gyro(&raw);
        
        // 计算角度变化
        float dt = (esp_timer_get_time() - last_time) / 1e6;
        angles[0] += (raw.x - gyro_bias.x) / GYRO_SCALE * dt;
        angles[1] += (raw.y - gyro_bias.y) / GYRO_SCALE * dt;
        angles[2] += (raw.z - gyro_bias.z) / GYRO_SCALE * dt;
        last_time = esp_timer_get_time();

        // 发送MQTT数据
        char payload[64];
        snprintf(payload, sizeof(payload), 
                "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}", 
                angles[0], angles[1], angles[2]);
        mqtt_publish("6050_data", payload, strlen(payload), 1);

        vTaskDelay(pdMS_TO_TICKS(SAMPLING_MS));
    }
}