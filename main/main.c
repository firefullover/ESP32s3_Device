#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "config.h"
#include "mpu6050.h"
#include "wifi_mod.h"
#include "st7789.h"
#include "mqtt.h"

static const char *TAG = "APP_Main";
static mpu6050_data_t gyro_bias;

// MQTT连接状态回调
static void mqtt_connection_callback(bool connected) {
    if(connected) {
        ESP_LOGI(TAG, "MQTT已连接到服务器");
    } else {
        ESP_LOGI(TAG, "MQTT连接断开");
    }
}

// // MQTT图像接收处理
// static void display_image_callback(const uint8_t *data, size_t len) {
//     st7789_display_raw(data, len);
//     ESP_LOGI(TAG, "已刷新显示屏图像");
// }

void app_main(void) {
    // WiFi初始化
    wifi_init();
    ESP_LOGI(TAG, "等待网络连接...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    // MQTT初始化
    mqtt_comm_init(mqtt_connection_callback);
    ESP_LOGI(TAG, "等待MQTT连接...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    // 显示屏初始化
    st7789_init();
    st7789_fill_screen(0xFFFF);  // 初始化时填充白色

    // 传感器初始化
    mpu6050_init();
    mpu6050_calibrate_gyro(&gyro_bias, 100);  // 校准陀螺仪偏移

    float angles[2] = {90.0f,90.0f};  // Y/Z轴角度
    int64_t last_time = esp_timer_get_time();

    while (1) {
        // 读取传感器数据
        mpu6050_data_t raw;
        mpu6050_read_gyro(&raw);
        
        // 计算角度变化
        float dt = (esp_timer_get_time() - last_time) / 1e6;
        angles[0] = mpu6050_calculate_angle(angles[0], (raw.y - gyro_bias.y) / GYRO_SCALE , dt);
        angles[1] = mpu6050_calculate_angle(angles[1], (raw.z - gyro_bias.z) / GYRO_SCALE , dt);
        last_time = esp_timer_get_time();

        printf("Y: %.2f, Z: %.2f\n", angles[0], angles[1]);

        // 发送MQTT数据
        char payload[64];
        snprintf(payload, sizeof(payload), 
                "{\"y\":%.2f,\"z\":%.2f}", 
                angles[0], angles[1]);
        mqtt_publish(MQTT_TOPIC, payload, strlen(payload), 1);

        vTaskDelay(pdMS_TO_TICKS(100));// 延时100ms（0.1s）
    }
}