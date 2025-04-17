#include "mqtt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static esp_mqtt_client_handle_t client = NULL;
static QueueHandle_t send_queue = NULL;
static mqtt_conn_callback_t conn_callback = NULL;

static const char *TAG = "MQTT";

/* 内部函数声明 */
static void event_handler(void *args, esp_event_base_t base, 
                        int32_t event_id, void *event_data);
static void queue_manager_task(void *arg);

// 默认图像处理参数
#define IMG_TOPIC        "6818_image"
#define IMG_WIDTH        240
#define IMG_HEIGHT       240
#define IMG_PIXEL_SIZE   2
static uint8_t img_buffer[IMG_WIDTH * IMG_HEIGHT * IMG_PIXEL_SIZE];
static size_t img_received = 0;

void mqtt_comm_init(const char *broker_uri, const char *client_id, mqtt_conn_callback_t conn_cb) {
    // 参数校验
    if (!broker_uri || !client_id) {
        ESP_LOGE(TAG, "Invalid parameters");
        return;
    }

    // 创建发送队列
    send_queue = xQueueCreate(5, sizeof(struct {
        char topic[64];
        void *data;
        size_t len;
    }));
    
    // 配置MQTT客户端
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
        .credentials.client_id = client_id,
        .buffer.size = MQTT_RX_BUFFER_SIZE,
        .network.disable_auto_reconnect = false
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    conn_callback = conn_cb;
    
    // 注册事件回调
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, event_handler, NULL);
    esp_mqtt_client_start(client);
    
    // 启动队列管理任务
    xTaskCreate(queue_manager_task, "mqtt_queue", 4096, NULL, 5, NULL);
}

static void event_handler(void *args, esp_event_base_t base, 
                        int32_t event_id, void *event_data) 
{
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to broker");
        // 自动订阅图像主题
        esp_mqtt_client_subscribe(client, IMG_TOPIC, 1);
        if (conn_callback) conn_callback(true);
        break;
        
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Disconnected");
        if (conn_callback) conn_callback(false);
        break;
        
    case MQTT_EVENT_DATA:
        // 图像数据处理
        if (strncmp(event->topic, IMG_TOPIC, event->topic_len) == 0) {
            if (img_received + event->data_len > sizeof(img_buffer)) {
                ESP_LOGE(TAG, "Image buffer overflow");
                img_received = 0;
                return;
            }
            
            memcpy(img_buffer + img_received, event->data, event->data_len);
            img_received += event->data_len;
            
            if (img_received >= sizeof(img_buffer)) {
                ESP_LOGI(TAG, "Image received, size: %d", img_received);
                // 此处调用显示接口
                // display_image(img_buffer, sizeof(img_buffer));
                img_received = 0;
            }
        }
        break;
        
    default:
        break;
    }
}

int mqtt_publish(const char *topic, const void *data, size_t len, int qos) {
    if (!client || !topic || !data) return -1;
    return esp_mqtt_client_publish(client, topic, data, len, qos, 0);
}

esp_err_t mqtt_subscribe(const char *topic, int qos) {
    if (!client || !topic) return ESP_FAIL;
    return esp_mqtt_client_subscribe(client, topic, qos);
}

QueueHandle_t mqtt_get_queue(void) {
    return send_queue;
}

static void queue_manager_task(void *arg) {
    struct {
        char topic[64];
        void *data;
        size_t len;
    } queue_item;
    
    while (1) {
        if (xQueueReceive(send_queue, &queue_item, portMAX_DELAY)) {
            if (mqtt_publish(queue_item.topic, queue_item.data, queue_item.len, 1) < 0) {
                ESP_LOGE(TAG, "Message dropped: %s", queue_item.topic);
            }
            free(queue_item.data); // 释放payload内存
        }
    }
}