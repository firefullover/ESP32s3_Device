#ifndef __ST7789_DRIVER_H__
#define __ST7789_DRIVER_H__

#include "driver/spi_master.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 硬件配置 */
#define ST7789_SPI_HOST       SPI3_HOST
#define ST7789_MOSI_PIN       11      // SPI数据线
#define ST7789_SCLK_PIN       12      // SPI时钟线
#define ST7789_RES_PIN        6       // 复位引脚
#define ST7789_DC_PIN         7       // 数据/命令控制引脚
#define ST7789_PIXEL_FORMAT   0x55    // 16-bit RGB565
#define ST7789_MAX_TRANS_SIZE 4096    // SPI单次传输最大字节数

/* 屏幕参数 */
#define ST7789_WIDTH          240
#define ST7789_HEIGHT         240

/**
 * @brief  初始化ST7789显示屏
 * @note  包含GPIO、SPI总线和设备初始化
 */
void st7789_init(void);

/**
 * @brief 清屏操作
 * @param color 16位RGB颜色值
 */
void st7789_fill_screen(uint16_t color);

/**
 * @brief 绘制RGB565图像
 * @param image_data 图像数据指针
 * @param width 图像宽度（最大240）
 * @param height 图像高度（最大240）
 */
void st7789_draw_image(const uint16_t *image_data);

/**
 * @brief 校验图像是否符合要求
 * @param data 图像数据指针（需为240x240x2字节）
 * @param len 数据长度（应等于240*240*2）
 */
void st7789_display_raw(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif //__ST7789_DRIVER_H__