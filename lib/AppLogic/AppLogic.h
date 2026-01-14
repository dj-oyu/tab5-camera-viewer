#ifndef APPLOGIC_H
#define APPLOGIC_H

#include <M5Unified.h>
#include <cstdint>
#include <driver/jpeg_decode.h>
#include <esp_cache.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

class AppLogic {
public:
  static void begin();
  static void update();

private:
  static void initWiFi();
  static void mjpegFetchTask(void *pvParameters);
  static void mjpegRenderTask(void *pvParameters);
  static bool on_color_trans_done(esp_lcd_panel_handle_t panel,
                                  esp_lcd_dpi_panel_event_data_t *edata,
                                  void *user_ctx);

  struct FrameData {
    uint8_t *buf;
    size_t len;
    bool is_linear; // If true, this buffer belongs to linearBuf pool
  };

  static QueueHandle_t frameQueue;
  static QueueHandle_t linearFreeQueue;
  static SemaphoreHandle_t displayDoneSema;
  static esp_lcd_panel_handle_t panel_handle;
  static uint16_t *decode_bufs[2];
  static int decode_idx;
  static uint16_t *fb;
  static uint8_t *ring_buf;
  static uint8_t *linear_bufs[2];

  static const uint32_t STREAM_WIDTH = 640;
  static const uint32_t STREAM_HEIGHT = 480;
  static const uint32_t RING_BUF_SIZE = 1024 * 1024;
  static const uint32_t LINEAR_BUF_SIZE = 256 * 1024;
  static uint32_t panel_h;
  static uint32_t panel_w;
};

#endif // APPLOGIC_H
