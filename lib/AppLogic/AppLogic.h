#ifndef APPLOGIC_H
#define APPLOGIC_H

#include <M5Unified.h>
#include <PPAPipeline.h>
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

class AppLogic
{
public:
  static void begin();
  static void update();

private:
  static void initWiFi();
  static void mjpegFetchTask(void *pvParameters);
  static void mjpegDecodeTask(void *pvParameters);
  static void mjpegRenderTask(void *pvParameters);
  static bool on_color_trans_done(esp_lcd_panel_handle_t panel,
                                  esp_lcd_dpi_panel_event_data_t *edata,
                                  void *user_ctx);

  struct FrameData
  {
    uint8_t *buf;
    size_t len;
    bool is_linear; // If true, this buffer belongs to linearBuf pool
  };

  struct DecodedFrameData
  {
    int buf_idx;         // Index of decode_bufs[] containing decoded RGB565 data
    uint8_t *linear_buf; // Linear buffer to return (if is_linear was true)
    bool has_linear_buf; // Whether linear_buf needs to be returned
  };

  static QueueHandle_t frameQueue;
  static QueueHandle_t decodedFrameQueue;
  static QueueHandle_t linearFreeQueue;
  static SemaphoreHandle_t displayDoneSema;
  static SemaphoreHandle_t ppaDoneSema;
  static esp_lcd_panel_handle_t panel_handle;
  static uint16_t *decode_bufs[2];
  static int decode_idx;
  static uint16_t *fb;
  static uint8_t *ring_buf;
  static uint8_t *linear_bufs[2];

  // Stream and Panel size constants
  static const uint32_t STREAM_WIDTH = 640;
  static const uint32_t STREAM_HEIGHT = 480;

  // Panel size (M5Stack CoreS3 is 320x240, adjust for your device)
  // For M5Stack Tab5 with portrait orientation: 1024x600
  static const uint32_t PANEL_WIDTH = 720;
  static const uint32_t PANEL_HEIGHT = 1280;

  // Compile-time buffer size calculation (max of stream and panel)
  static const uint32_t DECODE_BUF_SIZE =
      ((STREAM_WIDTH * STREAM_HEIGHT) > (PANEL_WIDTH * PANEL_HEIGHT)
           ? (STREAM_WIDTH * STREAM_HEIGHT)
           : (PANEL_WIDTH * PANEL_HEIGHT)) *
      2;

  static const uint32_t RING_BUF_SIZE = 1024 * 1024;
  static const uint32_t LINEAR_BUF_SIZE = 256 * 1024;

  static const int BG_COLOR = 0x8b008b;

  // Runtime panel size (for dynamic detection, usually matches constants)
  static uint32_t panel_h;
  static uint32_t panel_w;
};

#endif // APPLOGIC_H
