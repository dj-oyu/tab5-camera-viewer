#ifndef APPLOGIC_H
#define APPLOGIC_H

#include <M5Unified.h>
#include <cstdint>
#include <driver/jpeg_decode.h>
#include <esp_cache.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

class AppLogic {
public:
  static void begin();
  static void update();

private:
  static bool find_FF_pos(uint8_t *buf, uint8_t adjacent, size_t len, uint8_t **pos);
  static void initWiFi();
  static void mjpegFetchTask(void *pvParameters);
  static void mjpegRenderTask(void *pvParameters);

  struct FrameData {
    uint8_t *buf;
    size_t len;
  };

  static QueueHandle_t frameQueue;
  static QueueHandle_t freeQueue;
  static uint16_t *fb;
  static uint16_t *decode_buf;

  static const uint32_t STREAM_WIDTH = 640;
  static const uint32_t STREAM_HEIGHT = 480;
  static uint32_t panel_h;
  static uint32_t panel_w;
};

#endif // APPLOGIC_H
