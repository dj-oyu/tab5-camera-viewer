#include "AppLogic.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <esp_lcd_mipi_dsi.h>
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>

#define STACK_DEPTH 16384
#define BITSTREAM_PAD 64

uint32_t AppLogic::panel_h = 0;
uint32_t AppLogic::panel_w = 0;

QueueHandle_t AppLogic::frameQueue = nullptr;
QueueHandle_t AppLogic::linearFreeQueue = nullptr;
esp_lcd_panel_handle_t AppLogic::panel_handle = nullptr;
uint16_t *AppLogic::decode_bufs[2] = {nullptr, nullptr};
int AppLogic::decode_idx = 0;
uint16_t *AppLogic::fb = nullptr;
uint8_t *AppLogic::ring_buf = nullptr;
uint8_t *AppLogic::linear_bufs[2] = {nullptr, nullptr};
SemaphoreHandle_t AppLogic::displayDoneSema = nullptr;

// Helper class to access protected member
class Panel_DSI_Accessor : public lgfx::Panel_DSI {
public:
  esp_lcd_panel_handle_t getHandle() { return _disp_panel_handle; }
};

void AppLogic::begin() {
  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Display.fillScreen(TFT_BLACK);
  panel_h = M5.Display.height();
  panel_w = M5.Display.width();

  // v62: Smart-Staging MJPEG (Strategy B)
  Serial.println("AppLogic v62 starting...");

  displayDoneSema = xSemaphoreCreateBinary();
  xSemaphoreGive(displayDoneSema);

  linearFreeQueue = xQueueCreate(2, sizeof(uint8_t *));
  frameQueue = xQueueCreate(2, sizeof(FrameData));

  // v60: Minimal safety margins
  ring_buf = (uint8_t *)heap_caps_aligned_alloc(
      64, RING_BUF_SIZE + 256, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  for (int i = 0; i < 2; i++) {
    linear_bufs[i] = (uint8_t *)heap_caps_aligned_alloc(
        64, LINEAR_BUF_SIZE + BITSTREAM_PAD + 64,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *ptr = linear_bufs[i];
    xQueueSend(linearFreeQueue, &ptr, 0);
  }

  // Get Panel Handle
  auto dsi = static_cast<lgfx::Panel_DSI *>(M5.Display.getPanel());
  auto accessor = static_cast<Panel_DSI_Accessor *>(dsi);
  panel_handle = accessor->getHandle();

  esp_lcd_dpi_panel_event_callbacks_t cbs = {.on_color_trans_done =
                                                 on_color_trans_done};
  esp_lcd_dpi_panel_register_event_callbacks(panel_handle, &cbs, NULL);

  auto detail = dsi->config_detail();
  fb = (uint16_t *)detail.buffer;

  for (int i = 0; i < 2; i++) {
    decode_bufs[i] = (uint16_t *)heap_caps_aligned_alloc(
        64, panel_w * panel_h * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }

  vTaskDelay(1000);

  // v61: Both on Core 1 (Shared Cache) at equal priority
  xTaskCreatePinnedToCore(mjpegFetchTask, "Fetch", STACK_DEPTH, NULL, 5, NULL,
                          1);
  xTaskCreatePinnedToCore(mjpegRenderTask, "Render", STACK_DEPTH, NULL, 5, NULL,
                          1);
}

void AppLogic::update() { M5.update(); }

bool IRAM_ATTR AppLogic::on_color_trans_done(
    esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata,
    void *user_ctx) {
  BaseType_t high_priority_task_awoken = pdFALSE;
  if (displayDoneSema) {
    xSemaphoreGiveFromISR(displayDoneSema, &high_priority_task_awoken);
  }
  return high_priority_task_awoken == pdTRUE;
}

void AppLogic::initWiFi() {
#ifdef WIFI_SSID
  WiFi.setPins(12, 13, 11, 10, 9, 8, 15);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
#endif
}

void AppLogic::mjpegFetchTask(void *pvParameters) {
  initWiFi();
#ifdef MJPEG_URL
  HTTPClient http;
  const uint32_t RB_MASK = RING_BUF_SIZE - 1;

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(1000);
      continue;
    }

    http.begin(MJPEG_URL);
    http.setReuse(false);
    http.setTimeout(10000);

    if (http.GET() != 200) {
      Serial.println("HTTP GET failed");
      http.end();
      vTaskDelay(1000);
      continue;
    }

    // v62: Use auto& to avoid Stream alignment/type issues
    auto &stream = http.getStream();
    stream.setTimeout(1000);

    uint32_t last_data_time = millis();
    uint32_t last_sub_time = millis();
    uint32_t rb_head = 0;
    uint32_t rb_tail = 0;
    uint32_t rb_parsed = 0;

    enum ParseState { FIND_SOI, FETCH_JPEG };
    ParseState state = FIND_SOI;
    uint8_t *active_lbuf = nullptr;
    uint32_t active_lptr = 0;

    while (http.connected()) {
      bool made_progress = false;

      // 1. Fill Ring Buffer
      int avail = stream.available();
      if (avail > 0) {
        uint32_t rb_fill = (rb_head - rb_tail) & RB_MASK;
        uint32_t space = RING_BUF_SIZE - rb_fill - 1;
        if (space > 0) {
          uint32_t to_read = std::min((uint32_t)avail, space);
          uint32_t first_part = std::min(to_read, RING_BUF_SIZE - rb_head);
          int r1 = stream.readBytes(&ring_buf[rb_head], first_part);
          if (r1 > 0) {
            rb_head = (rb_head + r1) & RB_MASK;
            made_progress = true;
            last_data_time = millis();
            if (r1 < (int)to_read) {
              int r2 = stream.readBytes(&ring_buf[rb_head], to_read - r1);
              if (r2 > 0) {
                rb_head = (rb_head + r2) & RB_MASK;
              }
            }
          }
        }
      }

      // 2. State Machine
      while (((rb_head - rb_parsed) & RB_MASK) > 0) {
        uint8_t b = ring_buf[rb_parsed & RB_MASK];

        if (state == FIND_SOI) {
          if (b == 0xFF && ((rb_head - rb_parsed) & RB_MASK) >= 2) {
            if (ring_buf[(rb_parsed + 1) & RB_MASK] == 0xD8) {
              if (xQueueReceive(linearFreeQueue, &active_lbuf, 0) == pdTRUE) {
                active_lptr = 0;
                active_lbuf[active_lptr++] = 0xFF;
                active_lbuf[active_lptr++] = 0xD8;
                rb_parsed = (rb_parsed + 2) & RB_MASK;
                state = FETCH_JPEG;
                made_progress = true;
                continue;
              }
            }
          }
          rb_parsed = (rb_parsed + 1) & RB_MASK;
          rb_tail = rb_parsed;
          made_progress = true;
        } else if (state == FETCH_JPEG) {
          if (active_lptr < LINEAR_BUF_SIZE) {
            active_lbuf[active_lptr++] = b;
            rb_parsed = (rb_parsed + 1) & RB_MASK;
            made_progress = true;

            if (active_lptr >= 2 && active_lbuf[active_lptr - 2] == 0xFF &&
                active_lbuf[active_lptr - 1] == 0xD9) {
              size_t l_pad = (active_lptr + BITSTREAM_PAD + 63) & ~63;
              memset(active_lbuf + active_lptr, 0, l_pad - active_lptr);
              esp_cache_msync((void *)active_lbuf, l_pad,
                              ESP_CACHE_MSYNC_FLAG_DIR_C2M);

              FrameData fd;
              fd.buf = active_lbuf;
              fd.len = active_lptr;
              fd.is_linear = true;
              xQueueSend(frameQueue, &fd, 0);
              last_sub_time = millis();

              rb_tail = rb_parsed;
              active_lbuf = nullptr;
              state = FIND_SOI;
            }
          } else {
            Serial.println("Fetcher: Linear Overflow!");
            xQueueSend(linearFreeQueue, &active_lbuf, 0);
            active_lbuf = nullptr;
            state = FIND_SOI;
          }
        }
      }

      if (!made_progress) {
        vTaskDelay(1);
      } else {
        taskYIELD();
      }

      uint32_t now = millis();
      if (now - last_data_time > 5000) {
        Serial.println("Fetcher: Data Timeout!");
        break;
      }
      if (now - last_sub_time > 10000) {
        Serial.println("Fetcher: Stall!");
        break;
      }
    }
    if (active_lbuf)
      xQueueSend(linearFreeQueue, &active_lbuf, 0);
    http.end();
    vTaskDelay(1000);
  }
#endif
}

void AppLogic::mjpegRenderTask(void *pvParameters) {
  uint32_t frame_count = 0;
  uint32_t last_fps_time = millis();
  jpeg_decoder_handle_t decoder = nullptr;
  jpeg_decode_engine_cfg_t eng_cfg = {.intr_priority = 0, .timeout_ms = 100};
  jpeg_decode_cfg_t dec_cfg = {
      .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
      .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
      .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
  };

  FrameData fd;
  while (1) {
    if (xQueueReceive(frameQueue, &fd, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (!decoder) {
        jpeg_new_decoder_engine(&eng_cfg, &decoder);
      }

      uint32_t out_size = 0;
      size_t process_len = (fd.len + BITSTREAM_PAD + 63) & ~63;

      esp_cache_msync((void *)fd.buf, process_len,
                      ESP_CACHE_MSYNC_FLAG_INVALIDATE);
      __asm__ __volatile__("fence" ::: "memory");

      if (xSemaphoreTake(displayDoneSema, pdMS_TO_TICKS(500)) == pdTRUE) {
        jpeg_decoder_process(decoder, &dec_cfg, fd.buf, process_len,
                             (uint8_t *)decode_bufs[decode_idx],
                             panel_w * panel_h * 2, &out_size);

        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, STREAM_WIDTH,
                                  STREAM_HEIGHT, decode_bufs[decode_idx]);

        decode_idx ^= 1;
        frame_count++;
        if (frame_count >= 100) {
          uint32_t now = millis();
          Serial.printf("Render FPS: %.1f\n",
                        100.0f * 1000.0f / (now - last_fps_time));
          last_fps_time = now;
          frame_count = 0;
        }
      } else {
        Serial.println("Renderer: DSI Timeout");
        xSemaphoreGive(displayDoneSema);
      }

      if (fd.is_linear) {
        xQueueSend(linearFreeQueue, &fd.buf, 0);
      }
    } else {
      vTaskDelay(1);
    }
  }
}
