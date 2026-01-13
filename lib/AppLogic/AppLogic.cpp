#include "AppLogic.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>

#define STACK_DEPTH 8192

uint32_t AppLogic::panel_h = 0;
uint32_t AppLogic::panel_w = 0;

QueueHandle_t AppLogic::frameQueue = nullptr;
QueueHandle_t AppLogic::linearFreeQueue = nullptr;
QueueHandle_t AppLogic::rbReleaseQueue = nullptr;
SemaphoreHandle_t AppLogic::renderReadySema = nullptr;
esp_lcd_panel_handle_t AppLogic::panel_handle = nullptr;
uint16_t *AppLogic::decode_bufs[2] = {nullptr, nullptr};
int AppLogic::decode_idx = 0;
uint16_t *AppLogic::fb = nullptr;
uint8_t *AppLogic::ring_buf = nullptr;
uint8_t *AppLogic::linear_bufs[2] = {nullptr, nullptr};

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

  M5.Display.println("MJPEG Profiling Mode v4...");
  Serial.println("AppLogic v4 starting...");

  linearFreeQueue = xQueueCreate(2, sizeof(uint8_t *));
  frameQueue = xQueueCreate(2, sizeof(FrameData));
  rbReleaseQueue = xQueueCreate(64, sizeof(size_t));
  renderReadySema = xSemaphoreCreateBinary();
  xSemaphoreGive(renderReadySema);

  // Add padding for safe cache sync beyond buffer end
  ring_buf = (uint8_t *)heap_caps_aligned_alloc(
      64, RING_BUF_SIZE + 128, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  for (int i = 0; i < 2; i++) {
    linear_bufs[i] = (uint8_t *)heap_caps_aligned_alloc(
        64, LINEAR_BUF_SIZE + 128, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *ptr = linear_bufs[i];
    xQueueSend(linearFreeQueue, &ptr, 0);
  }

  // Get Panel Handle
  auto dsi = static_cast<lgfx::Panel_DSI *>(M5.Display.getPanel());
  auto accessor = static_cast<Panel_DSI_Accessor *>(dsi);
  panel_handle = accessor->getHandle();

  auto detail = dsi->config_detail();
  fb = (uint16_t *)detail.buffer;

  // Allocate 2 decode buffers in PSRAM
  for (int i = 0; i < 2; i++) {
    decode_bufs[i] = (uint16_t *)heap_caps_aligned_alloc(
        64, STREAM_WIDTH * STREAM_HEIGHT * 2,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }

  xTaskCreatePinnedToCore(mjpegFetchTask, "Fetch", STACK_DEPTH, NULL, 5, NULL,
                          0);
  xTaskCreatePinnedToCore(mjpegRenderTask, "Render", STACK_DEPTH, NULL, 5, NULL,
                          1);
}

void AppLogic::update() { M5.update(); }

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
  uint32_t rb_head = 0;      // Writer index
  uint32_t rb_tail = 0;      // Oldest byte still in use
  uint32_t rb_parsed = 0;    // Next byte to parse
  uint32_t searched_len = 0; // How much of current frame scanned
  bool pendingFrame = false;
  FrameData fdPending;
  uint32_t frame_count = 0;
  uint32_t last_fps_time = millis();

  // Pre-calculate mask for modulo operations (RING_BUF_SIZE must be Power of 2)
  const uint32_t RB_MASK = RING_BUF_SIZE - 1;

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(1000);
      continue;
    }

    http.begin(MJPEG_URL);
    if (http.GET() == 200) {
      WiFiClient *stream = http.getStreamPtr();
      stream->setTimeout(50);
      rb_head = rb_tail = rb_parsed = 0;
      searched_len = 0;
      pendingFrame = false;

      while (stream->connected()) {
        // 1. Reclaim space
        size_t relLength;
        while (xQueueReceive(rbReleaseQueue, &relLength, 0) == pdTRUE) {
          rb_tail = (rb_tail + relLength) & RB_MASK;
        }

        // 2. Read from stream into RB
        int avail = stream->available();
        if (avail > 0) {
          uint32_t rb_fill = (rb_head - rb_tail) & RB_MASK;
          uint32_t space = RING_BUF_SIZE - rb_fill - 1;
          if (space > 0) {
            uint32_t to_read = std::min((uint32_t)avail, space);
            uint32_t first_part = std::min(to_read, RING_BUF_SIZE - rb_head);
            int r1 = stream->read(&ring_buf[rb_head], first_part);
            if (r1 > 0) {
              rb_head = (rb_head + r1) & RB_MASK;
              if (r1 < (int)to_read) {
                int r2 = stream->read(&ring_buf[rb_head], to_read - r1);
                if (r2 > 0)
                  rb_head = (rb_head + r2) & RB_MASK;
              }
            }
          }
        }

        // 3. Parse Frame
        if (!pendingFrame) {
          uint32_t bytes_to_parse = (rb_head - rb_parsed) & RB_MASK;
          uint32_t rb_fill = (rb_head - rb_tail) & RB_MASK;

          if (bytes_to_parse >= 2) {
            bool found_soi = false;
            if (ring_buf[rb_parsed] == 0xFF &&
                ring_buf[(rb_parsed + 1) & RB_MASK] == 0xD8) {
              found_soi = true;
            } else {
              uint32_t junk = 0;
              while (bytes_to_parse >= 2) {
                if (ring_buf[rb_parsed] == 0xFF &&
                    ring_buf[(rb_parsed + 1) & RB_MASK] == 0xD8) {
                  found_soi = true;
                  break;
                }
                rb_parsed = (rb_parsed + 1) & RB_MASK;
                bytes_to_parse--;
                junk++;
              }
              if (junk > 0)
                xQueueSend(rbReleaseQueue, &junk, 0);
              searched_len = 0;
            }

            if (found_soi) {
              if (searched_len < 2)
                searched_len = 2;
              uint32_t len = searched_len;
              uint32_t search = (rb_parsed + len) & RB_MASK;
              bool found_eoi = false;
              bool found_next_soi = false;

              while (len + 1 < bytes_to_parse) {
                if (ring_buf[search] == 0xFF) {
                  uint8_t m = ring_buf[(search + 1) & RB_MASK];
                  if (m == 0xD9) {
                    len += 2;
                    found_eoi = true;
                    break;
                  } else if (m == 0xD8) {
                    found_next_soi = true;
                    break;
                  }
                }
                search = (search + 1) & RB_MASK;
                len++;
              }
              searched_len = len;

              if (found_eoi || found_next_soi) {
                fdPending.len = len;
                bool is_contiguous = (rb_parsed + len <= RING_BUF_SIZE);
                bool is_aligned = (((uintptr_t)&ring_buf[rb_parsed] & 63) == 0);

                if (is_contiguous && is_aligned && !found_next_soi) {
                  fdPending.buf = &ring_buf[rb_parsed];
                  fdPending.is_linear = false;
                  size_t aligned_len = (fdPending.len + 63) & ~63;
                  esp_cache_msync(fdPending.buf, aligned_len,
                                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
                  pendingFrame = true;
                  rb_parsed = (rb_parsed + len) & RB_MASK;
                  searched_len = 0;
                } else {
                  uint8_t *lbuf = nullptr;
                  if (xQueueReceive(linearFreeQueue, &lbuf, 0) == pdTRUE) {
                    if (is_contiguous) {
                      memcpy(lbuf, &ring_buf[rb_parsed], len);
                    } else {
                      uint32_t f1 = RING_BUF_SIZE - rb_parsed;
                      memcpy(lbuf, &ring_buf[rb_parsed], f1);
                      memcpy(lbuf + f1, &ring_buf[0], len - f1);
                    }
                    if (found_next_soi && len <= LINEAR_BUF_SIZE - 2) {
                      lbuf[len++] = 0xFF;
                      lbuf[len++] = 0xD9;
                    }
                    fdPending.buf = lbuf;
                    fdPending.len = len;
                    fdPending.is_linear = true;
                    size_t aligned_len = (len + 63) & ~63;
                    esp_cache_msync(lbuf, aligned_len,
                                    ESP_CACHE_MSYNC_FLAG_DIR_C2M);
                    pendingFrame = true;
                    uint32_t raw_len = found_next_soi ? (len - 2) : len;
                    rb_parsed = (rb_parsed + raw_len) & RB_MASK;
                    xQueueSend(rbReleaseQueue, &raw_len, 0);
                    searched_len = 0;
                  }
                }
              }
            }
          }
          if (rb_fill >= RING_BUF_SIZE - 8192) {
            uint32_t skip = 4096;
            rb_parsed = (rb_parsed + skip) & RB_MASK;
            xQueueSend(rbReleaseQueue, &skip, 0);
            searched_len = 0;
          }
        }

        // 4. Dispatch Frame
        if (pendingFrame) {
          if (xSemaphoreTake(renderReadySema, 0) == pdTRUE) {
            if (xQueueSend(frameQueue, &fdPending, 0) == pdTRUE) {
              pendingFrame = false;
              frame_count++;
              if (frame_count >= 100) {
                uint32_t now = millis();
                float fps = 100000.0f / (now - last_fps_time);
                Serial.printf("FPS: %.1f\n", fps);
                frame_count = 0;
                last_fps_time = now;
              }
            } else {
              xSemaphoreGive(renderReadySema);
            }
          }
        }
        if ((avail <= 0) || pendingFrame) {
          vTaskDelay(1);
        }
      }
      http.end();
      vTaskDelay(1000);
    }
#endif
  }
}

void AppLogic::mjpegRenderTask(void *pvParameters) {
  jpeg_decoder_handle_t decoder;
  jpeg_decode_engine_cfg_t eng_cfg = {.intr_priority = 0, .timeout_ms = 100};
  if (jpeg_new_decoder_engine(&eng_cfg, &decoder) != ESP_OK) {
  }

  jpeg_decode_cfg_t dec_cfg = {
      .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
      .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
      .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
  };

  FrameData fd;
  uint32_t x_offset = (panel_w - STREAM_WIDTH) / 2;
  uint32_t y_offset = (panel_h - STREAM_HEIGHT) / 2;

  while (1) {
    if (xQueueReceive(frameQueue, &fd, portMAX_DELAY) == pdTRUE) {
      uint32_t out_size;

      // Decode into the current double-buffer
      // Pass a slightly larger length (padded) to prevent HW decoder from
      // complaining about missing EOI if it performs look-ahead near the
      // buffer end.
      size_t padded_len = (fd.len + 63) & ~63;
      if (jpeg_decoder_process(decoder, &dec_cfg, fd.buf, fd.len,
                               (uint8_t *)decode_bufs[decode_idx],
                               STREAM_WIDTH * STREAM_HEIGHT * 2,
                               &out_size) == ESP_OK) {

        // Release input buffer so Fetcher can continue
        if (fd.is_linear) {
          xQueueSend(linearFreeQueue, &fd.buf, 0);
        } else {
          xQueueSend(rbReleaseQueue, &fd.len, 0);
        }

        // Cache Sync: decode_bufs was written by HW (Jpeg).
        // Manually align up out_size.
        size_t aligned_len = (out_size + 63) & ~63;
        esp_cache_msync(decode_bufs[decode_idx], aligned_len,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                            ESP_CACHE_MSYNC_FLAG_INVALIDATE);

        // Draw to Panel using IDF API
        if (panel_handle) {
          esp_err_t ret;
          // Retry loop for non-blocking API
          do {
            ret = esp_lcd_panel_draw_bitmap(
                panel_handle, x_offset, y_offset, x_offset + STREAM_WIDTH,
                y_offset + STREAM_HEIGHT, decode_bufs[decode_idx]);
            if (ret != ESP_OK) {
              vTaskDelay(1);
            }
          } while (ret != ESP_OK);
        }

        // Swap decode buffer (Bitwise optimization)
        decode_idx ^= 1;

      } else {
        // Handle decode fail
        if (fd.is_linear) {
          xQueueSend(linearFreeQueue, &fd.buf, 0);
        } else {
          xQueueSend(rbReleaseQueue, &fd.len, 0);
        }
      }

      // Signal Fetcher
      xSemaphoreGive(renderReadySema);
    }
    taskYIELD();
  }
}
