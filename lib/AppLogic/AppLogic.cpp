#include "AppLogic.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <esp_lcd_mipi_dsi.h>
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>

#define STACK_DEPTH 8192

uint32_t AppLogic::panel_h = 0;
uint32_t AppLogic::panel_w = 0;

QueueHandle_t AppLogic::frameQueue = nullptr;
QueueHandle_t AppLogic::linearFreeQueue = nullptr;
QueueHandle_t AppLogic::rbReleaseQueue = nullptr;
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

  M5.Display.println("MJPEG Profiling Mode v29 (Cache Aligned)...");
  Serial.println("AppLogic v29 starting...");

  displayDoneSema = xSemaphoreCreateBinary();
  xSemaphoreGive(displayDoneSema);

  linearFreeQueue = xQueueCreate(2, sizeof(uint8_t *));
  frameQueue = xQueueCreate(2, sizeof(FrameData));
  rbReleaseQueue = xQueueCreate(64, sizeof(size_t));

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

  esp_lcd_dpi_panel_event_callbacks_t cbs = {.on_color_trans_done =
                                                 on_color_trans_done};
  esp_lcd_dpi_panel_register_event_callbacks(panel_handle, &cbs, NULL);

  auto detail = dsi->config_detail();
  fb = (uint16_t *)detail.buffer;

  // Allocate 2 decode buffers in PSRAM
  for (int i = 0; i < 2; i++) {
    decode_bufs[i] = (uint16_t *)heap_caps_aligned_alloc(
        64, STREAM_WIDTH * STREAM_HEIGHT * 2,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }

  // v27: Wait for DSI/DMA2D hardware to fully stabilize before starting tasks
  vTaskDelay(500);

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

              // v28: Guaranteed 128-byte Lookahead for EOI detection
              if (found_eoi || found_next_soi) {
                // v28: Need 128B margin after frame for HW DMA lookahead
                uint32_t needed_margin = 128;
                bool is_contiguous =
                    (rb_parsed + len + needed_margin <= RING_BUF_SIZE);
                bool has_margins =
                    ((rb_head - (rb_parsed + len)) & RB_MASK) >= needed_margin;
                // v29: Strict 64-byte alignment requirement for cache sync
                bool is_aligned = (((uintptr_t)&ring_buf[rb_parsed] & 63) == 0);

                if (is_contiguous && has_margins && is_aligned &&
                    !found_next_soi) {
                  // Zero-copy path: directly reference Ring Buffer
                  fdPending.buf = &ring_buf[rb_parsed];
                  fdPending.len =
                      len; // Renderer will use process_len = len + 128
                  fdPending.is_linear = false;
                  // Sync frame + lookahead margin (must be 64-aligned size)
                  esp_cache_msync(fdPending.buf, (len + 128 + 63) & ~63,
                                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
                  pendingFrame = true;
                } else {
                  // Linear copy path: for wrapped or unaligned frames
                  uint8_t *lbuf = nullptr;
                  if (xQueueReceive(linearFreeQueue, &lbuf, 0) == pdTRUE) {
                    if (rb_parsed + len <= RING_BUF_SIZE) {
                      memcpy(lbuf, &ring_buf[rb_parsed], len);
                    } else {
                      uint32_t f1 = RING_BUF_SIZE - rb_parsed;
                      memcpy(lbuf, &ring_buf[rb_parsed], f1);
                      memcpy(lbuf + f1, &ring_buf[0], len - f1);
                    }

                    uint32_t bitstream_len = len;
                    if (found_next_soi) {
                      lbuf[bitstream_len++] = 0xFF;
                      lbuf[bitstream_len++] = 0xD9;
                    }

                    // v28: Append 128 bytes of 0xFF for lookahead safety
                    uint32_t padded_total = (bitstream_len + 128 + 63) & ~63;
                    memset(lbuf + bitstream_len, 0xFF,
                           padded_total - bitstream_len);

                    fdPending.buf = lbuf;
                    fdPending.len = bitstream_len;
                    fdPending.is_linear = true;
                    esp_cache_msync(lbuf, padded_total,
                                    ESP_CACHE_MSYNC_FLAG_DIR_C2M);
                    pendingFrame = true;
                  }
                }

                if (pendingFrame) {
                  rb_parsed = (rb_parsed + len) & RB_MASK;
                  if (fdPending.is_linear) {
                    xQueueSend(rbReleaseQueue, &len, 0);
                  }
                  searched_len = 0;

                  frame_count++;
                  if (frame_count >= 100) {
                    uint32_t now = millis();
                    float fps = 100000.0f / (now - last_fps_time);
                    Serial.printf("Fetcher FPS: %.1f, Len: %u, ZC: %d\n", fps,
                                  len, !fdPending.is_linear);
                    frame_count = 0;
                    last_fps_time = now;
                  }
                }
              }
            }
          }
        }
        // v17: Emergency Frame Recovery
        // If buffer is nearly full and we haven't found a
        // frame,強制的にスキップ
        uint32_t rb_fill = (rb_head - rb_tail) & RB_MASK;
        if (rb_fill >= RING_BUF_SIZE - 32768) {
          uint32_t skipped = 0;
          uint32_t btp = (rb_head - rb_parsed) & RB_MASK;
          while (btp >= 2 && skipped < 16384) { // Look for SOI
            if (ring_buf[rb_parsed] == 0xFF &&
                ring_buf[(rb_parsed + 1) & RB_MASK] == 0xD8)
              break;
            rb_parsed = (rb_parsed + 1) & RB_MASK;
            btp--;
            skipped++;
          }
          if (skipped > 0) {
            if (btp < 2) { // No SOI found, just clear 16KB
              rb_parsed = (rb_parsed + 16384) & RB_MASK;
              uint32_t force_skip = 16384;
              xQueueSend(rbReleaseQueue, &force_skip, 0);
            } else {
              xQueueSend(rbReleaseQueue, &skipped, 0);
            }
          }
          searched_len = 0;
        }
        if (pendingFrame) {
          if (xQueueSend(frameQueue, &fdPending, 0) == pdTRUE) {
            pendingFrame = false;
          }
        }
        // v17: Mandatory yield to satisfy Task Watchdog on Core 0
        vTaskDelay(1);
      }

      http.end();
      vTaskDelay(1000);
    }
  }
#endif
}

void AppLogic::mjpegRenderTask(void *pvParameters) {
  uint32_t frame_count = 0;
  uint32_t last_fps_time = millis();
  uint32_t consecutive_errors = 0;
  jpeg_decoder_handle_t decoder = nullptr;
  jpeg_decode_engine_cfg_t eng_cfg = {.intr_priority = 0, .timeout_ms = 100};

  auto create_engine = [&]() {
    if (decoder)
      jpeg_del_decoder_engine(decoder);
    if (jpeg_new_decoder_engine(&eng_cfg, &decoder) != ESP_OK) {
      Serial.println("Failed to create JPEG decoder engine");
      return false;
    }
    return true;
  };

  if (!create_engine())
    return;

  jpeg_decode_cfg_t dec_cfg = {
      .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
      .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
      .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
  };

  FrameData fd;
  uint32_t x_offset = (panel_w - STREAM_WIDTH) / 2;
  uint32_t y_offset = (panel_h - STREAM_HEIGHT) / 2;

  while (1) {
    if (xQueueReceive(frameQueue, &fd, pdMS_TO_TICKS(500)) == pdTRUE) {
      uint32_t out_size = 0;

      // v19: Deadlock Resolution
      // Use timeout to prevent permanent hang if callback is missed
      if (xSemaphoreTake(displayDoneSema, pdMS_TO_TICKS(500)) != pdTRUE) {
        Serial.println("Renderer: Display Deadlock! Force recovery...");
        xSemaphoreGive(displayDoneSema);
        xSemaphoreTake(displayDoneSema, 0);
      }
      uint32_t t_start = esp_timer_get_time();

      // v28: Robust Lookahead (Frame + 128B) to satisfy HW DMA
      size_t process_len = fd.len + 128;

      if (jpeg_decoder_process(decoder, &dec_cfg, fd.buf, process_len,
                               (uint8_t *)decode_bufs[decode_idx],
                               STREAM_WIDTH * STREAM_HEIGHT * 2,
                               &out_size) == ESP_OK) {

        uint32_t t_dec = esp_timer_get_time();

        frame_count++;
        if (frame_count >= 100) {
          uint32_t now = millis();
          float fps = 100000.0f / (now - last_fps_time);
          Serial.printf("Render FPS: %.1f\n", fps);
          frame_count = 0;
          last_fps_time = now;
        }

        // Release input buffer early
        if (fd.is_linear) {
          xQueueSend(linearFreeQueue, &fd.buf, 0);
        } else {
          xQueueSend(rbReleaseQueue, &fd.len, 0);
        }

        // Cache Sync: decode_bufs was written by HW (Jpeg).
        // Manually align up out_size.
        size_t aligned_out = (out_size + 63) & ~63;
        esp_cache_msync(decode_bufs[decode_idx], aligned_out,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                            ESP_CACHE_MSYNC_FLAG_INVALIDATE);

        // v18: Pure IDF Rendering bypassing LGFX
        // This is non-blocking; we wait for displayDoneSema at the start of
        // next loop.
        esp_lcd_panel_draw_bitmap(
            panel_handle, x_offset, y_offset, x_offset + STREAM_WIDTH,
            y_offset + STREAM_HEIGHT, decode_bufs[decode_idx]);

        uint32_t t_draw = esp_timer_get_time();
        if (frame_count == 1) { // Log once every 100 frames
          Serial.printf("Timing: Dec %uus, Draw %uus\n",
                        (uint32_t)(t_dec - t_start),
                        (uint32_t)(t_draw - t_dec));
        }

        // Swap decode buffer (Bitwise optimization)
        decode_idx ^= 1;
        taskYIELD();

      } else {
        Serial.printf("Decode Error for len %u\n", fd.len);
        // v19: Release semaphore even on error, otherwise next loop deadlocks
        xSemaphoreGive(displayDoneSema);
        // v26: Optimization: Don't create_engine on single errors.
        consecutive_errors++;
        vTaskDelay(5); // v28: Allow DMA2D FSM to clear
        if (consecutive_errors > 10) {
          Serial.println(
              "Watchdog: Persistent Decode Errors. Resetting Engine...");
          create_engine();
          consecutive_errors = 0;
        }
        // Release buffer even on error
        if (fd.is_linear) {
          xQueueSend(linearFreeQueue, &fd.buf, 0);
        } else {
          xQueueSend(rbReleaseQueue, &fd.len, 0);
        }
      }
    } else {
      // Renderer Stall Warning (v16)
      // If we wait >500ms for a frame, something is wrong in the fetcher or
      // network.
      Serial.println("Renderer: Frame Timeout (500ms). Stall detected.");
    }

    // Signal Fetcher loop is not needed as we use linearFreeQueue
    taskYIELD();
  }
}
