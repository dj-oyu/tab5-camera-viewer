#include "AppLogic.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>

#define STACK_DEPTH 16384
#define BITSTREAM_PAD 64

// Forward declaration for DPI panel event callbacks
extern "C" esp_err_t esp_lcd_dpi_panel_register_event_callbacks(
    esp_lcd_panel_handle_t panel,
    const esp_lcd_dpi_panel_event_callbacks_t *cbs,
    void *user_ctx);

uint32_t AppLogic::panel_h = 0;
uint32_t AppLogic::panel_w = 0;

QueueHandle_t AppLogic::frameQueue = nullptr;
QueueHandle_t AppLogic::decodedFrameQueue = nullptr;
QueueHandle_t AppLogic::linearFreeQueue = nullptr;
esp_lcd_panel_handle_t AppLogic::panel_handle = nullptr;
uint16_t *AppLogic::decode_bufs[2] = {nullptr, nullptr};
int AppLogic::decode_idx = 0;
uint16_t *AppLogic::fb = nullptr;
uint8_t *AppLogic::ring_buf = nullptr;
uint8_t *AppLogic::linear_bufs[2] = {nullptr, nullptr};
SemaphoreHandle_t AppLogic::displayDoneSema = nullptr;
SemaphoreHandle_t AppLogic::ppaDoneSema = nullptr;

// Helper class to access protected member
class Panel_DSI_Accessor : public lgfx::Panel_DSI {
public:
  esp_lcd_panel_handle_t getHandle() { return _disp_panel_handle; }
};

void AppLogic::begin() {
  // Initialize M5Unified for basic peripherals (touch, I2C, etc)
  auto cfg = M5.config();
  cfg.output_power = true;
  M5.begin(cfg);

  panel_w = 720;
  panel_h = 1280;

  Serial.println("AppLogic v64: Dual Framebuffer + PPA Zero-copy");
  Serial.printf("Panel: %dx%d\n", panel_w, panel_h);

  displayDoneSema = xSemaphoreCreateBinary();
  xSemaphoreGive(displayDoneSema);

  ppaDoneSema = xSemaphoreCreateBinary();

  // Initialize PPA Pipeline
  if (!PPAPipeline::begin()) {
    Serial.println("Failed to initialize PPA Pipeline!");
  }

  linearFreeQueue = xQueueCreate(2, sizeof(uint8_t *));
  frameQueue = xQueueCreate(2, sizeof(FrameData));
  decodedFrameQueue = xQueueCreate(2, sizeof(DecodedFrameData));

  ring_buf = (uint8_t *)heap_caps_aligned_alloc(
      64, RING_BUF_SIZE + 256, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  for (int i = 0; i < 2; i++) {
    linear_bufs[i] = (uint8_t *)heap_caps_aligned_alloc(
        64, LINEAR_BUF_SIZE + BITSTREAM_PAD + 64,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    xQueueSend(linearFreeQueue, &linear_bufs[i], 0);
  }

  // Get panel handle and framebuffer from M5Unified
  auto dsi = static_cast<lgfx::Panel_DSI *>(M5.Display.getPanel());
  auto accessor = static_cast<Panel_DSI_Accessor *>(dsi);
  panel_handle = accessor->getHandle();

  // Get framebuffer using esp_lcd_dpi_panel_get_frame_buffer
  esp_lcd_dpi_panel_get_frame_buffer(panel_handle, 1, (void **)&fb);

  if (fb == nullptr) {
    Serial.println("Failed to get framebuffer, allocating dedicated buffer");
    fb = (uint16_t *)heap_caps_aligned_alloc(
        64, PANEL_WIDTH * PANEL_HEIGHT * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  Serial.printf("Framebuffer: %p\n", fb);

  // Register DSI transfer done callback
  esp_lcd_dpi_panel_event_callbacks_t cbs = {
    .on_color_trans_done = on_color_trans_done,
  };
  esp_lcd_dpi_panel_register_event_callbacks(panel_handle, &cbs, nullptr);
  Serial.println("DSI transfer callback registered");

  // Allocate dual decode buffers for parallel JPEG decode + PPA processing
  for (int i = 0; i < 2; i++) {
    decode_bufs[i] = (uint16_t *)heap_caps_aligned_alloc(
        64, DECODE_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    Serial.printf("Decode buffer[%d]: %p (size=%u)\n", i, decode_bufs[i], DECODE_BUF_SIZE);
    if (decode_bufs[i] == nullptr) {
      Serial.printf("FATAL: Failed to allocate decode buffer[%d]!\n", i);
      while(1) vTaskDelay(1000);
    }
  }

  vTaskDelay(1000);

  // 3-stage pipeline: Fetch -> Decode -> Render (PPA+DSI)
  xTaskCreatePinnedToCore(mjpegFetchTask, "Fetch", STACK_DEPTH, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(mjpegDecodeTask, "Decode", STACK_DEPTH, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(mjpegRenderTask, "Render", STACK_DEPTH, NULL, 5, NULL, 1);
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

    int httpCode = http.GET();
    if (httpCode != 200) {
      Serial.printf("HTTP GET failed, code=%d\n", httpCode);
      http.end();
      vTaskDelay(1000);
      continue;
    }

    Serial.println("Stream connected");
    auto &stream = http.getStream();
    stream.setTimeout(1000);

    uint32_t last_data_time = millis();
    uint32_t last_sub_time = millis();
    uint32_t rb_head = 0;
    uint32_t rb_tail = 0;
    uint32_t rb_parsed = 0;

    enum ParseState { FIND_SOI, SCAN_CHUNK, COPY_MODE };
    ParseState state = FIND_SOI;
    uint8_t *active_lbuf = nullptr;
    uint32_t active_lptr = 0;

    // chunkedエンコーディング処理用
    enum ChunkedState { CHUNK_SIZE, CHUNK_DATA, CHUNK_TRAILER };
    ChunkedState chunk_state = CHUNK_SIZE;
    uint32_t chunk_remaining = 0;
    uint32_t chunk_data_start = 0; // 現在のチャンクデータ開始位置
    char chunk_size_buf[16];
    uint8_t chunk_size_idx = 0;
    bool seen_cr = false;

    // ゼロコピー用
    uint32_t frame_start_pos = 0;
    bool needs_copy = false;

    // chunkedレイヤー処理: ring_bufferの生データを解析してチャンクを追跡
    auto processChunkedLayer = [&]() {
      while (((rb_head - rb_tail) & RB_MASK) > 0) {
        uint8_t raw = ring_buf[rb_tail & RB_MASK];

        switch (chunk_state) {
        case CHUNK_SIZE:
          rb_tail = (rb_tail + 1) & RB_MASK;
          if (raw == '\r') {
            // \rをスキップ
          } else if (raw == '\n') {
            chunk_size_buf[chunk_size_idx] = '\0';
            if (chunk_size_idx > 0) {
              chunk_remaining = strtoul(chunk_size_buf, nullptr, 16);
              chunk_size_idx = 0;
              if (chunk_remaining == 0) {
                return false; // 終了チャンク
              }
              chunk_data_start = rb_tail;
              chunk_state = CHUNK_DATA;
              return true;
            }
          } else if ((raw >= '0' && raw <= '9') ||
                     (raw >= 'a' && raw <= 'f') ||
                     (raw >= 'A' && raw <= 'F')) {
            if (chunk_size_idx < 15) {
              chunk_size_buf[chunk_size_idx++] = raw;
            }
          }
          break;

        case CHUNK_DATA:
          // データは消費しない、rb_tailはそのまま
          return true;

        case CHUNK_TRAILER:
          rb_tail = (rb_tail + 1) & RB_MASK;
          if (raw == '\r') {
            seen_cr = true;
          } else if (raw == '\n' && seen_cr) {
            chunk_state = CHUNK_SIZE;
            seen_cr = false;
            return true;
          }
          break;
        }
      }
      return false;
    };

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

      // 2. Chunkedレイヤー処理
      processChunkedLayer();

      // 3. State Machine（ゼロコピー最適化版）
      if (chunk_state == CHUNK_DATA && chunk_remaining > 0) {
        uint32_t available = std::min(chunk_remaining,
                                      (rb_head - rb_tail) & RB_MASK);
        if (available == 0) {
          // データ待ち
        } else if (state == FIND_SOI) {
          // SOIを探索
          for (uint32_t i = 0; i < available - 1; i++) {
            uint32_t pos = (rb_tail + i) & RB_MASK;
            if (ring_buf[pos] == 0xFF && ring_buf[(pos + 1) & RB_MASK] == 0xD8) {
              frame_start_pos = pos;
              rb_parsed = (pos + 2) & RB_MASK;
              rb_tail = rb_parsed;
              chunk_remaining -= (i + 2);
              state = SCAN_CHUNK;
              needs_copy = false;
              made_progress = true;
              break;
            }
          }
          if (state == FIND_SOI) {
            // SOI未発見、データを破棄
            rb_tail = (rb_tail + available) & RB_MASK;
            chunk_remaining -= available;
            if (chunk_remaining == 0) {
              chunk_state = CHUNK_TRAILER;
              seen_cr = false;
            }
          }
        } else if (state == SCAN_CHUNK) {
          // chunk内でEOIを探索（ゼロコピー判定）
          uint32_t scan_end = rb_tail + available;
          for (uint32_t i = 0; i < available - 1; i++) {
            uint32_t pos = (rb_tail + i) & RB_MASK;
            if (ring_buf[pos] == 0xFF && ring_buf[(pos + 1) & RB_MASK] == 0xD9) {
              // EOI発見: chunk内完結 -> ゼロコピー可能
              uint32_t frame_len = ((pos + 2) - frame_start_pos) & RB_MASK;

              // Ring bufferが連続領域か確認
              if (frame_start_pos + frame_len <= RING_BUF_SIZE) {
                // 連続領域 -> ゼロコピー
                size_t pad_len = (frame_len + BITSTREAM_PAD + 63) & ~63;
                if (frame_start_pos + pad_len <= RING_BUF_SIZE) {
                  memset(ring_buf + frame_start_pos + frame_len, 0,
                         pad_len - frame_len);
                  esp_cache_msync((void *)(ring_buf + frame_start_pos), pad_len,
                                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);

                  FrameData fd;
                  fd.buf = ring_buf + frame_start_pos;
                  fd.len = frame_len;
                  fd.is_linear = false; // Ring buffer参照
                  xQueueSend(frameQueue, &fd, 0);
                  last_sub_time = millis();

                  rb_tail = (pos + 2) & RB_MASK;
                  rb_parsed = rb_tail;
                  chunk_remaining -= (i + 2);
                  state = FIND_SOI;
                  made_progress = true;
                  break;
                }
              }

              // ゼロコピー不可 -> コピーモードへ
              needs_copy = true;
              state = COPY_MODE;
              break;
            }
          }

          if (state == SCAN_CHUNK) {
            // EOI未発見
            rb_tail = (rb_tail + available) & RB_MASK;
            rb_parsed = rb_tail;
            chunk_remaining -= available;
            if (chunk_remaining == 0) {
              // Chunk終了、次のchunkにまたがる -> コピーモード
              needs_copy = true;
              state = COPY_MODE;
              chunk_state = CHUNK_TRAILER;
              seen_cr = false;
            }
          }
        }

        if (state == COPY_MODE) {
          // Linear bufferにコピー
          if (!active_lbuf) {
            if (xQueueReceive(linearFreeQueue, &active_lbuf, 0) != pdTRUE) {
              // バッファなし、次回まで待機
            } else {
              active_lptr = 0;
              // フレーム開始位置からコピー
              uint32_t copy_len = (rb_parsed - frame_start_pos) & RB_MASK;
              for (uint32_t i = 0; i < copy_len; i++) {
                active_lbuf[active_lptr++] = ring_buf[(frame_start_pos + i) & RB_MASK];
              }
            }
          }

          if (active_lbuf) {
            // 継続してコピー
            while (chunk_state == CHUNK_DATA && chunk_remaining > 0 &&
                   ((rb_head - rb_tail) & RB_MASK) > 0 && active_lptr < LINEAR_BUF_SIZE) {
              uint8_t b = ring_buf[rb_tail & RB_MASK];
              rb_tail = (rb_tail + 1) & RB_MASK;
              chunk_remaining--;
              active_lbuf[active_lptr++] = b;
              made_progress = true;

              if (active_lptr >= 2 && active_lbuf[active_lptr - 2] == 0xFF &&
                  active_lbuf[active_lptr - 1] == 0xD9) {
                // EOI発見
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

                active_lbuf = nullptr;
                rb_parsed = rb_tail;
                state = FIND_SOI;
                if (chunk_remaining == 0) {
                  chunk_state = CHUNK_TRAILER;
                  seen_cr = false;
                }
                break;
              }
            }

            if (chunk_remaining == 0 && chunk_state == CHUNK_DATA) {
              chunk_state = CHUNK_TRAILER;
              seen_cr = false;
            }

            if (active_lptr >= LINEAR_BUF_SIZE) {
              Serial.println("Fetcher: Linear Overflow!");
              xQueueSend(linearFreeQueue, &active_lbuf, 0);
              active_lbuf = nullptr;
              state = FIND_SOI;
            }
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

// Decode Task: JPEG decode only
void AppLogic::mjpegDecodeTask(void *pvParameters) {
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

      // Decode JPEG to decode_bufs
      int current_idx = decode_idx;
      jpeg_decoder_process(decoder, &dec_cfg, fd.buf, process_len,
                           (uint8_t *)decode_bufs[current_idx],
                           DECODE_BUF_SIZE, &out_size);

      size_t aligned_size = (out_size + 63) & ~63;
      esp_cache_msync(decode_bufs[current_idx], aligned_size,
                      ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                      ESP_CACHE_MSYNC_FLAG_INVALIDATE);

      // Switch decode buffer for next frame
      decode_idx ^= 1;

      // Send decoded frame to render task
      DecodedFrameData dfd;
      dfd.buf_idx = current_idx;
      dfd.linear_buf = fd.is_linear ? fd.buf : nullptr;
      dfd.has_linear_buf = fd.is_linear;
      xQueueSend(decodedFrameQueue, &dfd, portMAX_DELAY);
    } else {
      vTaskDelay(1);
    }
  }
}

// Render Task: PPA + DSI only
void AppLogic::mjpegRenderTask(void *pvParameters) {
  uint32_t frame_count = 0;
  uint32_t last_fps_time = millis();
  DecodedFrameData dfd;

  while (1) {
    if (xQueueReceive(decodedFrameQueue, &dfd, pdMS_TO_TICKS(1000)) == pdTRUE) {
      // Return linear buffer to fetch task
      if (dfd.has_linear_buf) {
        xQueueSend(linearFreeQueue, &dfd.linear_buf, 0);
      }

      if (xSemaphoreTake(displayDoneSema, pdMS_TO_TICKS(500)) == pdTRUE) {
        // Calculate scaling to fit screen (compile-time constants)
        constexpr float scale_x = (float)PANEL_WIDTH / STREAM_WIDTH;
        constexpr float scale_y = (float)PANEL_HEIGHT / STREAM_HEIGHT;
        constexpr float base_scale = scale_x < scale_y ? scale_x : scale_y;

        // Check if rotation is needed (compile-time)
        constexpr bool panel_is_portrait = PANEL_HEIGHT > PANEL_WIDTH;
        constexpr bool stream_is_landscape = STREAM_WIDTH > STREAM_HEIGHT;

        ppa_srm_rotation_angle_t rotation;
        float scale;

        if constexpr (panel_is_portrait && stream_is_landscape) {
          rotation = PPA_SRM_ROTATION_ANGLE_90;
          // After rotation, stream becomes portrait (480x640)
          constexpr float rot_scale_x = (float)PANEL_WIDTH / STREAM_HEIGHT;
          constexpr float rot_scale_y = (float)PANEL_HEIGHT / STREAM_WIDTH;
          scale = rot_scale_x < rot_scale_y ? rot_scale_x : rot_scale_y;
        } else {
          rotation = PPA_SRM_ROTATION_ANGLE_0;
          scale = base_scale;
        }

        // Clamp scale to PPA limitations
        if (scale > 4.0f) scale = 4.0f;
        if (scale < 0.125f) scale = 0.125f;

        // PPA transforms directly to device framebuffer
        bool ppa_ok = PPAPipeline::transform(
            (const uint8_t *)decode_bufs[dfd.buf_idx],
            (uint8_t *)fb,
            STREAM_WIDTH, STREAM_HEIGHT,
            PANEL_WIDTH, PANEL_HEIGHT,
            PPA_SRM_COLOR_MODE_RGB565,
            PPA_SRM_COLOR_MODE_RGB565,
            rotation,
            scale, scale,
            ppaDoneSema
        );

        if (ppa_ok) {
          xSemaphoreTake(ppaDoneSema, portMAX_DELAY);
          esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, PANEL_WIDTH, PANEL_HEIGHT, fb);
        } else {
          xSemaphoreGive(displayDoneSema);
        }

        frame_count++;
        if (frame_count >= 100) {
          uint32_t now = millis();
          Serial.printf("Render FPS: %.1f\n",
                        100.0f * 1000.0f / (now - last_fps_time));
          last_fps_time = now;
          frame_count = 0;
        }
      } else {
        xSemaphoreGive(displayDoneSema);
      }
    } else {
      vTaskDelay(1);
    }
  }
}
