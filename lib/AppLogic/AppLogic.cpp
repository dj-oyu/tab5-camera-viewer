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

  M5.Display.println("MJPEG Profiling Mode v38 (Unified Logic)...");
  Serial.println("AppLogic v38 starting...");

  displayDoneSema = xSemaphoreCreateBinary();
  xSemaphoreGive(displayDoneSema);

  linearFreeQueue = xQueueCreate(2, sizeof(uint8_t *));
  frameQueue = xQueueCreate(2, sizeof(FrameData));

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
  const uint32_t RB_MASK = RING_BUF_SIZE - 1;

  // Local helper: peek 16-bit big-endian value across RB wrap
  auto rb_peek16 = [&](uint32_t idx) {
    uint16_t b1 = ring_buf[idx & RB_MASK];
    uint16_t b2 = ring_buf[(idx + 1) & RB_MASK];
    return (uint16_t)((b1 << 8) | b2);
  };

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(1000);
      continue;
    }

    http.begin(MJPEG_URL);
    if (http.GET() != 200) {
      http.end();
      vTaskDelay(1000);
      continue;
    }

    WiFiClient *stream = http.getStreamPtr();
    stream->setTimeout(50);

    // Parser State
    uint32_t rb_head = 0;
    uint32_t rb_tail = 0;
    uint32_t rb_parsed = 0;
    uint32_t searched_len = 0;
    uint32_t frame_count_f = 0;
    uint32_t last_fps_time_f = millis();

    // Parser State Machine
    enum ParseState { FIND_SOI, FIND_EOI, WAIT_MARGIN };
    ParseState state = FIND_SOI;
    bool pending_is_next_soi = false;
    uint32_t margin_wait_start = 0;

    while (stream->connected()) {
      bool made_progress = false;

      // 1. Fill Ring Buffer
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
            made_progress = true;
            if (r1 < (int)to_read) {
              int r2 = stream->read(&ring_buf[rb_head], to_read - r1);
              if (r2 > 0)
                rb_head = (rb_head + r2) & RB_MASK;
            }
          }
        }
      }

      // 3. Process State Machine
      uint32_t btp = (rb_head - rb_parsed) & RB_MASK;

      // Emergency Clear...
      if (((rb_head - rb_tail) & RB_MASK) >= RING_BUF_SIZE - 32768) {
        uint32_t force_skip = 16384;
        rb_parsed = (rb_parsed + force_skip) & RB_MASK;
        rb_tail = rb_parsed; // Synchronous skip
        searched_len = 0;
        state = FIND_SOI;
        made_progress = true;
      }

      switch (state) {
      case FIND_SOI:
        if (btp >= 2) {
          if (rb_peek16(rb_parsed) == 0xFFD8) {
            state = FIND_EOI;
            searched_len = 2;
          } else {
            rb_parsed = (rb_parsed + 1) & RB_MASK;
            rb_tail = rb_parsed; // Synchronous skip
          }
          made_progress = true;
        }
        break;

      case FIND_EOI: {
        if (btp < 2) {
          state = FIND_SOI;
          break;
        }
        uint32_t len = searched_len;
        bool found = false;
        bool is_next_soi = false;
        while (len + 1 < btp) {
          uint16_t marker = rb_peek16(rb_parsed + len);
          if (marker == 0xFFD9) {
            len += 2;
            found = true;
            break;
          } else if (marker == 0xFFD8) {
            is_next_soi = true;
            found = true;
            break;
          }
          len++;
        }
        searched_len = len;
        if (found) {
          pending_is_next_soi = is_next_soi;
          state = WAIT_MARGIN;
          margin_wait_start = millis();
          made_progress = true;
        }
        break;
      }

      case WAIT_MARGIN: {
        uint32_t len = searched_len;
        bool is_next_soi = pending_is_next_soi;
        uint32_t needed_margin = 128; // v37: Safer 128B margin
        uint32_t margin = (rb_head - (rb_parsed + len)) & RB_MASK;

        bool timeout = (millis() - margin_wait_start > 50);

        if (margin < needed_margin && !is_next_soi && !timeout) {
          break; // Need more data
        }

        FrameData fd;
        fd.len = len;
        uintptr_t p_addr = (uintptr_t)&ring_buf[rb_parsed];
        // v37: Strictly aligned process_len with padding
        size_t process_len_aligned = (len + needed_margin + 63) & ~63;
        bool can_zc = (rb_parsed + process_len_aligned <= RING_BUF_SIZE) &&
                      (p_addr % 64 == 0) && (!is_next_soi);

        if (can_zc) {
          fd.buf = &ring_buf[rb_parsed];
          fd.is_linear = false;
          // No need to fill padding here as it's the ring buffer
        } else {
          uint8_t *lbuf = nullptr;
          if (xQueueReceive(linearFreeQueue, &lbuf, 0) == pdTRUE) {
            if (rb_parsed + len <= RING_BUF_SIZE) {
              memcpy(lbuf, &ring_buf[rb_parsed], len);
            } else {
              uint32_t f1 = RING_BUF_SIZE - rb_parsed;
              memcpy(lbuf, &ring_buf[rb_parsed], f1);
              memcpy(lbuf + f1, &ring_buf[0], len - f1);
            }
            if (is_next_soi) {
              lbuf[len++] = 0xFF;
              lbuf[len++] = 0xD9;
            }
            // v37: Use 0x00 (NULL) padding for bitstream reader stability
            memset(lbuf + len, 0x00, process_len_aligned - len);
            fd.buf = lbuf;
            fd.len = len;
            fd.is_linear = true;
          } else {
            break;
          }
        }

        if (xQueueSend(frameQueue, &fd, 0) == pdTRUE) {
          rb_tail = rb_parsed; // Space before this frame is now reclaimable
          rb_parsed = (rb_parsed + len) & RB_MASK;
          searched_len = 0;
          state = FIND_SOI;
          made_progress = true;

          frame_count_f++;
          if (frame_count_f >= 100) {
            uint32_t now = millis();
            float fps = 100000.0f / (now - last_fps_time_f);
            Serial.printf("Fetcher FPS: %.1f, Len: %u, ZC: %d\n", fps, len,
                          !fd.is_linear);
            frame_count_f = 0;
            last_fps_time_f = now;
          }
        } else {
          // Drop frame if queue is full
          if (fd.is_linear) {
            xQueueSend(linearFreeQueue, &fd.buf, 0);
          }
          rb_parsed = (rb_parsed + len) & RB_MASK;
          rb_tail = rb_parsed;
          searched_len = 0;
          state = FIND_SOI;
          made_progress = true;
        }
        break;
      }
      default:
        state = FIND_SOI;
        break;
      }

      if (!made_progress) {
        vTaskDelay(1);
      } else {
        taskYIELD();
      }
    }
    http.end();
    vTaskDelay(1000);
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
      size_t process_len =
          (fd.len + 128 + 63) & ~63; // v37: 128B Aligned margin

      // v37: Invalidate output buffer BEFORE HW writes, to ensure NO cache
      // dirty lines.
      esp_cache_msync(decode_bufs[decode_idx], STREAM_WIDTH * STREAM_HEIGHT * 2,
                      ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                          ESP_CACHE_MSYNC_FLAG_INVALIDATE);

      // v37: Ensure input buffer is synced
      esp_cache_msync((void *)fd.buf, process_len,
                      ESP_CACHE_MSYNC_FLAG_DIR_C2M);

      uint32_t t_start = esp_timer_get_time();
      if (jpeg_decoder_process(decoder, &dec_cfg, fd.buf, process_len,
                               (uint8_t *)decode_bufs[decode_idx],
                               STREAM_WIDTH * STREAM_HEIGHT * 2,
                               &out_size) == ESP_OK) {

        uint32_t t_dec = esp_timer_get_time();

        // -------------------------------------------------------------------
        // v37 Parallel Optimization:
        // Wait for Previous Draw exactly when we need the display hardware.
        // This allows jpeg_decoder_process to run in parallel with the LCD.
        // -------------------------------------------------------------------
        if (xSemaphoreTake(displayDoneSema, pdMS_TO_TICKS(500)) != pdTRUE) {
          Serial.println("Renderer: Display Deadlock! Force recovery...");
          xSemaphoreGive(displayDoneSema);
          xSemaphoreTake(displayDoneSema, 0);
        }

        // v18: Pure IDF Rendering
        esp_lcd_panel_draw_bitmap(
            panel_handle, x_offset, y_offset, x_offset + STREAM_WIDTH,
            y_offset + STREAM_HEIGHT, decode_bufs[decode_idx]);

        uint32_t t_draw = esp_timer_get_time();
        frame_count++;
        if (frame_count >= 100) {
          uint32_t now = millis();
          float fps = 100000.0f / (now - last_fps_time);
          Serial.printf("Render FPS: %.1f\n", fps);
          frame_count = 0;
          last_fps_time = now;
        }

        // Release input buffer
        if (fd.is_linear) {
          xQueueSend(linearFreeQueue, &fd.buf, 0);
        }

        if (frame_count == 1) { // Log once every 100 frames
          Serial.printf("Timing: Dec %uus, Draw %uus\n",
                        (uint32_t)(t_dec - t_start),
                        (uint32_t)(t_draw - t_dec));
        }

        // Swap decode buffer
        decode_idx ^= 1;
        taskYIELD();

      } else {
        Serial.printf("Decode Error for len %u, ZC: %d\n", fd.len,
                      !fd.is_linear);
        // v35: Inspect bitstream on error
        Serial.print("  Head: ");
        for (int i = 0; i < 16; i++)
          Serial.printf("%02X ", fd.buf[i]);
        Serial.print("\n  Tail: ");
        for (int i = 0; i < 16; i++)
          Serial.printf("%02X ", fd.buf[fd.len - 16 + i]);
        Serial.println();

        // v19: Release semaphore even on error
        xSemaphoreGive(displayDoneSema);
        consecutive_errors++;
        vTaskDelay(2); // v31: Safety wait
        if (consecutive_errors > 10) {
          Serial.println(
              "Watchdog: Persistent Decode Errors. Resetting Engine...");
          create_engine();
          consecutive_errors = 0;
        }
        // Release buffer even on error
        if (fd.is_linear) {
          xQueueSend(linearFreeQueue, &fd.buf, 0);
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
