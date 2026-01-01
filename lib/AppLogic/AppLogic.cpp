#include "AppLogic.h"
#include <HTTPClient.h>
#include <WiFi.h>
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
SemaphoreHandle_t AppLogic::transferDoneSema = nullptr;
uint16_t *AppLogic::decode_bufs[2] = {nullptr, nullptr};
int AppLogic::decode_idx = 0;
uint16_t *AppLogic::fb = nullptr;
uint8_t *AppLogic::ring_buf = nullptr;
uint8_t *AppLogic::linear_bufs[2] = {nullptr, nullptr};

void AppLogic::begin() {
  auto cfg = M5.config();
  M5.begin(cfg);

  // M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);
  panel_h = M5.Display.height();
  panel_w = M5.Display.width();

  M5.Display.println("MJPEG Profiling Mode...");

  linearFreeQueue = xQueueCreate(2, sizeof(uint8_t *));
  frameQueue = xQueueCreate(2, sizeof(FrameData));
  rbReleaseQueue = xQueueCreate(4, sizeof(size_t));
  renderReadySema = xSemaphoreCreateBinary();
  xSemaphoreGive(renderReadySema);

  transferDoneSema = xSemaphoreCreateBinary();
  xSemaphoreGive(transferDoneSema); // Initially available so first frame doesn't block

  PPAPipeline::begin();

  ring_buf = (uint8_t *)heap_caps_aligned_alloc(
      64, RING_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  for (int i = 0; i < 2; i++) {
    linear_bufs[i] = (uint8_t *)heap_caps_aligned_alloc(
        64, LINEAR_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *ptr = linear_bufs[i];
    xQueueSend(linearFreeQueue, &ptr, 0);
  }

  auto dsi = static_cast<lgfx::Panel_DSI *>(M5.Display.getPanel());
  auto detail = dsi->config_detail();
  fb = (uint16_t *)detail.buffer;

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
  uint32_t rb_head = 0;   // Writer index
  uint32_t rb_tail = 0;   // Oldest byte still in use
  uint32_t rb_parsed = 0; // Next byte to parse
  bool pendingFrame = false;
  FrameData fdPending;

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(1000);
      continue;
    }

    http.begin(MJPEG_URL);
    if (http.GET() == 200) {
      WiFiClient *stream = http.getStreamPtr();
      stream->setTimeout(100);
      Serial.println("MJPEG Stream Connected (Pipelined)");
      rb_head = rb_tail = rb_parsed = 0;
      pendingFrame = false;

      while (stream->connected()) {
        // 1. Reclaim space
        size_t relLength;
        while (xQueueReceive(rbReleaseQueue, &relLength, 0) == pdTRUE) {
          rb_tail = (rb_tail + relLength) % RING_BUF_SIZE;
        }

        // 2. Read from stream into RB (non-blocking chunk)
        int avail = stream->available();
        if (avail > 0) {
          uint32_t rb_fill =
              (rb_head - rb_tail + RING_BUF_SIZE) % RING_BUF_SIZE;
          uint32_t space = RING_BUF_SIZE - rb_fill - 1;
          if (space > 0) {
            uint32_t to_read = min((uint32_t)avail, (uint32_t)4096);
            if (to_read > space)
              to_read = space;
            uint32_t first_part = min(to_read, RING_BUF_SIZE - rb_head);
            int r1 = stream->read(&ring_buf[rb_head], first_part);
            if (r1 > 0) {
              rb_head = (rb_head + r1) % RING_BUF_SIZE;
              if (r1 < (int)to_read) {
                int r2 = stream->read(&ring_buf[rb_head], to_read - r1);
                if (r2 > 0)
                  rb_head = (rb_head + r2) % RING_BUF_SIZE;
              }
            }
          }
        }

        // 3. Parse Frame (if none pending)
        if (!pendingFrame) {
          uint32_t bytes_to_parse =
              (rb_head - rb_parsed + RING_BUF_SIZE) % RING_BUF_SIZE;
          if (bytes_to_parse >= 2) {
            uint32_t junk = 0;
            bool found_soi = false;
            while (bytes_to_parse >= 2) {
              if (ring_buf[rb_parsed] == 0xFF &&
                  ring_buf[(rb_parsed + 1) % RING_BUF_SIZE] == 0xD8) {
                found_soi = true;
                break;
              }
              rb_parsed = (rb_parsed + 1) % RING_BUF_SIZE;
              bytes_to_parse--;
              junk++;
            }
            if (junk > 0)
              xQueueSend(rbReleaseQueue, &junk, 0);

            if (found_soi) {
              uint32_t search = (rb_parsed + 2) % RING_BUF_SIZE;
              uint32_t len = 2;
              bool found_eoi = false;
              while (len + 1 <= bytes_to_parse) {
                if (ring_buf[search] == 0xFF &&
                    ring_buf[(search + 1) % RING_BUF_SIZE] == 0xD9) {
                  len += 2;
                  found_eoi = true;
                  break;
                }
                search = (search + 1) % RING_BUF_SIZE;
                len++;
              }

              if (found_eoi) {
                fdPending.len = len;
                bool is_contiguous = (rb_parsed + len <= RING_BUF_SIZE);
                bool is_aligned = (((uintptr_t)&ring_buf[rb_parsed]) % 64 == 0);

                if (is_contiguous && is_aligned) {
                  fdPending.buf = &ring_buf[rb_parsed];
                  fdPending.is_linear = false;
                  esp_cache_msync(fdPending.buf, fdPending.len,
                                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
                  pendingFrame = true;
                  rb_parsed = (rb_parsed + len) % RING_BUF_SIZE;
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
                    fdPending.buf = lbuf;
                    fdPending.is_linear = true;
                    esp_cache_msync(fdPending.buf, fdPending.len,
                                    ESP_CACHE_MSYNC_FLAG_DIR_C2M);
                    pendingFrame = true;
                    rb_parsed = (rb_parsed + len) % RING_BUF_SIZE;
                    xQueueSend(rbReleaseQueue, &len, 0);
                  }
                }
              }
            }
          }
        }

        // 4. Dispatch Frame (if renderer ready)
        if (pendingFrame) {
          if (xSemaphoreTake(renderReadySema, 0) == pdTRUE) {
            if (xQueueSend(frameQueue, &fdPending, 0) == pdTRUE) {
              pendingFrame = false;
            } else {
              xSemaphoreGive(renderReadySema);
            }
          }
        }
        vTaskDelay(1);
      }
    }
    http.end();
    vTaskDelay(1000);
  }
#endif
}

void AppLogic::mjpegRenderTask(void *pvParameters) {
  jpeg_decoder_handle_t decoder;
  jpeg_decode_engine_cfg_t eng_cfg = {.intr_priority = 0, .timeout_ms = 40};
  if (jpeg_new_decoder_engine(&eng_cfg, &decoder) != ESP_OK) {
    Serial.println("Render: Failed to init HW decoder");
  }

  jpeg_decode_cfg_t dec_cfg = {
      .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
      .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
      .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
  };

  FrameData fd;

  while (1) {
    if (xQueueReceive(frameQueue, &fd, portMAX_DELAY) == pdTRUE) {
      uint32_t out_size;

      // 1. Decode to the current decode buffer
      //    (This happens while PPA might be copying the PREVIOUS buffer)
      if (jpeg_decoder_process(
              decoder, &dec_cfg, fd.buf, fd.len, (uint8_t *)decode_bufs[decode_idx],
              STREAM_WIDTH * STREAM_HEIGHT * 2, &out_size) == ESP_OK) {

        // 2. Release Input Buffer Immediately
        if (fd.is_linear) {
          xQueueSend(linearFreeQueue, &fd.buf, 0);
        } else {
          xQueueSend(rbReleaseQueue, &fd.len, 0);
        }

        // 3. Wait for Previous PPA to finish
        //    (We cannot start a new PPA transaction until the previous one is done,
        //     and we also implicitly wait for the PREVIOUS decode buffer to be free
        //     since PPA uses it).
        //    Actually, PPA copies From decode_bufs[prev] To fb.
        //    We just wrote to decode_bufs[current].
        //    If prev == current (single buffer), we had to wait BEFORE decode.
        //    With double buffer, we decode to current, while PPA reads prev.
        //    So we must ensure PPA is done with prev before we overwrite it?
        //    No, we write to `current`. `prev` is being read. They are different buffers.
        //    We must ensure PPA is done before we START the next PPA transaction
        //    (if PPA queue depth is 1 or we want to strictly serialize FB updates).

        xSemaphoreTake(transferDoneSema, portMAX_DELAY);

        // 4. Start PPA Copy (Current Decode Buf -> FB)
        //    (Removed cache sync to test performance/alignment)
        bool copied = PPAPipeline::copy(
            (const uint8_t*)decode_bufs[decode_idx], (uint8_t*)fb,
            STREAM_WIDTH, STREAM_HEIGHT,
            panel_w, panel_h,
            PPA_SRM_COLOR_MODE_RGB565, PPA_SRM_COLOR_MODE_RGB565,
            transferDoneSema
        );

        if (!copied) {
           // If failed to start, give back semaphore so we don't hang next loop
           xSemaphoreGive(transferDoneSema);
        }

        // 5. Swap Buffers
        decode_idx = (decode_idx + 1) % 2;

      } else {
        // Handle decode fail
        if (fd.is_linear) {
          xQueueSend(linearFreeQueue, &fd.buf, 0);
        } else {
          xQueueSend(rbReleaseQueue, &fd.len, 0);
        }
      }

      // 6. Signal Fetcher
      xSemaphoreGive(renderReadySema);
    }
    taskYIELD();
  }
}
