#include "AppLogic.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include <cstdint>
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>

#define STACK_DEPTH 8192

QueueHandle_t AppLogic::frameQueue = nullptr;
QueueHandle_t AppLogic::freeQueue = nullptr;
uint16_t *AppLogic::decode_buf = nullptr;

size_t find_FF_pos(uint8_t *buf, uint8_t adjacent, size_t len, bool *result) {
  uint32_t test;
  for (size_t i = 0; i < len - 4; i += 4) {
    test = ~(buf[i] | buf[i + 1] << 8 | buf[i + 2] << 16 | buf[i + 3] << 24);
    if ((test - 0x01010101u & ~test & 0x80808080u) == 0)
      continue;
    for (size_t j = 0; j < 4; j++) {
      if(buf[i + j] == 0xFFu && buf[i + j + 1] == adjacent) {
        *result = true;
        return i + j;
      }
    }
  }

  for (size_t i = len / 4 * 4; i < len; i++) {
    if(buf[i] == 0xFFu && buf[i + 1] == adjacent) {
      *result = true;
      return i;
    }
  }
  *result = false;
  return 0;
}

void AppLogic::begin() {
  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.println("MJPEG Profiling Mode...");

  freeQueue = xQueueCreate(3, sizeof(uint8_t *));
  frameQueue = xQueueCreate(2, sizeof(FrameData));
  for (int i = 0; i < 3; i++) {
    uint8_t *buf = (uint8_t *)heap_caps_aligned_alloc(
        64, 512 * 1024, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    xQueueSend(freeQueue, &buf, 0);
  }

  decode_buf =
      (uint16_t *)heap_caps_aligned_alloc(64, STREAM_WIDTH * STREAM_HEIGHT * 2,
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

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

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(1000);
      continue;
    }

    http.begin(MJPEG_URL);
    if (http.GET() == 200) {
      WiFiClient *stream = http.getStreamPtr();
      stream->setTimeout(100);
      Serial.println("MJPEG Stream Connected");

      while (stream->connected()) {
        uint8_t *buf = nullptr;
        if (xQueueReceive(freeQueue, &buf, 0) != pdTRUE) {
          vTaskDelay(1);
          continue;
        }

        size_t f_pos = 0;
        bool soi = false, found = false;

        // 1. Search for SOI (0xFF 0xD8)
        while (stream->connected() && !soi) {
          int avail = stream->available();
          if (avail < 2) {
            vTaskDelay(1);
            continue;
          }

          // Read into temporary location to search
          int r = stream->read(buf, 4096);
          if (r > 0) {
            size_t p = find_FF_pos(buf, 0xD8u, r, &found);
            if (found) {
              memmove(buf, &buf[p], r - p);
              f_pos = r - p;
              soi = true;
              break;
            }
          }
        }

        // 2. Direct Bulk Read until EOI (0xFF 0xD9)
        bool eoi = false;
        uint8_t last_byte = (f_pos > 0) ? buf[f_pos - 1] : 0;
        const size_t MAX_F = 511 * 1024;

        while (stream->connected() && soi && !eoi && f_pos < MAX_F) {
          int avail = stream->available();
          if (avail <= 0) {
            vTaskDelay(1);
            continue;
          }

          size_t to_read = (avail > 4096) ? 4096 : avail;
          if (f_pos + to_read > MAX_F)
            to_read = MAX_F - f_pos;

          int r = stream->read(&buf[f_pos], to_read);
          if (r > 0) {
            // size_t p = find_FF_pos(&buf[f_pos], r, &found);
            // if (found && buf[p + 1] == 0xD9) {
            //   f_pos = r - p;
            //   eoi = true;
            //   break;
            // } 
            // f_pos += r;
            for (int i = 0; i < r; i++) {
              if (last_byte == 0xFF && buf[f_pos + i] == 0xD9) {
                f_pos += (i + 1);
                eoi = true;
                break;
              }
              last_byte = buf[f_pos + i];
            }
            if (!eoi)
              f_pos += r;
          }
        }

        if (eoi) {
          for (int i = 0; i < 512 && f_pos < 512 * 1024; i++)
            buf[f_pos++] = 0;
          esp_cache_msync(buf, 512 * 1024, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
          FrameData fd = {buf, f_pos};
          xQueueSend(frameQueue, &fd, 0);
        } else {
          xQueueSend(freeQueue, &buf, 0);
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
  uint32_t frames = 0, last_fps = millis();

  auto dsi = static_cast<lgfx::Panel_DSI *>(M5.Display.getPanel());
  auto detail = dsi->config_detail();
  void *fb = detail.buffer;
  uint32_t stride = 720;

  while (1) {
    if (xQueueReceive(frameQueue, &fd, portMAX_DELAY) == pdTRUE) {
      uint32_t t1 = millis();
      uint32_t out_size;
      size_t look_ahead_len = fd.len + 512;
      if (look_ahead_len > 512 * 1024)
        look_ahead_len = 512 * 1024;

      if (jpeg_decoder_process(
              decoder, &dec_cfg, fd.buf, look_ahead_len, (uint8_t *)decode_buf,
              STREAM_WIDTH * STREAM_HEIGHT * 2, &out_size) == ESP_OK) {
        uint32_t t2 = millis();

        esp_cache_msync(decode_buf, STREAM_WIDTH * STREAM_HEIGHT * 2,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                            ESP_CACHE_MSYNC_FLAG_INVALIDATE);

        uint16_t *src = (uint16_t *)decode_buf;
        uint16_t *dst = (uint16_t *)fb;
        for (int y = 0; y < (int)STREAM_HEIGHT; y++) {
          memcpy(&dst[y * stride], &src[y * STREAM_WIDTH], STREAM_WIDTH * 2);
        }

        esp_cache_msync(fb, STREAM_HEIGHT * stride * 2,
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                            ESP_CACHE_MSYNC_FLAG_UNALIGNED);
        uint32_t t3 = millis();

        frames++;
        if (frames % 10 == 0) {
          Serial.printf("Decode:%ums Copy:%lums\n", (unsigned int)(t2 - t1),
                        (unsigned int)(t3 - t2));
        }
      }
      xQueueSend(freeQueue, &fd.buf, 0);
    }

    if (millis() - last_fps >= 1000) {
      Serial.printf("FPS: %u\n", (unsigned int)frames);
      frames = 0;
      last_fps = millis();
    }
    taskYIELD();
  }
}
