#include "FetchTask.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <algorithm>
#include <cstdint>
#include <cstring>

namespace {
enum class ParseState { FindSOI, ScanChunk, CopyMode };

struct ChunkedParser {
  enum class State { Size, Data, Trailer };
  State state = State::Size;
  uint32_t remaining = 0;
  char size_buf[16] = {};
  uint8_t size_idx = 0;
  bool seen_cr = false;

  bool advance(uint8_t *ring_buf, uint32_t rb_head, uint32_t &rb_tail,
               uint32_t rb_mask) {
    while (((rb_head - rb_tail) & rb_mask) > 0) {
      uint8_t raw = ring_buf[rb_tail & rb_mask];
      switch (state) {
      case State::Size:
        rb_tail = (rb_tail + 1) & rb_mask;
        if (raw == '\r') {
          continue;
        }
        if (raw == '\n') {
          size_buf[size_idx] = '\0';
          if (size_idx > 0) {
            remaining = strtoul(size_buf, nullptr, 16);
            size_idx = 0;
            if (remaining == 0) {
              return false;
            }
            state = State::Data;
            return true;
          }
        } else if ((raw >= '0' && raw <= '9') ||
                   (raw >= 'a' && raw <= 'f') ||
                   (raw >= 'A' && raw <= 'F')) {
          if (size_idx < 15) {
            size_buf[size_idx++] = raw;
          }
        }
        break;
      case State::Data:
        return true;
      case State::Trailer:
        rb_tail = (rb_tail + 1) & rb_mask;
        if (raw == '\r') {
          seen_cr = true;
        } else if (raw == '\n' && seen_cr) {
          state = State::Size;
          seen_cr = false;
          return true;
        }
        break;
      }
    }
    return false;
  }
};

void padAndSync(uint8_t *buf, size_t len) {
  size_t pad_len = (len + BITSTREAM_PAD + 63) & ~63;
  memset(buf + len, 0, pad_len - len);
  esp_cache_msync(reinterpret_cast<void *>(buf), pad_len,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
}

void submitFrame(PipelineContext &ctx, uint8_t *buf, size_t len,
                 bool is_linear) {
  FrameData fd;
  fd.buf = buf;
  fd.len = len;
  fd.is_linear = is_linear;
  xQueueSend(ctx.frameQueue(), &fd, 0);
}

void initWiFi() {
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

void fetchTask(void *pvParameters) {
  auto *ctx = static_cast<PipelineContext *>(pvParameters);
  initWiFi();

#ifdef MJPEG_URL
  HTTPClient http;
  const uint32_t rb_mask = RING_BUF_SIZE - 1;

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

    ParseState state = ParseState::FindSOI;
    uint8_t *active_lbuf = nullptr;
    uint32_t active_lptr = 0;

    ChunkedParser chunked;
    uint32_t frame_start_pos = 0;

    while (http.connected()) {
      bool made_progress = false;

      int avail = stream.available();
      if (avail > 0) {
        uint32_t rb_fill = (rb_head - rb_tail) & rb_mask;
        uint32_t space = RING_BUF_SIZE - rb_fill - 1;
        if (space > 0) {
          uint32_t to_read = std::min(static_cast<uint32_t>(avail), space);
          uint32_t first_part = std::min(to_read, RING_BUF_SIZE - rb_head);
          int r1 = stream.readBytes(&ctx->ringBuffer()[rb_head], first_part);
          if (r1 > 0) {
            rb_head = (rb_head + r1) & rb_mask;
            made_progress = true;
            last_data_time = millis();
            if (r1 < static_cast<int>(to_read)) {
              int r2 =
                  stream.readBytes(&ctx->ringBuffer()[rb_head], to_read - r1);
              if (r2 > 0) {
                rb_head = (rb_head + r2) & rb_mask;
              }
            }
          }
        }
      }

      chunked.advance(ctx->ringBuffer(), rb_head, rb_tail, rb_mask);

      if (chunked.state == ChunkedParser::State::Data &&
          chunked.remaining > 0) {
        uint32_t available =
            std::min(chunked.remaining, (rb_head - rb_tail) & rb_mask);
        if (available == 0) {
          // wait for more data
        } else if (state == ParseState::FindSOI) {
          for (uint32_t i = 0; i < available - 1; ++i) {
            uint32_t pos = (rb_tail + i) & rb_mask;
            if (ctx->ringBuffer()[pos] == 0xFF &&
                ctx->ringBuffer()[(pos + 1) & rb_mask] == 0xD8) {
              frame_start_pos = pos;
              rb_parsed = (pos + 2) & rb_mask;
              rb_tail = rb_parsed;
              chunked.remaining -= (i + 2);
              state = ParseState::ScanChunk;
              made_progress = true;
              break;
            }
          }
          if (state == ParseState::FindSOI) {
            rb_tail = (rb_tail + available) & rb_mask;
            chunked.remaining -= available;
            if (chunked.remaining == 0) {
              chunked.state = ChunkedParser::State::Trailer;
              chunked.seen_cr = false;
            }
          }
        } else if (state == ParseState::ScanChunk) {
          for (uint32_t i = 0; i < available - 1; ++i) {
            uint32_t pos = (rb_tail + i) & rb_mask;
            if (ctx->ringBuffer()[pos] == 0xFF &&
                ctx->ringBuffer()[(pos + 1) & rb_mask] == 0xD9) {
              uint32_t frame_len = ((pos + 2) - frame_start_pos) & rb_mask;

              if (frame_start_pos + frame_len <= RING_BUF_SIZE) {
                size_t pad_len = (frame_len + BITSTREAM_PAD + 63) & ~63;
                if (frame_start_pos + pad_len <= RING_BUF_SIZE) {
                  padAndSync(ctx->ringBuffer() + frame_start_pos, frame_len);
                  submitFrame(*ctx, ctx->ringBuffer() + frame_start_pos,
                              frame_len, false);
                  last_sub_time = millis();

                  rb_tail = (pos + 2) & rb_mask;
                  rb_parsed = rb_tail;
                  chunked.remaining -= (i + 2);
                  state = ParseState::FindSOI;
                  made_progress = true;
                  break;
                }
              }

              state = ParseState::CopyMode;
              break;
            }
          }

          if (state == ParseState::ScanChunk) {
            rb_tail = (rb_tail + available) & rb_mask;
            rb_parsed = rb_tail;
            chunked.remaining -= available;
            if (chunked.remaining == 0) {
              chunked.state = ChunkedParser::State::Trailer;
              chunked.seen_cr = false;
              state = ParseState::CopyMode;
            }
          }
        }

        if (state == ParseState::CopyMode) {
          if (!active_lbuf) {
            if (ctx->acquireLinear(&active_lbuf)) {
              active_lptr = 0;
              uint32_t copy_len = (rb_parsed - frame_start_pos) & rb_mask;
              for (uint32_t i = 0; i < copy_len; ++i) {
                active_lbuf[active_lptr++] =
                    ctx->ringBuffer()[(frame_start_pos + i) & rb_mask];
              }
            }
          }

          if (active_lbuf) {
            while (chunked.state == ChunkedParser::State::Data &&
                   chunked.remaining > 0 &&
                   ((rb_head - rb_tail) & rb_mask) > 0 &&
                   active_lptr < LINEAR_BUF_SIZE) {
              uint8_t b = ctx->ringBuffer()[rb_tail & rb_mask];
              rb_tail = (rb_tail + 1) & rb_mask;
              chunked.remaining--;
              active_lbuf[active_lptr++] = b;
              made_progress = true;

              if (active_lptr >= 2 && active_lbuf[active_lptr - 2] == 0xFF &&
                  active_lbuf[active_lptr - 1] == 0xD9) {
                padAndSync(active_lbuf, active_lptr);
                submitFrame(*ctx, active_lbuf, active_lptr, true);
                last_sub_time = millis();

                active_lbuf = nullptr;
                rb_parsed = rb_tail;
                state = ParseState::FindSOI;
                if (chunked.remaining == 0) {
                  chunked.state = ChunkedParser::State::Trailer;
                  chunked.seen_cr = false;
                }
                break;
              }
            }

            if (chunked.remaining == 0 &&
                chunked.state == ChunkedParser::State::Data) {
              chunked.state = ChunkedParser::State::Trailer;
              chunked.seen_cr = false;
            }

            if (active_lptr >= LINEAR_BUF_SIZE) {
              Serial.println("Fetcher: Linear Overflow!");
              ctx->releaseLinear(active_lbuf);
              active_lbuf = nullptr;
              state = ParseState::FindSOI;
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
    if (active_lbuf) {
      ctx->releaseLinear(active_lbuf);
    }
    http.end();
    vTaskDelay(1000);
  }
#endif
}
} // namespace

void FetchTask::start(PipelineContext &ctx, UBaseType_t priority,
                      BaseType_t core) {
  xTaskCreatePinnedToCore(fetchTask, "Fetch", STACK_DEPTH, &ctx, priority,
                          nullptr, core);
}
