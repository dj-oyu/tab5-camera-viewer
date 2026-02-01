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

// Content-Length based MJPEG parser state machine
enum class MjpegState {
  CHUNK_SIZE,    // "xxxx\r\n" chunk size line
  MJPEG_HEADER,  // "--frame\r\n...Content-Length:N\r\n\r\n"
  JPEG_BODY,     // JPEG binary data (direct to buffer)
  CHUNK_TRAILER  // "\r\n" chunk terminator
};

struct MjpegParser {
  MjpegState state = MjpegState::CHUNK_SIZE;
  uint32_t chunk_remaining = 0;   // remaining bytes in current chunk
  uint32_t content_length = 0;    // parsed Content-Length
  uint32_t jpeg_remaining = 0;    // remaining JPEG bytes
  char header_buf[HEADER_BUF_SIZE];
  uint8_t header_idx = 0;
  uint32_t write_ptr = 0;

  void reset() {
    state = MjpegState::CHUNK_SIZE;
    chunk_remaining = 0;
    content_length = 0;
    jpeg_remaining = 0;
    header_idx = 0;
    write_ptr = 0;
  }
};

void padAndSync(uint8_t *buf, size_t len) {
  size_t pad_len = (len + BITSTREAM_PAD + 63) & ~63;
  memset(buf + len, 0, pad_len - len);
  esp_cache_msync(reinterpret_cast<void *>(buf), pad_len,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
}

void submitFrame(PipelineContext &ctx, uint8_t *buf, size_t len) {
  FrameData fd;
  fd.buf = buf;
  fd.len = len;
  fd.is_linear = true;
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

// Parse stream and fill buffer, returns true when frame is complete
bool parseStream(Stream &stream, MjpegParser &parser, uint8_t *linear_buf) {
  while (stream.available() > 0) {
    switch (parser.state) {

    case MjpegState::CHUNK_SIZE: {
      while (stream.available() > 0 && parser.header_idx < 16) {
        char c = stream.read();

        if (c == '\r')
          continue;

        if (c == '\n') {
          parser.header_buf[parser.header_idx] = '\0';
          if (parser.header_idx > 0) {
            parser.chunk_remaining = strtoul(parser.header_buf, nullptr, 16);
            parser.header_idx = 0;

            if (parser.chunk_remaining == 0) {
              // terminal chunk
              return false;
            }

            // decide next state
            parser.state = (parser.jpeg_remaining > 0) ? MjpegState::JPEG_BODY
                                                       : MjpegState::MJPEG_HEADER;
          }
          break;
        }

        // accumulate hex digits
        if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
            (c >= 'A' && c <= 'F')) {
          if (parser.header_idx < 15) {
            parser.header_buf[parser.header_idx++] = c;
          }
        }
      }
      break;
    }

    case MjpegState::MJPEG_HEADER: {
      while (stream.available() > 0 && parser.chunk_remaining > 0 &&
             parser.header_idx < HEADER_BUF_SIZE - 1) {
        char c = stream.read();
        parser.header_buf[parser.header_idx++] = c;
        parser.chunk_remaining--;

        // detect "\r\n\r\n" (header end)
        if (parser.header_idx >= 4) {
          char *end = parser.header_buf + parser.header_idx;
          if (end[-4] == '\r' && end[-3] == '\n' && end[-2] == '\r' &&
              end[-1] == '\n') {
            // extract Content-Length
            parser.header_buf[parser.header_idx] = '\0';
            char *cl = strstr(parser.header_buf, "Content-Length:");
            if (cl) {
              parser.content_length = strtoul(cl + 15, nullptr, 10);
              parser.jpeg_remaining = parser.content_length;
            }

            parser.header_idx = 0;
            parser.state = MjpegState::JPEG_BODY;
            break;
          }
        }
      }

      // chunk exhausted but header incomplete
      if (parser.chunk_remaining == 0 &&
          parser.state == MjpegState::MJPEG_HEADER) {
        parser.state = MjpegState::CHUNK_TRAILER;
      }
      break;
    }

    case MjpegState::JPEG_BODY: {
      // calculate bytes to read
      uint32_t to_read =
          std::min({parser.chunk_remaining, parser.jpeg_remaining,
                    static_cast<uint32_t>(stream.available()),
                    LINEAR_BUF_SIZE - parser.write_ptr});

      if (to_read > 0) {
        // zero-copy direct read to linear buffer
        int bytes_read =
            stream.readBytes(linear_buf + parser.write_ptr, to_read);

        if (bytes_read > 0) {
          parser.write_ptr += bytes_read;
          parser.chunk_remaining -= bytes_read;
          parser.jpeg_remaining -= bytes_read;
        }
      }

      // frame complete check
      if (parser.jpeg_remaining == 0 && parser.content_length > 0) {
        // next frame state transition
        if (parser.chunk_remaining > 0) {
          // next MJPEG header in same chunk
          parser.state = MjpegState::MJPEG_HEADER;
          parser.header_idx = 0;
        } else {
          parser.state = MjpegState::CHUNK_TRAILER;
        }
        return true; // frame complete
      }

      // chunk exhausted but JPEG incomplete
      if (parser.chunk_remaining == 0) {
        parser.state = MjpegState::CHUNK_TRAILER;
      }
      break;
    }

    case MjpegState::CHUNK_TRAILER: {
      // skip "\r\n"
      if (stream.available() >= 2) {
        stream.read(); // '\r'
        stream.read(); // '\n'
        parser.state = MjpegState::CHUNK_SIZE;
        parser.header_idx = 0;
      }
      break;
    }
    }
  }
  return false; // frame not complete
}

void fetchTask(void *pvParameters) {
  auto *ctx = static_cast<PipelineContext *>(pvParameters);
  initWiFi();

#ifdef MJPEG_URL
  HTTPClient http;
  MjpegParser parser;

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(1000);
      continue;
    }

    http.begin(MJPEG_URL);
    http.setReuse(true);
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
    uint32_t last_frame_time = millis();
    uint8_t *active_buf = nullptr;
    parser.reset();

    while (http.connected()) {
      // acquire buffer if not held
      if (!active_buf) {
        if (!ctx->acquireLinear(&active_buf)) {
          taskYIELD();
          continue;
        }
        parser.write_ptr = 0;
      }

      bool frame_complete = parseStream(stream, parser, active_buf);

      if (frame_complete) {
        padAndSync(active_buf, parser.write_ptr);
        submitFrame(*ctx, active_buf, parser.write_ptr);
        last_frame_time = millis();

        active_buf = nullptr; // get new buffer next iteration
        parser.content_length = 0;
        parser.jpeg_remaining = 0;
        parser.write_ptr = 0;
      }

      // update data timestamp
      if (stream.available() > 0) {
        last_data_time = millis();
      }

      // timeout detection
      uint32_t now = millis();
      if (now - last_data_time > 5000) {
        Serial.println("Fetcher: Data Timeout!");
        break;
      }
      if (now - last_frame_time > 10000) {
        Serial.println("Fetcher: Stall!");
        break;
      }

      if (stream.available() == 0) {
        taskYIELD();
      } else {
        taskYIELD();
      }
    }

    if (active_buf) {
      ctx->releaseLinear(active_buf);
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
