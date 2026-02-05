#include "FetchTask.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <errno.h>
#include <esp_heap_caps.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <lwip/sockets.h>

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
  uint8_t trailer_remaining = 0;  // remaining trailer bytes (CRLF)
  char header_buf[HEADER_BUF_SIZE];
  uint8_t header_idx = 0;
  uint32_t write_ptr = 0;
  bool frame_overflow = false;

  void reset() {
    state = MjpegState::CHUNK_SIZE;
    chunk_remaining = 0;
    content_length = 0;
    jpeg_remaining = 0;
    trailer_remaining = 0;
    header_idx = 0;
    write_ptr = 0;
    frame_overflow = false;
  }

  void resetFrameState() {
    content_length = 0;
    jpeg_remaining = 0;
    write_ptr = 0;
    frame_overflow = false;
  }

  void enterChunkTrailer() {
    state = MjpegState::CHUNK_TRAILER;
    trailer_remaining = 2;
    header_idx = 0;
  }
};

struct FetchPerfStats {
  uint32_t frames = 0;
  uint32_t queue_drops = 0;
  uint32_t parser_resets = 0;
  uint32_t oversize_drops = 0;
  uint32_t no_linear_waits = 0;
  uint32_t coalesce_reads = 0;
  uint32_t bootstrap_reads = 0;
  uint32_t raw_reads = 0;
  uint64_t parse_us = 0;
  uint64_t sync_us = 0;
  uint64_t read_wait_us = 0;
  uint64_t idle_wait_us = 0;
  uint64_t coalesce_wait_us = 0;
  uint64_t read_bytes = 0;
  uint64_t frame_bytes = 0;
  uint64_t bytes_per_read_acc = 0;
  uint32_t read_calls = 0;
};

bool isHexDigit(char c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
         (c >= 'A' && c <= 'F');
}

void padAndSync(uint8_t *buf, size_t len) {
  size_t pad_len = (len + BITSTREAM_PAD + 63) & ~63;
  memset(buf + len, 0, pad_len - len);
  esp_cache_msync(reinterpret_cast<void *>(buf), pad_len,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
}

bool submitFrame(PipelineContext &ctx, uint8_t *buf, size_t len) {
  FrameData fd;
  fd.buf = buf;
  fd.len = len;
  fd.is_linear = true;
  return xQueueSend(ctx.frameQueue(), &fd, pdMS_TO_TICKS(1)) == pdTRUE;
}

void initWiFi() {
#ifdef WIFI_SSID
  WiFi.setPins(12, 13, 11, 10, 9, 8, 15);
  WiFi.setSleep(false); // keep radio active for stable throughput
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.printf("\nWiFi Connected (RSSI=%d dBm)\n", WiFi.RSSI());
#endif
}

void configureFetchSocket(WiFiClient &stream) {
  int rcvbuf = static_cast<int>(FETCH_TCP_RCVBUF_BYTES);
  int rcvbuf_res =
      stream.setSocketOption(SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
  int nodelay_res = stream.setNoDelay(true);
  Serial.printf("Fetch socket: rcvbuf=%lu(%s) nodelay=%s\n",
                static_cast<unsigned long>(FETCH_TCP_RCVBUF_BYTES),
                (rcvbuf_res == 0) ? "ok" : "fail",
                (nodelay_res == 0) ? "on" : "fail");
}

bool isWouldBlockError(int err) {
  return err == EWOULDBLOCK || err == EAGAIN;
}

int waitReadable(int sockfd, uint32_t wait_us, uint64_t *wait_time_us) {
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(sockfd, &rfds);
  timeval tv = {};
  tv.tv_sec = wait_us / 1000000UL;
  tv.tv_usec = wait_us % 1000000UL;
  int64_t wait_start = esp_timer_get_time();
  int ready = select(sockfd + 1, &rfds, nullptr, nullptr, &tv);
  if (wait_time_us) {
    *wait_time_us = static_cast<uint64_t>(esp_timer_get_time() - wait_start);
  }
  return ready;
}

int readBootstrapStream(WiFiClient &stream, uint8_t *buf, size_t cap,
                        FetchPerfStats &perf) {
  int64_t read_start = esp_timer_get_time();
  int bytes_read = stream.read(buf, cap);
  uint64_t read_us = static_cast<uint64_t>(esp_timer_get_time() - read_start);
  if (bytes_read > 0) {
    perf.read_wait_us += read_us;
    perf.bootstrap_reads++;
  } else {
    perf.idle_wait_us += read_us;
  }
  return bytes_read;
}

int readRawSocketCoalesced(int sockfd, uint8_t *buf, size_t cap,
                           FetchPerfStats &perf) {
  uint64_t select_wait_us = 0;
  int ready = waitReadable(sockfd, FETCH_COALESCE_WAIT_US, &select_wait_us);
  if (ready <= 0) {
    perf.idle_wait_us += select_wait_us;
    return 0;
  }

  int64_t first_read_start = esp_timer_get_time();
  int first = recv(sockfd, buf, cap, MSG_DONTWAIT);
  uint64_t first_read_us =
      static_cast<uint64_t>(esp_timer_get_time() - first_read_start);
  perf.read_wait_us += select_wait_us + first_read_us;

  if (first <= 0) {
    if (first < 0 && isWouldBlockError(errno)) {
      perf.idle_wait_us += select_wait_us;
      return 0;
    }
    return first;
  }

  perf.raw_reads++;
  size_t total = static_cast<size_t>(first);
  if (total >= FETCH_COALESCE_MIN_BYTES) {
    return static_cast<int>(total);
  }

  int64_t coalesce_start = esp_timer_get_time();
  while (total < cap) {
    int64_t elapsed = esp_timer_get_time() - coalesce_start;
    if (elapsed >= static_cast<int64_t>(FETCH_COALESCE_WAIT_US)) {
      break;
    }

    int n = recv(sockfd, buf + total, cap - total, MSG_DONTWAIT);
    if (n > 0) {
      total += static_cast<size_t>(n);
      perf.coalesce_reads++;
      continue;
    }
    if (n == 0) {
      break;
    }

    if (!isWouldBlockError(errno)) {
      break;
    }
    esp_rom_delay_us(FETCH_COALESCE_POLL_US);
  }

  perf.coalesce_wait_us +=
      static_cast<uint64_t>(esp_timer_get_time() - coalesce_start);
  return static_cast<int>(total);
}

int readCoalesced(WiFiClient &stream, int sockfd, uint8_t *buf, size_t cap,
                  bool &raw_mode, FetchPerfStats &perf) {
  if (!raw_mode) {
    int bootstrap = readBootstrapStream(stream, buf, cap, perf);
    if (bootstrap > 0) {
      raw_mode = true;
      Serial.println("Fetch: switched to raw socket recv path");
    }
    return bootstrap;
  }
  return readRawSocketCoalesced(sockfd, buf, cap, perf);
}

// Parse byte stream and fill buffer.
// consumed_out is always set to consumed byte count from [data, len).
// Returns true when one JPEG frame is complete.
bool parseBytes(const uint8_t *data, size_t len, MjpegParser &parser,
                uint8_t *linear_buf, size_t *consumed_out) {
  size_t idx = 0;

  while (idx < len) {
    switch (parser.state) {

    case MjpegState::CHUNK_SIZE: {
      while (idx < len) {
        char c = static_cast<char>(data[idx++]);

        if (c == '\r') {
          continue;
        }

        if (c == '\n') {
          parser.header_buf[parser.header_idx] = '\0';
          if (parser.header_idx > 0) {
            parser.chunk_remaining = strtoul(parser.header_buf, nullptr, 16);
            parser.header_idx = 0;

            if (parser.chunk_remaining == 0) {
              // terminal chunk
              if (consumed_out) {
                *consumed_out = idx;
              }
              return false;
            }

            parser.state = (parser.jpeg_remaining > 0) ? MjpegState::JPEG_BODY
                                                        : MjpegState::MJPEG_HEADER;
          }
          break;
        }

        if (isHexDigit(c) && parser.header_idx < 15) {
          parser.header_buf[parser.header_idx++] = c;
        }
      }
      break;
    }

    case MjpegState::MJPEG_HEADER: {
      while (idx < len && parser.chunk_remaining > 0) {
        char c = static_cast<char>(data[idx++]);
        parser.chunk_remaining--;

        if (parser.header_idx < HEADER_BUF_SIZE - 1) {
          parser.header_buf[parser.header_idx++] = c;
        } else {
          // Keep a sliding window to avoid hard-failing on longer headers.
          memmove(parser.header_buf, parser.header_buf + 1, HEADER_BUF_SIZE - 2);
          parser.header_buf[HEADER_BUF_SIZE - 2] = c;
          parser.header_idx = HEADER_BUF_SIZE - 1;
        }

        if (parser.header_idx >= 4) {
          char *end = parser.header_buf + parser.header_idx;
          if (end[-4] == '\r' && end[-3] == '\n' && end[-2] == '\r' &&
              end[-1] == '\n') {
            parser.header_buf[parser.header_idx] = '\0';

            char *cl = strstr(parser.header_buf, "Content-Length:");
            if (!cl) {
              cl = strstr(parser.header_buf, "content-length:");
            }
            if (cl) {
              parser.content_length = strtoul(cl + 15, nullptr, 10);
              parser.jpeg_remaining = parser.content_length;
            } else {
              parser.content_length = 0;
              parser.jpeg_remaining = 0;
            }

            parser.header_idx = 0;
            parser.frame_overflow = false;
            parser.state = MjpegState::JPEG_BODY;
            break;
          }
        }
      }

      if (parser.state == MjpegState::MJPEG_HEADER && parser.chunk_remaining == 0) {
        parser.enterChunkTrailer();
      }
      break;
    }

    case MjpegState::JPEG_BODY: {
      if (parser.jpeg_remaining == 0 || parser.content_length == 0) {
        parser.state = MjpegState::MJPEG_HEADER;
        parser.header_idx = 0;
        break;
      }

      size_t payload = std::min<size_t>(len - idx,
                                        std::min(parser.chunk_remaining,
                                                 parser.jpeg_remaining));
      if (payload == 0) {
        if (parser.chunk_remaining == 0) {
          parser.enterChunkTrailer();
        }
        break;
      }

      size_t writable = LINEAR_BUF_SIZE - parser.write_ptr;
      size_t copy_bytes = 0;
      if (!parser.frame_overflow) {
        copy_bytes = std::min(payload, writable);
        if (copy_bytes > 0) {
          memcpy(linear_buf + parser.write_ptr, data + idx, copy_bytes);
          parser.write_ptr += copy_bytes;
        }
        if (copy_bytes < payload) {
          parser.frame_overflow = true;
        }
      }

      idx += payload;
      parser.chunk_remaining -= payload;
      parser.jpeg_remaining -= payload;

      if (parser.jpeg_remaining == 0 && parser.content_length > 0) {
        if (parser.chunk_remaining > 0) {
          parser.state = MjpegState::MJPEG_HEADER;
          parser.header_idx = 0;
        } else {
          parser.enterChunkTrailer();
        }

        if (consumed_out) {
          *consumed_out = idx;
        }
        return true;
      }

      if (parser.chunk_remaining == 0) {
        parser.enterChunkTrailer();
      }
      break;
    }

    case MjpegState::CHUNK_TRAILER: {
      if (parser.trailer_remaining == 0) {
        parser.state = MjpegState::CHUNK_SIZE;
        parser.header_idx = 0;
        break;
      }

      size_t skip = std::min<size_t>(len - idx, parser.trailer_remaining);
      idx += skip;
      parser.trailer_remaining -= skip;
      if (parser.trailer_remaining == 0) {
        parser.state = MjpegState::CHUNK_SIZE;
        parser.header_idx = 0;
      }
      break;
    }
    }
  }

  if (consumed_out) {
    *consumed_out = idx;
  }
  return false;
}

void logFetchPerf(const FetchPerfStats &perf, uint32_t window_ms,
                  const PipelineContext &ctx) {
  if (window_ms == 0) {
    return;
  }

  float window_sec = window_ms / 1000.0f;
  float fetch_mbps = (window_sec > 0.0f)
                         ? (perf.read_bytes * 8.0f) / (window_sec * 1000000.0f)
                         : 0.0f;
  float fps = (window_sec > 0.0f) ? (perf.frames / window_sec) : 0.0f;
  uint64_t avg_frame_bytes = (perf.frames > 0) ? (perf.frame_bytes / perf.frames) : 0;
  uint64_t bytes_per_read =
      (perf.read_calls > 0) ? (perf.bytes_per_read_acc / perf.read_calls) : 0;
  uint64_t frame_div = (perf.frames > 0) ? perf.frames : 1;
  uint64_t read_div = (perf.read_calls > 0) ? perf.read_calls : 1;

  Serial.printf("MJPEG FPS: %.1f\n", fps);
  Serial.printf(
      "Fetch Perf: fps=%.1f mbps=%.2f frame=%llub read=%llub parse=%lluus "
      "sync=%lluus rwait=%lluus idle=%lluus cwait=%lluus reads=%u creads=%u "
      "boot=%u raw=%u drops=%u ovf=%u reset=%u no_buf=%u "
      "queues(frame=%u linear=%u)\n",
      fps, fetch_mbps, static_cast<unsigned long long>(avg_frame_bytes),
      static_cast<unsigned long long>(bytes_per_read),
      static_cast<unsigned long long>(perf.parse_us / frame_div),
      static_cast<unsigned long long>(perf.sync_us / frame_div),
      static_cast<unsigned long long>(perf.read_wait_us / read_div),
      static_cast<unsigned long long>(perf.idle_wait_us / read_div),
      static_cast<unsigned long long>(perf.coalesce_wait_us / read_div),
      perf.read_calls, perf.coalesce_reads, perf.bootstrap_reads, perf.raw_reads,
      perf.queue_drops, perf.oversize_drops, perf.parser_resets,
      perf.no_linear_waits,
      uxQueueMessagesWaiting(ctx.frameQueue()),
      uxQueueMessagesWaiting(ctx.linearFreeQueue()));
}

void fetchTask(void *pvParameters) {
  auto *ctx = static_cast<PipelineContext *>(pvParameters);
  initWiFi();

#ifdef MJPEG_URL
  HTTPClient http;
  MjpegParser parser;
  static uint8_t *rx_buf = nullptr;
  static bool rx_buf_internal = false;
  if (!rx_buf) {
    rx_buf = static_cast<uint8_t *>(heap_caps_aligned_alloc(
        64, FETCH_RX_BUF_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    rx_buf_internal = rx_buf != nullptr;
    if (!rx_buf) {
      rx_buf = static_cast<uint8_t *>(heap_caps_aligned_alloc(
          64, FETCH_RX_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    }
    if (!rx_buf) {
      rx_buf = static_cast<uint8_t *>(heap_caps_aligned_alloc(
          64, FETCH_RX_BUF_SIZE, MALLOC_CAP_8BIT));
    }
    if (!rx_buf) {
      Serial.printf("Failed to allocate fetch RX buffer (%lu bytes)\n",
                    static_cast<unsigned long>(FETCH_RX_BUF_SIZE));
      vTaskDelete(nullptr);
      return;
    }
    Serial.printf("Fetch RX buffer: %p (%s, %lu bytes)\n", rx_buf,
                  rx_buf_internal ? "INTERNAL" : "SPIRAM/GENERIC",
                  static_cast<unsigned long>(FETCH_RX_BUF_SIZE));
  }

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
    WiFiClient &stream = http.getStream();
    configureFetchSocket(stream);
    int stream_fd = stream.fd();
    if (stream_fd < 0) {
      Serial.println("Fetch: invalid stream fd");
      http.end();
      vTaskDelay(1000);
      continue;
    }

    uint32_t last_data_time = millis();
    uint32_t last_frame_time = millis();
    uint32_t perf_window_start = millis();
    FetchPerfStats perf;
    uint8_t *active_buf = nullptr;
    bool raw_socket_mode = false;
    parser.reset();

    while (http.connected()) {
      if (!active_buf) {
        if (!ctx->acquireLinear(&active_buf)) {
          perf.no_linear_waits++;
          taskYIELD();
          esp_rom_delay_us(FETCH_IDLE_BACKOFF_US);
          continue;
        }
        parser.write_ptr = 0;
      }

      int bytes_read = readCoalesced(stream, stream_fd, rx_buf, FETCH_RX_BUF_SIZE,
                                     raw_socket_mode, perf);

      if (bytes_read > 0) {
        perf.read_calls++;
        perf.read_bytes += static_cast<uint32_t>(bytes_read);
        perf.bytes_per_read_acc += static_cast<uint32_t>(bytes_read);
        last_data_time = millis();

        size_t offset = 0;
        while (offset < static_cast<size_t>(bytes_read)) {
          size_t consumed = 0;
          int64_t parse_start = esp_timer_get_time();
          bool frame_complete = parseBytes(
              rx_buf + offset, static_cast<size_t>(bytes_read) - offset, parser,
              active_buf, &consumed);
          perf.parse_us +=
              static_cast<uint64_t>(esp_timer_get_time() - parse_start);

          if (consumed == 0) {
            perf.parser_resets++;
            parser.reset();
            break;
          }
          offset += consumed;

          if (frame_complete) {
            if (parser.frame_overflow) {
              perf.oversize_drops++;
              ctx->releaseLinear(active_buf);
            } else {
              int64_t sync_start = esp_timer_get_time();
              padAndSync(active_buf, parser.write_ptr);
              perf.sync_us +=
                  static_cast<uint64_t>(esp_timer_get_time() - sync_start);
              perf.frame_bytes += parser.write_ptr;

              if (!submitFrame(*ctx, active_buf, parser.write_ptr)) {
                perf.queue_drops++;
                ctx->releaseLinear(active_buf);
              }
            }

            perf.frames++;
            last_frame_time = millis();
            active_buf = nullptr;
            parser.resetFrameState();

            if (offset < static_cast<size_t>(bytes_read)) {
              if (!ctx->acquireLinear(&active_buf)) {
                perf.no_linear_waits++;
                perf.parser_resets++;
                parser.reset();
                break;
              }
              parser.write_ptr = 0;
            }
          }
        }
      } else if (bytes_read == 0) {
        taskYIELD();
        esp_rom_delay_us(FETCH_IDLE_BACKOFF_US);
      } else {
        Serial.printf("Fetch: read error (%d)\n", bytes_read);
        break;
      }

      uint32_t now = millis();
      if (now - last_data_time > 5000) {
        Serial.println("Fetcher: Data Timeout!");
        break;
      }
      if (now - last_frame_time > 10000) {
        Serial.println("Fetcher: Stall!");
        break;
      }

      if (perf.frames >= PERF_LOG_WINDOW_FRAMES ||
          (now - perf_window_start) >= PERF_LOG_INTERVAL_MS) {
        uint32_t window_ms = now - perf_window_start;
        logFetchPerf(perf, window_ms, *ctx);
        perf = {};
        perf_window_start = now;
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
