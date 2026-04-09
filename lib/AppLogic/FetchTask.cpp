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
#include <esp_cache.h>
#include <esp_heap_caps.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <lwip/sockets.h>

#ifdef TAILSCALE_AUTH_KEY
#include "TailscaleTask.h"
#endif

namespace
{

  enum class MjpegState
  {
    CHUNK_SIZE,
    MJPEG_HEADER,
    JPEG_BODY,
    CHUNK_TRAILER
  };

  struct MjpegParser
  {
    MjpegState state = MjpegState::CHUNK_SIZE;
    uint32_t chunk_remaining = 0;
    uint32_t content_length = 0;
    uint32_t jpeg_remaining = 0;
    uint32_t trailer_remaining = 0;
    char header_buf[HEADER_BUF_SIZE];
    uint32_t header_idx = 0;
    size_t write_ptr = 0;
    bool frame_overflow = false;

    void reset()
    {
      state = MjpegState::CHUNK_SIZE;
      chunk_remaining = 0;
      content_length = 0;
      jpeg_remaining = 0;
      trailer_remaining = 0;
      header_idx = 0;
      write_ptr = 0;
      frame_overflow = false;
    }

    void resetFrameState()
    {
      content_length = 0;
      jpeg_remaining = 0;
      write_ptr = 0;
      frame_overflow = false;
    }

    void enterChunkTrailer()
    {
      state = MjpegState::CHUNK_TRAILER;
      trailer_remaining = 2;
    }
  };

  struct FetchPerfStats
  {
    uint32_t frames = 0;
    uint32_t read_calls = 0;
    uint64_t read_bytes = 0;
    uint64_t frame_bytes = 0;
    uint64_t parse_us = 0;
    uint64_t sync_us = 0;
    uint64_t read_wait_us = 0;
    uint64_t idle_wait_us = 0;
    uint32_t oversize_drops = 0;
    uint32_t parser_resets = 0;
    uint32_t no_slot_waits = 0;
    uint32_t drain_drops = 0;
  };

  bool isHexDigit(char c)
  {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
           (c >= 'A' && c <= 'F');
  }

  size_t padAndSync(uint8_t *buf, size_t len)
  {
    size_t aligned_len = (len + BITSTREAM_PAD + 63) & ~63;
    memset(buf + len, 0, aligned_len - len);
    esp_cache_msync(reinterpret_cast<void *>(buf), aligned_len,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    return aligned_len;
  }

  void initWiFi()
  {
#ifdef WIFI_SSID
    WiFi.setPins(12, 13, 11, 10, 9, 8, 15);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(100);
      Serial.print(".");
    }
    Serial.printf("\nWiFi Connected (RSSI=%d dBm)\n", WiFi.RSSI());
    if (VERIFY_WIFI_DIAG_LOG) WiFi.printDiag(Serial);
#endif
  }

  void configureFetchSocket(WiFiClient &stream)
  {
    int sockfd = stream.fd();
    int rcvbuf = static_cast<int>(FETCH_TCP_RCVBUF_BYTES);
    stream.setSocketOption(SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    timeval rcv_to = {};
    rcv_to.tv_sec = FETCH_BLOCK_TIMEOUT_MS / 1000UL;
    rcv_to.tv_usec = (FETCH_BLOCK_TIMEOUT_MS % 1000UL) * 1000UL;
    stream.setSocketOption(SOL_SOCKET, SO_RCVTIMEO, &rcv_to, sizeof(rcv_to));
    stream.setNoDelay(true);
    Serial.printf("Fetch socket: fd=%d rcvbuf=%d rcvto=%lums nodelay=on\n",
                  sockfd, rcvbuf, static_cast<unsigned long>(FETCH_BLOCK_TIMEOUT_MS));
  }

  bool isWouldBlockError(int err)
  {
    return err == EWOULDBLOCK || err == EAGAIN;
  }

  [[nodiscard]] bool parseBytes(const uint8_t *data, size_t len,
                                MjpegParser &parser, uint8_t *linear_buf,
                                size_t *consumed_out)
  {
    size_t idx = 0;

    while (idx < len)
    {
      switch (parser.state)
      {
      case MjpegState::CHUNK_SIZE:
      {
        while (idx < len)
        {
          char c = static_cast<char>(data[idx++]);
          if (c == '\r') continue;
          if (c == '\n')
          {
            parser.header_buf[parser.header_idx] = '\0';
            if (parser.header_idx > 0)
            {
              parser.chunk_remaining = strtoul(parser.header_buf, nullptr, 16);
              parser.header_idx = 0;
              if (parser.chunk_remaining == 0)
              {
                if (consumed_out) *consumed_out = idx;
                return false;
              }
              parser.state = (parser.jpeg_remaining > 0) ? MjpegState::JPEG_BODY
                                                         : MjpegState::MJPEG_HEADER;
            }
            break;
          }
          if (isHexDigit(c) && parser.header_idx < 15)
            parser.header_buf[parser.header_idx++] = c;
        }
        break;
      }

      case MjpegState::MJPEG_HEADER:
      {
        while (idx < len && parser.chunk_remaining > 0)
        {
          char c = static_cast<char>(data[idx++]);
          parser.chunk_remaining--;
          if (parser.header_idx < HEADER_BUF_SIZE - 1)
            parser.header_buf[parser.header_idx++] = c;
          else
          {
            memmove(parser.header_buf, parser.header_buf + 1, HEADER_BUF_SIZE - 2);
            parser.header_buf[HEADER_BUF_SIZE - 2] = c;
            parser.header_idx = HEADER_BUF_SIZE - 1;
          }
          if (parser.header_idx >= 4)
          {
            char *end = parser.header_buf + parser.header_idx;
            if (end[-4] == '\r' && end[-3] == '\n' && end[-2] == '\r' && end[-1] == '\n')
            {
              parser.header_buf[parser.header_idx] = '\0';
              char *cl = strstr(parser.header_buf, "Content-Length:");
              if (!cl) cl = strstr(parser.header_buf, "content-length:");
              if (cl) {
                parser.content_length = strtoul(cl + 15, nullptr, 10);
                parser.jpeg_remaining = parser.content_length;
              } else {
                parser.content_length = 0;
                parser.jpeg_remaining = 0;
              }
              parser.header_idx = 0;
              parser.frame_overflow = (linear_buf == nullptr);
              parser.state = MjpegState::JPEG_BODY;
              break;
            }
          }
        }
        if (parser.state == MjpegState::MJPEG_HEADER && parser.chunk_remaining == 0)
          parser.enterChunkTrailer();
        break;
      }

      case MjpegState::JPEG_BODY:
      {
        if (parser.jpeg_remaining == 0 || parser.content_length == 0)
        {
          parser.state = MjpegState::MJPEG_HEADER;
          parser.header_idx = 0;
          break;
        }
        size_t payload = std::min<size_t>(len - idx,
                                          std::min(parser.chunk_remaining, parser.jpeg_remaining));
        if (payload == 0)
        {
          if (parser.chunk_remaining == 0) parser.enterChunkTrailer();
          break;
        }
        if (!parser.frame_overflow)
        {
          size_t writable = LINEAR_BUF_SIZE - parser.write_ptr;
          size_t copy_bytes = std::min(payload, writable);
          if (copy_bytes > 0)
            memcpy(linear_buf + parser.write_ptr, data + idx, copy_bytes);
          parser.write_ptr += copy_bytes;
          if (copy_bytes < payload) parser.frame_overflow = true;
        }
        idx += payload;
        parser.chunk_remaining -= payload;
        parser.jpeg_remaining -= payload;
        if (parser.jpeg_remaining == 0 && parser.content_length > 0)
        {
          if (parser.chunk_remaining > 0) {
            parser.state = MjpegState::MJPEG_HEADER;
            parser.header_idx = 0;
          } else {
            parser.enterChunkTrailer();
          }
          if (consumed_out) *consumed_out = idx;
          return true;
        }
        if (parser.chunk_remaining == 0) parser.enterChunkTrailer();
        break;
      }

      case MjpegState::CHUNK_TRAILER:
      {
        if (parser.trailer_remaining == 0)
        {
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
    if (consumed_out) *consumed_out = idx;
    return false;
  }

  void fetchTask(void *pvParameters)
  {
    auto *ctx = static_cast<PipelineContext *>(pvParameters);
    initWiFi();

#ifdef TAILSCALE_AUTH_KEY
    Serial.println("Fetch: Waiting for Tailscale VPN...");
    EventGroupHandle_t vpn_eg = TailscaleTask::eventGroup();
    if (vpn_eg)
      xEventGroupWaitBits(vpn_eg, VPN_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    Serial.println("Fetch: VPN ready, starting stream");
#endif

#ifdef MJPEG_URL
    HTTPClient http;
    MjpegParser parser;
    static uint8_t *rx_buf = nullptr;
    if (!rx_buf)
    {
      rx_buf = static_cast<uint8_t *>(heap_caps_aligned_alloc(
          64, FETCH_RX_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
      if (!rx_buf)
        rx_buf = static_cast<uint8_t *>(heap_caps_aligned_alloc(
            64, FETCH_RX_BUF_SIZE, MALLOC_CAP_8BIT));
      if (!rx_buf) { Serial.println("FATAL: rx_buf alloc failed"); vTaskDelete(nullptr); return; }
      Serial.printf("Fetch RX buffer: %p (%lu bytes)\n", rx_buf,
                    static_cast<unsigned long>(FETCH_RX_BUF_SIZE));
    }

    while (1)
    {
      if (WiFi.status() != WL_CONNECTED) { vTaskDelay(1000); continue; }

      http.begin(MJPEG_URL);
      http.setReuse(true);
      http.setTimeout(10000);
      int httpCode = http.GET();
      if (httpCode != 200)
      {
        Serial.printf("HTTP GET failed, code=%d\n", httpCode);
        http.end();
        vTaskDelay(1000);
        continue;
      }

      Serial.println("Stream connected");
      WiFiClient &stream = http.getStream();
      configureFetchSocket(stream);
      int stream_fd = stream.fd();
      if (stream_fd < 0) { http.end(); vTaskDelay(1000); continue; }

      uint32_t last_data_time = millis();
      uint32_t last_frame_time = millis();
      uint32_t perf_window_start = millis();
      FetchPerfStats perf;
      parser.reset();
      bool raw_socket_mode = false;
      uint32_t w = ctx->ringWrite().load(std::memory_order_relaxed);

      while (http.connected())
      {
        // Acquire ring slot
        uint32_t r = ctx->ringRead().load(std::memory_order_acquire);
        if (w - r >= RING_DEPTH)
        {
          perf.no_slot_waits++;
          // Wait for RenderPipeline to free a slot (notification via xTaskNotifyGive)
          ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
          continue;
        }

        FrameSlot *slot = ctx->ringSlot(w);
        uint8_t *active_buf = slot->buf;

        // Read from socket
        int bytes_read;
        if (!raw_socket_mode)
        {
          int64_t t0 = esp_timer_get_time();
          bytes_read = stream.read(rx_buf, FETCH_RX_BUF_SIZE);
          uint64_t dt = static_cast<uint64_t>(esp_timer_get_time() - t0);
          if (bytes_read > 0) { perf.read_wait_us += dt; raw_socket_mode = true;
            Serial.println("Fetch: switched to raw socket recv path"); }
          else perf.idle_wait_us += dt;
        }
        else
        {
          int64_t t0 = esp_timer_get_time();
          bytes_read = recv(stream_fd, rx_buf, FETCH_RX_BUF_SIZE, 0);
          uint64_t dt = static_cast<uint64_t>(esp_timer_get_time() - t0);
          if (bytes_read > 0) perf.read_wait_us += dt;
          else {
            perf.idle_wait_us += dt;
            if (bytes_read < 0 && isWouldBlockError(errno)) bytes_read = 0;
          }
        }

        if (bytes_read > 0)
        {
          perf.read_calls++;
          perf.read_bytes += static_cast<uint32_t>(bytes_read);
          last_data_time = millis();

          size_t offset = 0;
          while (offset < static_cast<size_t>(bytes_read))
          {
            size_t consumed = 0;
            int64_t t0 = esp_timer_get_time();
            bool frame_complete = parseBytes(
                rx_buf + offset, static_cast<size_t>(bytes_read) - offset,
                parser, active_buf, &consumed);
            perf.parse_us += static_cast<uint64_t>(esp_timer_get_time() - t0);

            if (consumed == 0) { perf.parser_resets++; parser.reset(); break; }
            offset += consumed;

            if (frame_complete)
            {
              if (parser.frame_overflow)
              {
                perf.oversize_drops++;
              }
              else
              {
                int64_t t1 = esp_timer_get_time();
                slot->len = parser.write_ptr;
                slot->aligned_len = padAndSync(active_buf, parser.write_ptr);
                perf.sync_us += static_cast<uint64_t>(esp_timer_get_time() - t1);
                perf.frame_bytes += parser.write_ptr;

                // Commit to ring
                w++;
                ctx->ringWrite().store(w, std::memory_order_release);
                xSemaphoreGive(ctx->frameReadySema());
              }

              // Invalidate — slot now owned by consumer
              slot = nullptr;
              active_buf = nullptr;

              perf.frames++;
              last_frame_time = millis();
              parser.resetFrameState();

              // Try next slot for continued parsing
              if (offset < static_cast<size_t>(bytes_read))
              {
                r = ctx->ringRead().load(std::memory_order_acquire);
                if (w - r >= RING_DEPTH)
                {
                  perf.no_slot_waits++;
                  perf.drain_drops++;
                  parser.frame_overflow = true;
                }
                else
                {
                  slot = ctx->ringSlot(w);
                  active_buf = slot->buf;
                  parser.write_ptr = 0;
                }
              }
            }
          }
        }
        else if (bytes_read == 0)
        {
          vTaskDelay(1);
        }
        else
        {
          Serial.printf("Fetch: read error (%d)\n", bytes_read);
          break;
        }

        uint32_t now = millis();
        if (now - last_data_time > 5000) { Serial.println("Fetcher: Data Timeout!"); break; }
        if (now - last_frame_time > 10000) { Serial.println("Fetcher: Stall!"); break; }

        if (VERBOSE_PERF_LOG && (perf.frames >= PERF_LOG_WINDOW_FRAMES ||
            (now - perf_window_start) >= PERF_LOG_INTERVAL_MS))
        {
          uint32_t wms = now - perf_window_start;
          if (wms > 0) {
            float fps = perf.frames * 1000.0f / wms;
            Serial.printf("Fetch: fps=%.1f reads=%u bytes=%llu\n",
                          fps, perf.read_calls,
                          static_cast<unsigned long long>(perf.read_bytes));
          }
          perf = {};
          perf_window_start = now;
        }
      }

      http.end();
      vTaskDelay(1000);
    }
#endif
  }

} // namespace

void FetchTask::start(PipelineContext &ctx, UBaseType_t priority, BaseType_t core)
{
  TaskHandle_t handle = nullptr;
  BaseType_t ret = xTaskCreatePinnedToCore(fetchTask, "Fetch", STACK_DEPTH_FETCH,
                          &ctx, priority, &handle, core);
  if (ret != pdPASS || !handle) {
    Serial.println("FATAL: Failed to create Fetch task!");
    return;
  }
  ctx.setFetchTaskHandle(handle);
}
