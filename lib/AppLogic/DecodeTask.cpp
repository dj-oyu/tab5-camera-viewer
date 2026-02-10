#include "DecodeTask.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <esp_log.h>
#include <esp_timer.h>

namespace
{
  // Check for JPEG EOI marker (0xFF 0xD9) near end of data.
  // Truncated frames from MJPEG streaming lack EOI and cause
  // "data units mismatch" errors in the HW decoder.
  bool hasEoiMarker(const uint8_t *buf, size_t len)
  {
    if (len < 2) return false;
    // Search last 16 bytes for EOI (may have trailing padding)
    size_t search_start = (len > 16) ? len - 16 : 0;
    for (size_t i = search_start; i < len - 1; i++)
    {
      if (buf[i] == 0xFF && buf[i + 1] == 0xD9) return true;
    }
    return false;
  }

  void decodeTask(void *pvParameters)
  {
    auto *ctx = static_cast<PipelineContext *>(pvParameters);
    auto frame_queue = ctx->frameQueue();
    auto dma2d_gate = ctx->dma2dGate();

    // Suppress noisy ESP-IDF JPEG decoder internal errors
    // (e.g. "data units mismatch" on truncated frames)
    esp_log_level_set("jpeg.decoder", ESP_LOG_WARN);

    jpeg_decoder_handle_t decoder = nullptr;
    jpeg_decode_engine_cfg_t eng_cfg = {.intr_priority = 0, .timeout_ms = 100};
    jpeg_decode_cfg_t dec_cfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };
    if (jpeg_new_decoder_engine(&eng_cfg, &decoder) != ESP_OK || !decoder)
    {
      Serial.println("DecodeTask: Failed to create JPEG decoder");
      vTaskDelete(nullptr);
      return;
    }

    uint32_t perf_frames = 0;
    uint32_t perf_errors = 0;
    uint32_t perf_truncated = 0;
    uint32_t perf_received = 0;
    uint64_t perf_decode_us = 0;
    uint32_t perf_window_start = millis();
    FrameData fd;

    Serial.println("DecodeTask: Started (JPEG only, dma2d_gate)");

    while (1)
    {
      if (xQueueReceive(frame_queue, &fd, pdMS_TO_TICKS(1000)) == pdTRUE)
      {
        perf_received++;
        // Quick SOI marker check to avoid 100ms HW decoder timeout on corrupt data
        if (fd.len < 4 || fd.buf[0] != 0xFF || fd.buf[1] != 0xD8)
        {
          perf_errors++;
          if (fd.is_linear) { ctx->releaseLinear(fd.buf); }
          continue;
        }

        // EOI marker check: skip truncated frames that would cause HW decoder errors
        if (!hasEoiMarker(fd.buf, fd.len))
        {
          perf_truncated++;
          if (fd.is_linear) { ctx->releaseLinear(fd.buf); }
          continue;
        }

        // Acquire decode buffer slot (blocks until RenderTask releases one)
        uint8_t *decode_buf = ctx->acquireDecodeBuf();
        if (!decode_buf)
        {
          if (fd.is_linear) { ctx->releaseLinear(fd.buf); }
          continue;
        }

        uint32_t out_size = 0;
        size_t process_len = fd.aligned_len;

        // DMA2D gate: JPEG HW decoder uses DMA2D channels
        xSemaphoreTake(dma2d_gate, portMAX_DELAY);

        int64_t decode_start = esp_timer_get_time();
        esp_err_t ret = jpeg_decoder_process(
            decoder, &dec_cfg, fd.buf, process_len,
            decode_buf, DECODE_BUF_SIZE, &out_size);
        perf_decode_us +=
            static_cast<uint64_t>(esp_timer_get_time() - decode_start);

        xSemaphoreGive(dma2d_gate);

        // Release linear buffer ASAP (FetchTask can reuse immediately)
        if (fd.is_linear) { ctx->releaseLinear(fd.buf); }

        if (ret != ESP_OK || out_size == 0)
        {
          perf_errors++;
          ctx->discardDecodeBuf();
          if (perf_errors <= 3)
          {
            Serial.printf("Decode: jpeg failed err=%d(%s) out=%u len=%u aligned=%u\n",
                          static_cast<int>(ret), esp_err_to_name(ret),
                          out_size, static_cast<uint32_t>(fd.len),
                          static_cast<uint32_t>(process_len));
          }
        }
        else
        {
          ctx->commitDecodedFrame();
          perf_frames++;
        }

        // Perf logging (runs for both success and error paths)
        uint32_t now = millis();
        if (perf_frames >= PERF_LOG_WINDOW_FRAMES ||
            ((now - perf_window_start) >= PERF_LOG_INTERVAL_MS &&
             perf_frames > 0))
        {
          uint32_t window_ms = now - perf_window_start;
          if (window_ms == 0)
          {
            window_ms = 1;
          }
          Serial.printf(
              "Decode Perf: fps=%.1f decode=%lluus errors=%u truncated=%u "
              "queues(in=%u out=%u)\n",
              (1000.0f * perf_frames) / window_ms,
              static_cast<unsigned long long>(perf_decode_us / perf_frames),
              perf_errors, perf_truncated,
              uxQueueMessagesWaiting(frame_queue),
              ctx->decodedFramesPending());
          perf_frames = 0;
          perf_received = 0;
          perf_errors = 0;
          perf_truncated = 0;
          perf_decode_us = 0;
          perf_window_start = now;
        }
        else if ((now - perf_window_start) >= PERF_LOG_INTERVAL_MS &&
                 perf_frames == 0 && (perf_errors > 0 || perf_truncated > 0))
        {
          Serial.printf(
              "Decode ERRORS: recv=%u errors=%u truncated=%u "
              "queues(in=%u out=%u)\n",
              perf_received, perf_errors, perf_truncated,
              uxQueueMessagesWaiting(frame_queue),
              ctx->decodedFramesPending());
          perf_received = 0;
          perf_errors = 0;
          perf_truncated = 0;
          perf_window_start = now;
        }
        else if ((now - perf_window_start) >= 5000 &&
                 perf_frames == 0 && perf_received == 0)
        {
          Serial.printf(
              "Decode IDLE: queues(in=%u out=%u)\n",
              uxQueueMessagesWaiting(frame_queue),
              ctx->decodedFramesPending());
          perf_window_start = now;
        }
      }
      else
      {
        // xQueueReceive timed out (no frame for 1 second)
        uint32_t now = millis();
        if ((now - perf_window_start) >= 5000)
        {
          Serial.printf(
              "Decode IDLE: recv=%u ok=%u err=%u trunc=%u queues(in=%u out=%u)\n",
              perf_received, perf_frames, perf_errors, perf_truncated,
              uxQueueMessagesWaiting(frame_queue),
              ctx->decodedFramesPending());
          perf_received = 0;
          perf_frames = 0;
          perf_errors = 0;
          perf_truncated = 0;
          perf_decode_us = 0;
          perf_window_start = now;
        }
        vTaskDelay(1);
      }
    }
  }
} // namespace

void DecodeTask::start(PipelineContext &ctx, UBaseType_t priority,
                       BaseType_t core)
{
  xTaskCreatePinnedToCore(decodeTask, "Decode", STACK_DEPTH_DECODE, &ctx, priority,
                          nullptr, core);
}
