#include "DecodeTask.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <esp_timer.h>

namespace
{
  void decodeTask(void *pvParameters)
  {
    auto *ctx = static_cast<PipelineContext *>(pvParameters);
    auto frame_queue = ctx->frameQueue();
    auto decoded_queue = ctx->decodedFrameQueue();

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
    uint64_t perf_decode_us = 0;
    uint32_t perf_window_start = millis();
    FrameData fd;

    while (1)
    {
      if (xQueueReceive(frame_queue, &fd, pdMS_TO_TICKS(1000)) == pdTRUE)
      {
        // Quick SOI/EOI marker check to avoid 100ms HW decoder timeout on corrupt data
        if (fd.len < 4 || fd.buf[0] != 0xFF || fd.buf[1] != 0xD8 ||
            fd.buf[fd.len - 2] != 0xFF || fd.buf[fd.len - 1] != 0xD9)
        {
          perf_errors++;
          if (fd.is_linear) { ctx->releaseLinear(fd.buf); }
          continue;
        }

        uint32_t out_size = 0;
        size_t process_len = fd.aligned_len;

        int current_idx = ctx->nextDecodeIndex();
        int64_t decode_start = esp_timer_get_time();
        esp_err_t ret = jpeg_decoder_process(
            decoder, &dec_cfg, fd.buf, process_len,
            ctx->decodeBufferBytes(current_idx), DECODE_BUF_SIZE, &out_size);
        perf_decode_us +=
            static_cast<uint64_t>(esp_timer_get_time() - decode_start);

        if (ret != ESP_OK || out_size == 0)
        {
          perf_errors++;
          if (fd.is_linear)
          {
            ctx->releaseLinear(fd.buf);
          }
          continue;
        }

        DecodedFrameData dfd;
        dfd.buf_idx = current_idx;
        dfd.linear_buf = fd.is_linear ? fd.buf : nullptr;
        dfd.has_linear_buf = fd.is_linear;
        xQueueSend(decoded_queue, &dfd, portMAX_DELAY);
        perf_frames++;

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
              "Decode Perf: fps=%.1f decode=%lluus errors=%u queues(in=%u out=%u)\n",
              (1000.0f * perf_frames) / window_ms,
              static_cast<unsigned long long>(perf_decode_us / perf_frames),
              perf_errors, uxQueueMessagesWaiting(frame_queue),
              uxQueueMessagesWaiting(decoded_queue));
          perf_frames = 0;
          perf_errors = 0;
          perf_decode_us = 0;
          perf_window_start = now;
        }
      }
      else
      {
        vTaskDelay(1);
      }
    }
  }
} // namespace

void DecodeTask::start(PipelineContext &ctx, UBaseType_t priority,
                       BaseType_t core)
{
  xTaskCreatePinnedToCore(decodeTask, "Decode", STACK_DEPTH, &ctx, priority,
                          nullptr, core);
}
