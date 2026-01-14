#include "DecodeTask.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"

namespace {
void decodeTask(void *pvParameters) {
  auto *ctx = static_cast<PipelineContext *>(pvParameters);

  jpeg_decoder_handle_t decoder = nullptr;
  jpeg_decode_engine_cfg_t eng_cfg = {.intr_priority = 0, .timeout_ms = 100};
  jpeg_decode_cfg_t dec_cfg = {
      .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
      .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
      .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
  };

  FrameData fd;

  while (1) {
    if (xQueueReceive(ctx->frameQueue(), &fd, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (!decoder) {
        jpeg_new_decoder_engine(&eng_cfg, &decoder);
      }

      uint32_t out_size = 0;
      size_t process_len = (fd.len + BITSTREAM_PAD + 63) & ~63;

      esp_cache_msync(reinterpret_cast<void *>(fd.buf), process_len,
                      ESP_CACHE_MSYNC_FLAG_INVALIDATE);
      __asm__ __volatile__("fence" ::: "memory");

      int current_idx = ctx->nextDecodeIndex();
      jpeg_decoder_process(decoder, &dec_cfg, fd.buf, process_len,
                           reinterpret_cast<uint8_t *>(
                               ctx->decodeBuffer(current_idx)),
                           DECODE_BUF_SIZE, &out_size);

      size_t aligned_size = (out_size + 63) & ~63;
      esp_cache_msync(ctx->decodeBuffer(current_idx), aligned_size,
                      ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                          ESP_CACHE_MSYNC_FLAG_INVALIDATE);

      DecodedFrameData dfd;
      dfd.buf_idx = current_idx;
      dfd.linear_buf = fd.is_linear ? fd.buf : nullptr;
      dfd.has_linear_buf = fd.is_linear;
      xQueueSend(ctx->decodedFrameQueue(), &dfd, portMAX_DELAY);
    } else {
      vTaskDelay(1);
    }
  }
}
} // namespace

void DecodeTask::start(PipelineContext &ctx, UBaseType_t priority,
                       BaseType_t core) {
  xTaskCreatePinnedToCore(decodeTask, "Decode", STACK_DEPTH, &ctx, priority,
                          nullptr, core);
}
