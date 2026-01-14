#include "PipelineContext.h"
#include <Arduino.h>
#include <esp_heap_caps.h>

bool PipelineContext::init() {
  display_done_sema_ = xSemaphoreCreateBinary();
  if (!display_done_sema_) {
    Serial.println("Failed to create displayDoneSema");
    return false;
  }
  xSemaphoreGive(display_done_sema_);

  ppa_done_sema_ = xSemaphoreCreateBinary();
  if (!ppa_done_sema_) {
    Serial.println("Failed to create ppaDoneSema");
    return false;
  }

  linear_free_queue_ = xQueueCreate(2, sizeof(uint8_t *));
  frame_queue_ = xQueueCreate(2, sizeof(FrameData));
  decoded_frame_queue_ = xQueueCreate(2, sizeof(DecodedFrameData));
  if (!linear_free_queue_ || !frame_queue_ || !decoded_frame_queue_) {
    Serial.println("Failed to create queues");
    return false;
  }

  ring_buf_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(
      64, RING_BUF_SIZE + 256, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!ring_buf_) {
    Serial.println("Failed to allocate ring buffer");
    return false;
  }

  for (int i = 0; i < 2; ++i) {
    linear_bufs_[i] = static_cast<uint8_t *>(heap_caps_aligned_alloc(
        64, LINEAR_BUF_SIZE + BITSTREAM_PAD + 64,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!linear_bufs_[i]) {
      Serial.printf("Failed to allocate linear buffer[%d]\n", i);
      return false;
    }
    xQueueSend(linear_free_queue_, &linear_bufs_[i], 0);
  }

  for (int i = 0; i < 2; ++i) {
    decode_bufs_[i] = static_cast<uint16_t *>(heap_caps_aligned_alloc(
        64, DECODE_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    Serial.printf("Decode buffer[%d]: %p (size=%u)\n", i, decode_bufs_[i],
                  DECODE_BUF_SIZE);
    if (!decode_bufs_[i]) {
      Serial.printf("FATAL: Failed to allocate decode buffer[%d]!\n", i);
      while (1) {
        vTaskDelay(1000);
      }
    }
  }

  return true;
}

int PipelineContext::nextDecodeIndex() {
  int current = decode_idx_;
  decode_idx_ ^= 1;
  return current;
}

bool PipelineContext::acquireLinear(uint8_t **out_buf) {
  return xQueueReceive(linear_free_queue_, out_buf, 0) == pdTRUE;
}

void PipelineContext::releaseLinear(uint8_t *buf) {
  xQueueSend(linear_free_queue_, &buf, 0);
}
