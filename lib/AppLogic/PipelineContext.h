#pragma once

#include "PipelineConfig.h"
#include <cstdint>
#include <driver/jpeg_decode.h>
#include <esp_cache.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

struct FrameData {
  uint8_t *buf;
  size_t len;
  bool is_linear;
};

struct DecodedFrameData {
  int buf_idx;
  uint8_t *linear_buf;
  bool has_linear_buf;
};

class PipelineContext {
public:
  bool init();

  QueueHandle_t frameQueue() const { return frame_queue_; }
  QueueHandle_t decodedFrameQueue() const { return decoded_frame_queue_; }
  QueueHandle_t linearFreeQueue() const { return linear_free_queue_; }

  SemaphoreHandle_t displayDoneSema() const { return display_done_sema_; }
  SemaphoreHandle_t ppaDoneSema() const { return ppa_done_sema_; }

  esp_lcd_panel_handle_t panelHandle() const { return panel_handle_; }
  void setPanelHandle(esp_lcd_panel_handle_t handle) { panel_handle_ = handle; }

  uint16_t *framebuffer() const { return fb_; }
  void setFramebuffer(uint16_t *fb) { fb_ = fb; }

  uint16_t *decodeBuffer(int idx) const { return decode_bufs_[idx]; }
  int nextDecodeIndex();

  uint8_t *ringBuffer() const { return ring_buf_; }

  bool acquireLinear(uint8_t **out_buf);
  void releaseLinear(uint8_t *buf);

private:
  QueueHandle_t frame_queue_ = nullptr;
  QueueHandle_t decoded_frame_queue_ = nullptr;
  QueueHandle_t linear_free_queue_ = nullptr;
  SemaphoreHandle_t display_done_sema_ = nullptr;
  SemaphoreHandle_t ppa_done_sema_ = nullptr;
  esp_lcd_panel_handle_t panel_handle_ = nullptr;

  uint16_t *decode_bufs_[2] = {nullptr, nullptr};
  int decode_idx_ = 0;

  uint16_t *fb_ = nullptr;
  uint8_t *ring_buf_ = nullptr;
  uint8_t *linear_bufs_[2] = {nullptr, nullptr};
};
