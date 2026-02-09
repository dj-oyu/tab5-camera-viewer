#pragma once

#include "BatteryData.h"
#include "ConnectionData.h"
#include "DetectionData.h"
#include "RecordingData.h"
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
#include <freertos/task.h>

struct FrameData
{
  uint8_t *buf;
  size_t len;
  size_t aligned_len;
  bool is_linear;
};

struct DecodedFrameData
{
  int buf_idx;
  uint8_t *linear_buf;
  bool has_linear_buf;
};

struct SideLoadProfile
{
  float render_fps = 0.0f;
  uint8_t level = 0;
  uint32_t detection_min_interval_ms = 0;
  uint32_t connection_min_interval_ms = 0;
  bool throttled = false;
};

class PipelineContext
{
public:
  [[nodiscard]] bool init();

  QueueHandle_t frameQueue() const { return frame_queue_; }
  QueueHandle_t decodedFrameQueue() const { return decoded_frame_queue_; }
  QueueHandle_t linearFreeQueue() const { return linear_free_queue_; }
  QueueHandle_t renderFreeQueue() const { return render_free_queue_; }

  TaskHandle_t renderTaskHandle() const { return render_task_handle_; }
  void setRenderTaskHandle(TaskHandle_t h) { render_task_handle_ = h; }
  SemaphoreHandle_t displayDoneSema() const { return display_done_sema_; }

  esp_lcd_panel_handle_t panelHandle() const { return panel_handle_; }
  void setPanelHandle(esp_lcd_panel_handle_t handle) { panel_handle_ = handle; }

  uint16_t *decodeBuffer(int idx) { return decode_bufs_[idx]; }
  const uint16_t *decodeBuffer(int idx) const { return decode_bufs_[idx]; }
  uint8_t *decodeBufferBytes(int idx)
  {
    return reinterpret_cast<uint8_t *>(decode_bufs_[idx]);
  }
  const uint8_t *decodeBufferBytes(int idx) const
  {
    return reinterpret_cast<const uint8_t *>(decode_bufs_[idx]);
  }
  int nextDecodeIndex();

  [[nodiscard]] bool acquireLinear(uint8_t *&out_buf);
  void releaseLinear(uint8_t *buf);
  [[nodiscard]] bool acquireRenderBuffer(uint16_t *&out_buf,
                                         TickType_t wait_ticks = 0);
  void releaseRenderBuffer(uint16_t *buf);
  void updateRenderFps(float fps, uint32_t now_ms);
  [[nodiscard]] SideLoadProfile sideLoadProfile() const;

  DetectionData &detectionData() { return detection_data_; }
  const DetectionData &detectionData() const { return detection_data_; }
  ConnectionData &connectionData() { return connection_data_; }
  const ConnectionData &connectionData() const { return connection_data_; }
  RecordingData &recordingData() { return recording_data_; }
  const RecordingData &recordingData() const { return recording_data_; }
  BatteryData &batteryData() { return battery_data_; }
  const BatteryData &batteryData() const { return battery_data_; }

private:
  QueueHandle_t frame_queue_ = nullptr;
  QueueHandle_t decoded_frame_queue_ = nullptr;
  QueueHandle_t linear_free_queue_ = nullptr;
  QueueHandle_t render_free_queue_ = nullptr;
  TaskHandle_t render_task_handle_ = nullptr;
  SemaphoreHandle_t display_done_sema_ = nullptr;
  esp_lcd_panel_handle_t panel_handle_ = nullptr;

  uint16_t *decode_bufs_[2] = {nullptr, nullptr};
  int decode_idx_ = 0;

  uint8_t *linear_bufs_[LINEAR_BUF_COUNT] = {};
  uint16_t *render_bufs_[RENDER_BUF_COUNT] = {};
  SemaphoreHandle_t perf_mutex_ = nullptr;
  SideLoadProfile side_profile_ = {};
  uint32_t below_fps_since_ms_ = 0;
  uint32_t above_fps_since_ms_ = 0;
  DetectionData detection_data_;
  ConnectionData connection_data_;
  RecordingData recording_data_;
  BatteryData battery_data_;
};
