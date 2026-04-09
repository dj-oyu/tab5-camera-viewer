#pragma once

#include "BatteryData.h"
#include "ConnectionData.h"
#include "DetectionData.h"
#include "RecordingData.h"
#include "PipelineConfig.h"
#include <atomic>
#include <cstdint>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

/// SPSC ring slot: Fetch writes metadata, RenderPipeline reads.
struct FrameSlot
{
  uint8_t *buf = nullptr;
  size_t len = 0;
  size_t aligned_len = 0;
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

  // SPSC ring
  FrameSlot *ringSlot(uint32_t index) { return &frame_slots_[index % RING_DEPTH]; }
  std::atomic<uint32_t> &ringWrite() { return ring_write_; }
  std::atomic<uint32_t> &ringRead() { return ring_read_; }

  // Task handles
  void setFetchTaskHandle(TaskHandle_t h) { fetch_task_ = h; }
  void setRenderPipelineTaskHandle(TaskHandle_t h) { render_pipeline_task_ = h; }
  TaskHandle_t fetchTaskHandle() const { return fetch_task_; }
  TaskHandle_t renderPipelineTaskHandle() const { return render_pipeline_task_; }

  // Signaling (semaphore-based, ABI-safe with precompiled Arduino libs)
  SemaphoreHandle_t frameReadySema() const { return frame_ready_sema_; }

  // Single decode buffer
  uint8_t *decodeBuf() { return reinterpret_cast<uint8_t *>(decode_buf_); }

  // Display
  esp_lcd_panel_handle_t panelHandle() const { return panel_handle_; }
  void setPanelHandle(esp_lcd_panel_handle_t handle) { panel_handle_ = handle; }
  uint16_t *panelFramebuffer() const { return panel_fb_; }
  void setPanelFramebuffer(uint16_t *fb) { panel_fb_ = fb; }

  // Performance
  void updateRenderFps(float fps, uint32_t now_ms);
  [[nodiscard]] SideLoadProfile sideLoadProfile() const;

  // Shared data
  DetectionData &detectionData() { return detection_data_; }
  const DetectionData &detectionData() const { return detection_data_; }
  ConnectionData &connectionData() { return connection_data_; }
  const ConnectionData &connectionData() const { return connection_data_; }
  RecordingData &recordingData() { return recording_data_; }
  const RecordingData &recordingData() const { return recording_data_; }
  BatteryData &batteryData() { return battery_data_; }
  const BatteryData &batteryData() const { return battery_data_; }

private:
  FrameSlot frame_slots_[RING_DEPTH] = {};
  std::atomic<uint32_t> ring_write_{0};
  std::atomic<uint32_t> ring_read_{0};
  TaskHandle_t fetch_task_ = nullptr;
  TaskHandle_t render_pipeline_task_ = nullptr;

  SemaphoreHandle_t frame_ready_sema_ = nullptr;
  uint16_t *decode_buf_ = nullptr;
  uint8_t *linear_bufs_[RING_DEPTH] = {};

  esp_lcd_panel_handle_t panel_handle_ = nullptr;
  uint16_t *panel_fb_ = nullptr;

  SemaphoreHandle_t perf_mutex_ = nullptr;
  SideLoadProfile side_profile_ = {};
  uint32_t below_fps_since_ms_ = 0;
  uint32_t above_fps_since_ms_ = 0;

  DetectionData detection_data_;
  ConnectionData connection_data_;
  RecordingData recording_data_;
  BatteryData battery_data_;
};
