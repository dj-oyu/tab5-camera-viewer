#include "PipelineContext.h"
#include <Arduino.h>
#include <esp_heap_caps.h>

namespace {
struct LinearAllocResult {
  uint8_t *ptr = nullptr;
  bool internal = false;
};

LinearAllocResult allocLinearBuffer(size_t size, bool prefer_internal) {
  constexpr uint32_t kInternalCaps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  constexpr uint32_t kSpiramCaps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

  LinearAllocResult result;
  if (prefer_internal) {
    size_t free_internal = heap_caps_get_free_size(kInternalCaps);
    if (free_internal > size + INTERNAL_CACHE_GUARD_BYTES) {
      result.ptr = static_cast<uint8_t *>(
          heap_caps_aligned_alloc(64, size, kInternalCaps));
      result.internal = result.ptr != nullptr;
    }
  }

  if (!result.ptr) {
    result.ptr =
        static_cast<uint8_t *>(heap_caps_aligned_alloc(64, size, kSpiramCaps));
    result.internal = false;
  }

  if (!result.ptr) {
    result.ptr =
        static_cast<uint8_t *>(heap_caps_aligned_alloc(64, size, kInternalCaps));
    result.internal = result.ptr != nullptr;
  }

  return result;
}

uint32_t detectionIntervalMsForLevel(uint8_t level) {
  switch (level) {
  case 0:
    return 0;
  case 1:
    return 120;
  default:
    return 300;
  }
}

uint32_t connectionIntervalMsForLevel(uint8_t level) {
  switch (level) {
  case 0:
    return 0;
  case 1:
    return 160;
  default:
    return 400;
  }
}
} // namespace

bool PipelineContext::init() {
  perf_mutex_ = xSemaphoreCreateMutex();
  if (!perf_mutex_) {
    Serial.println("Failed to create perf mutex");
    return false;
  }

  dma2d_gate_ = xSemaphoreCreateCounting(DMA2D_GATE_COUNT, DMA2D_GATE_COUNT);
  if (!dma2d_gate_) {
    Serial.println("Failed to create DMA2D gate semaphore");
    return false;
  }
  Serial.printf("DMA2D gate: count=%u\n", DMA2D_GATE_COUNT);

  linear_free_queue_ = xQueueCreate(LINEAR_BUF_COUNT, sizeof(uint8_t *));
  frame_queue_ = xQueueCreate(LINEAR_BUF_COUNT, sizeof(FrameData));
  decoded_frame_queue_ = xQueueCreate(DECODE_BUF_COUNT, sizeof(DecodedFrameData));
  if (!linear_free_queue_ || !frame_queue_ || !decoded_frame_queue_) {
    Serial.println("Failed to create queues");
    return false;
  }

  for (int i = 0; i < LINEAR_BUF_COUNT; ++i) {
    bool prefer_internal = (i < static_cast<int>(LINEAR_INTERNAL_CACHE_COUNT));
    LinearAllocResult alloc = allocLinearBuffer(
        LINEAR_BUF_SIZE + BITSTREAM_PAD + 64, prefer_internal);
    linear_bufs_[i] = alloc.ptr;
    if (!linear_bufs_[i]) {
      Serial.printf("Failed to allocate linear buffer[%d]\n", i);
      return false;
    }
    Serial.printf("Linear buffer[%d]: %p (%s)\n", i, linear_bufs_[i],
                  alloc.internal ? "INTERNAL" : "SPIRAM");
    xQueueSend(linear_free_queue_, &linear_bufs_[i], 0);
  }

  for (int i = 0; i < DECODE_BUF_COUNT; ++i) {
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

  // render_buf廃止: PPA直接FB書込み (panel_fb_はDisplayInit::initで設定)

  if (!detection_data_.init()) {
    Serial.println("Failed to init DetectionData");
    return false;
  }

  if (!connection_data_.init()) {
    Serial.println("Failed to init ConnectionData");
    return false;
  }

  if (!recording_data_.init()) {
    Serial.println("Failed to init RecordingData");
    return false;
  }

  if (!battery_data_.init()) {
    Serial.println("Failed to init BatteryData");
    return false;
  }

  return true;
}

bool PipelineContext::acquireLinear(uint8_t *&out_buf) {
  return xQueueReceive(linear_free_queue_, &out_buf, 0) == pdTRUE;
}

void PipelineContext::releaseLinear(uint8_t *buf) {
  xQueueSend(linear_free_queue_, &buf, 0);
}

int PipelineContext::nextDecodeIndex() {
  int idx = decode_idx_;
  decode_idx_ ^= 1;
  return idx;
}

void PipelineContext::updateRenderFps(float fps, uint32_t now_ms) {
  if (!perf_mutex_) {
    return;
  }
  if (xSemaphoreTake(perf_mutex_, pdMS_TO_TICKS(2)) != pdTRUE) {
    return;
  }

  side_profile_.render_fps = fps;
  uint8_t prev_level = side_profile_.level;

  if (fps < FPS_THROTTLE_ON) {
    above_fps_since_ms_ = 0;
    if (below_fps_since_ms_ == 0) {
      below_fps_since_ms_ = now_ms;
    } else if (now_ms - below_fps_since_ms_ >= 2000) {
      if (side_profile_.level < 2) {
        side_profile_.level++;
      }
      below_fps_since_ms_ = now_ms;
    }
  } else if (fps >= FPS_THROTTLE_OFF) {
    below_fps_since_ms_ = 0;
    if (above_fps_since_ms_ == 0) {
      above_fps_since_ms_ = now_ms;
    } else if (now_ms - above_fps_since_ms_ >= 5000) {
      side_profile_.level = 0;
      above_fps_since_ms_ = now_ms;
    }
  } else {
    // Hysteresis band: keep current level but reset timers.
    below_fps_since_ms_ = 0;
    above_fps_since_ms_ = 0;
  }

  side_profile_.detection_min_interval_ms =
      detectionIntervalMsForLevel(side_profile_.level);
  side_profile_.connection_min_interval_ms =
      connectionIntervalMsForLevel(side_profile_.level);
  side_profile_.throttled = side_profile_.level > 0;

  if (VERBOSE_PERF_LOG && side_profile_.level != prev_level) {
    Serial.printf(
        "PerfCtrl: level=%u render_fps=%.1f det_int=%lu conn_int=%lu\n",
        side_profile_.level, side_profile_.render_fps,
        static_cast<unsigned long>(side_profile_.detection_min_interval_ms),
        static_cast<unsigned long>(side_profile_.connection_min_interval_ms));
  }

  xSemaphoreGive(perf_mutex_);
}

SideLoadProfile PipelineContext::sideLoadProfile() const {
  SideLoadProfile snapshot;
  if (!perf_mutex_) {
    return snapshot;
  }
  if (xSemaphoreTake(perf_mutex_, pdMS_TO_TICKS(2)) == pdTRUE) {
    snapshot = side_profile_;
    xSemaphoreGive(perf_mutex_);
  }
  return snapshot;
}
