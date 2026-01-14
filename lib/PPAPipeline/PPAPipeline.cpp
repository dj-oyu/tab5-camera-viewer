#include "PPAPipeline.h"
#include <esp_log.h>

static const char *TAG = "PPAPipeline";

ppa_client_handle_t PPAPipeline::client = nullptr;

bool PPAPipeline::ppa_event_cb(ppa_client_handle_t ppa_client,
                               ppa_event_data_t *event_data, void *user_data) {
  SemaphoreHandle_t sem = (SemaphoreHandle_t)user_data;
  if (sem) {
    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(sem, &task_woken);
    return task_woken == pdTRUE;
  }
  return false;
}

bool PPAPipeline::begin() {
  if (client != nullptr) {
    return true; // Already initialized
  }

  ppa_client_config_t ppa_config = {};
  ppa_config.oper_type = PPA_OPERATION_SRM;

  esp_err_t ret = ppa_register_client(&ppa_config, &client);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register PPA client: %s", esp_err_to_name(ret));
    return false;
  }

  ppa_event_callbacks_t cbs = {};
  cbs.on_trans_done = ppa_event_cb;
  ret = ppa_client_register_event_callbacks(client, &cbs);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register PPA callbacks: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(TAG, "PPA Pipeline initialized");
  return true;
}

bool PPAPipeline::transform(const uint8_t *src, uint8_t *dst,
                           uint32_t src_w, uint32_t src_h,
                           uint32_t dst_w, uint32_t dst_h,
                           ppa_srm_color_mode_t src_fmt,
                           ppa_srm_color_mode_t dst_fmt,
                           ppa_srm_rotation_angle_t rotation,
                           float scale_x, float scale_y,
                           SemaphoreHandle_t done_sem) {
  if (!client) {
    ESP_LOGE(TAG, "PPA client not initialized");
    return false;
  }

  ppa_srm_oper_config_t config = {};

  // Input configuration
  config.in.buffer = (const void *)src;
  config.in.pic_w = src_w;
  config.in.pic_h = src_h;
  config.in.block_w = src_w;
  config.in.block_h = src_h;
  config.in.block_offset_x = 0;
  config.in.block_offset_y = 0;
  config.in.srm_cm = src_fmt;

  // Output configuration
  config.out.buffer = (void *)dst;
  config.out.pic_w = dst_w;
  config.out.pic_h = dst_h;

  // Calculate buffer size
  uint32_t bpp = 2; // RGB565
  if (dst_fmt == PPA_SRM_COLOR_MODE_ARGB8888) {
    bpp = 4;
  }
  config.out.buffer_size = dst_w * dst_h * bpp;

  // Calculate scaled dimensions after rotation
  uint32_t scaled_w = (uint32_t)(src_w * scale_x);
  uint32_t scaled_h = (uint32_t)(src_h * scale_y);

  // For 90/270 degree rotation, swap width and height
  if (rotation == PPA_SRM_ROTATION_ANGLE_90 ||
      rotation == PPA_SRM_ROTATION_ANGLE_270) {
    uint32_t temp = scaled_w;
    scaled_w = scaled_h;
    scaled_h = temp;
  }

  // Center the scaled image in the destination
  uint32_t x_off = (dst_w > scaled_w) ? (dst_w - scaled_w) / 2 : 0;
  uint32_t y_off = (dst_h > scaled_h) ? (dst_h - scaled_h) / 2 : 0;

  config.out.block_offset_x = x_off;
  config.out.block_offset_y = y_off;
  config.out.srm_cm = dst_fmt;

  // Scaling and rotation
  config.scale_x = scale_x;
  config.scale_y = scale_y;
  config.rotation_angle = rotation;

  config.mode = PPA_TRANS_MODE_NON_BLOCKING;
  config.user_data = (void *)done_sem;

  esp_err_t ret = ppa_do_scale_rotate_mirror(client, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "PPA SRM failed: %s", esp_err_to_name(ret));
    return false;
  }

  return true;
}

bool PPAPipeline::copy(const uint8_t *src, uint8_t *dst,
                      uint32_t src_w, uint32_t src_h,
                      uint32_t dst_w, uint32_t dst_h,
                      ppa_srm_color_mode_t src_fmt,
                      ppa_srm_color_mode_t dst_fmt,
                      SemaphoreHandle_t done_sem) {
  return transform(src, dst, src_w, src_h, dst_w, dst_h,
                  src_fmt, dst_fmt,
                  PPA_SRM_ROTATION_ANGLE_0, 1.0f, 1.0f,
                  done_sem);
}
