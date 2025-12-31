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
  ppa_config.oper_type = PPA_OPERATION_SRM; // We primarily use SRM for copy/scale

  esp_err_t ret = ppa_register_client(&ppa_config, &client);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register PPA client: %s", esp_err_to_name(ret));
    return false;
  }

  // Register callback
  ppa_event_callbacks_t cbs = {};
  cbs.on_trans_done = ppa_event_cb;
  ret = ppa_client_register_event_callbacks(client, &cbs);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register PPA callbacks: %s", esp_err_to_name(ret));
    return false;
  }

  return true;
}

bool PPAPipeline::copy(const uint8_t *src, uint8_t *dst, uint32_t src_w,
                       uint32_t src_h, uint32_t dst_w, uint32_t dst_h,
                       ppa_srm_color_mode_t src_fmt, ppa_srm_color_mode_t dst_fmt,
                       SemaphoreHandle_t done_sem) {
  if (!client)
    return false;

  ppa_srm_oper_config_t config = {};

  // Input Config
  config.in.buffer = (const void *)src;
  config.in.pic_w = src_w;
  config.in.pic_h = src_h;
  config.in.block_w = src_w;
  config.in.block_h = src_h;
  config.in.block_offset_x = 0;
  config.in.block_offset_y = 0;
  config.in.srm_cm = src_fmt;

  // Output Config
  config.out.buffer = (void *)dst;
  config.out.pic_w = dst_w;
  config.out.pic_h = dst_h;

  // Center the image in the destination
  uint32_t x_off = (dst_w > src_w) ? (dst_w - src_w) / 2 : 0;
  uint32_t y_off = (dst_h > src_h) ? (dst_h - src_h) / 2 : 0;

  config.out.block_offset_x = x_off;
  config.out.block_offset_y = y_off;
  // block_w/block_h are not present in ppa_out_pic_blk_config_t.
  // Output block size is determined by input block size * scaling factor.
  config.out.srm_cm = dst_fmt;

  // Scaling factors (1.0 = no scaling)
  config.scale_x = 1.0f;
  config.scale_y = 1.0f;
  config.rotation_angle = PPA_SRM_ROTATION_ANGLE_0;

  config.mode = PPA_TRANS_MODE_NON_BLOCKING;
  config.user_data = (void *)done_sem;

  esp_err_t ret = ppa_do_scale_rotate_mirror(client, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "PPA SRM failed: %s", esp_err_to_name(ret));
    return false;
  }
  return true;
}
