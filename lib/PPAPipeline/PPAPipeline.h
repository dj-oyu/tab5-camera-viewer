#ifndef PPAPIPELINE_H
#define PPAPIPELINE_H

#include <cstdint>
#include <driver/ppa.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class PPAPipeline {
public:
  static bool begin();
  static bool copy(const uint8_t *src, uint8_t *dst, uint32_t src_w,
                   uint32_t src_h, uint32_t dst_w, uint32_t dst_h,
                   ppa_srm_color_mode_t src_fmt, ppa_srm_color_mode_t dst_fmt,
                   SemaphoreHandle_t done_sem = nullptr);

private:
  static ppa_client_handle_t client;
  static bool ppa_event_cb(ppa_client_handle_t ppa_client,
                           ppa_event_data_t *event_data, void *user_data);
};

#endif // PPAPIPELINE_H
