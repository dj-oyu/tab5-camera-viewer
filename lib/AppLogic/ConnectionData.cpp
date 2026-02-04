#include "ConnectionData.h"

bool ConnectionData::init() {
  mutex_ = xSemaphoreCreateMutex();
  return mutex_ != nullptr;
}

void ConnectionData::update(int total, int webrtc, int mjpeg) {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    total_ = total;
    webrtc_ = webrtc;
    mjpeg_ = mjpeg;
    updated_ = true;
    xSemaphoreGive(mutex_);
  }
}

bool ConnectionData::tryRead(int *out_total, int *out_webrtc, int *out_mjpeg) {
  if (xSemaphoreTake(mutex_, 0) != pdTRUE) {
    return false;
  }

  if (!updated_) {
    xSemaphoreGive(mutex_);
    return false;
  }

  *out_total = total_;
  *out_webrtc = webrtc_;
  *out_mjpeg = mjpeg_;
  updated_ = false;
  xSemaphoreGive(mutex_);
  return true;
}
