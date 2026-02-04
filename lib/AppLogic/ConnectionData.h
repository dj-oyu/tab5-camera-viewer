#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>

class ConnectionData {
public:
  bool init();

  // Writer interface (ConnectionTask)
  void update(int total, int webrtc, int mjpeg);

  // Reader interface (OverlayRenderer)
  bool tryRead(int *out_total, int *out_webrtc, int *out_mjpeg);

private:
  SemaphoreHandle_t mutex_ = nullptr;
  int total_ = 0;
  int webrtc_ = 0;
  int mjpeg_ = 0;
  bool updated_ = false;
};
