#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>

enum class ConnectionState {
  Connecting,  // Attempting to connect
  Connected,   // Successfully connected and receiving data
  Error        // Connection failed or disconnected
};

class ConnectionData {
public:
  bool init();

  // Writer interface (ConnectionTask)
  void update(int total, int webrtc, int mjpeg);
  void setState(ConnectionState state, int httpCode = 0);

  // Reader interface (OverlayRenderer)
  bool tryRead(int *out_total, int *out_webrtc, int *out_mjpeg);
  ConnectionState getState(int *out_httpCode = nullptr);

private:
  SemaphoreHandle_t mutex_ = nullptr;
  int total_ = 0;
  int webrtc_ = 0;
  int mjpeg_ = 0;
  bool updated_ = false;
  ConnectionState state_ = ConnectionState::Connecting;
  int httpCode_ = 0;
};
