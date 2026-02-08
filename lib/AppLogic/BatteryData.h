#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>

class BatteryData
{
public:
  [[nodiscard]] bool init();

  // Writer interface (BatteryTask)
  void update(int percent, bool charging, int currentMa, int busVoltageMv);

  // Reader interface (OverlayRenderer) - non-blocking
  [[nodiscard]] bool tryRead(int &out_percent, bool &out_charging,
                             int &out_currentMa, int &out_busVoltageMv);

private:
  SemaphoreHandle_t mutex_ = nullptr;
  int percent_ = -1; // -1 = not yet read
  bool charging_ = false;
  int currentMa_ = 0;
  int busVoltageMv_ = 0;
  bool updated_ = false;
};
