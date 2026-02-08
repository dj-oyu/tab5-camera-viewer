#include "BatteryData.h"

bool BatteryData::init()
{
  mutex_ = xSemaphoreCreateMutex();
  return mutex_ != nullptr;
}

void BatteryData::update(int percent, bool charging, int currentMa,
                         int busVoltageMv)
{
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE)
  {
    percent_ = percent;
    charging_ = charging;
    currentMa_ = currentMa;
    busVoltageMv_ = busVoltageMv;
    updated_ = true;
    xSemaphoreGive(mutex_);
  }
}

bool BatteryData::tryRead(int &out_percent, bool &out_charging,
                          int &out_currentMa, int &out_busVoltageMv)
{
  if (xSemaphoreTake(mutex_, 0) != pdTRUE)
  {
    return false;
  }

  if (!updated_)
  {
    xSemaphoreGive(mutex_);
    return false;
  }

  out_percent = percent_;
  out_charging = charging_;
  out_currentMa = currentMa_;
  out_busVoltageMv = busVoltageMv_;
  updated_ = false;
  xSemaphoreGive(mutex_);
  return true;
}
