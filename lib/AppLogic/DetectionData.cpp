#include "DetectionData.h"
#include <cstring>

bool DetectionData::init()
{
  mutex_ = xSemaphoreCreateMutex();
  return mutex_ != nullptr;
}

void DetectionData::beginUpdate()
{
  if (mutex_)
  {
    xSemaphoreTake(mutex_, portMAX_DELAY);
  }
}

void DetectionData::setDetectionCount(int count)
{
  if (count < 0)
    count = 0;
  if (count > MAX_DETECTIONS)
    count = MAX_DETECTIONS;
  count_ = count;
}

void DetectionData::setDetection(int index, const char *label,
                                 float confidence)
{
  if (index < 0 || index >= MAX_DETECTIONS)
    return;

  strncpy(detections_[index].label, label, MAX_LABEL_LEN - 1);
  detections_[index].label[MAX_LABEL_LEN - 1] = '\0';
  detections_[index].confidence = confidence;
}

void DetectionData::setTimestamp(uint32_t ts)
{
  timestamp_ = ts;
}

void DetectionData::endUpdate()
{
  updated_ = true;
  if (mutex_)
  {
    xSemaphoreGive(mutex_);
  }
}

bool DetectionData::tryRead(Detection *out_detections, int &out_count)
{
  if (!mutex_)
    return false;

  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1)) != pdTRUE)
  {
    return false; // Writer is updating, skip this frame
  }

  out_count = count_;
  if (out_detections && count_ > 0)
  {
    memcpy(out_detections, detections_, sizeof(Detection) * count_);
  }

  bool was_updated = updated_;
  updated_ = false;

  xSemaphoreGive(mutex_);
  return was_updated;
}

bool DetectionData::tryRead(Detection *out_detections, int &out_count,
                            uint32_t &out_timestamp)
{
  if (!mutex_)
    return false;

  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1)) != pdTRUE)
  {
    return false; // Writer is updating, skip this frame
  }

  out_count = count_;
  out_timestamp = timestamp_;
  if (out_detections && count_ > 0)
  {
    memcpy(out_detections, detections_, sizeof(Detection) * count_);
  }

  bool was_updated = updated_;
  updated_ = false;

  xSemaphoreGive(mutex_);
  return was_updated;
}
