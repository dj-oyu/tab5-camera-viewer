#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>

constexpr int MAX_DETECTIONS = 12;
constexpr int MAX_LABEL_LEN = 32;

struct Detection
{
  char label[MAX_LABEL_LEN];
  int x;
  int y;
  int w;
  int h;
  float confidence;
};

class DetectionData
{
public:
  [[nodiscard]] bool init();

  // Writer interface (DetectionTask)
  void beginUpdate();
  void setDetectionCount(int count);
  void setDetection(int index, const char *label, int x, int y, int w, int h,
                    float confidence);
  void setTimestamp(uint32_t ts);
  void endUpdate();

  // Reader interface (RenderTask)
  [[nodiscard]] bool tryRead(Detection *out_detections, int &out_count);
  [[nodiscard]] bool tryRead(Detection *out_detections, int &out_count,
                             uint32_t &out_timestamp);

private:
  SemaphoreHandle_t mutex_ = nullptr;
  Detection detections_[MAX_DETECTIONS];
  int count_ = 0;
  uint32_t timestamp_ = 0;
  bool updated_ = false;
};
