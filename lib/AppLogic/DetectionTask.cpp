#include "DetectionTask.h"
#include "DetectionData.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

namespace {

void parseDetectionJson(const char *json, DetectionData &data) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.printf("Detection JSON parse error: %s\n", error.c_str());
    return;
  }

  data.beginUpdate();

  JsonArray detections = doc["detections"];
  int count = 0;
  for (JsonObject det : detections) {
    if (count >= MAX_DETECTIONS)
      break;

    const char *label = det["label"] | "unknown";
    float confidence = det["confidence"] | 0.0f;

    data.setDetection(count, label, confidence);
    count++;
  }
  data.setDetectionCount(count);
  data.setTimestamp(millis());

  data.endUpdate();
}

void detectionTask(void *pvParameters) {
  auto *ctx = static_cast<PipelineContext *>(pvParameters);
  DetectionData &data = ctx->detectionData();

  // Wait for WiFi (initialized by FetchTask)
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  Serial.println("DetectionTask: WiFi ready");

#ifdef DETECTION_URL
  WiFiClientSecure secureClient;
  secureClient.setInsecure(); // Skip certificate verification

  HTTPClient http;
  String line;
  line.reserve(1024);

  Serial.printf("DetectionTask: URL = %s\n", DETECTION_URL);

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("DetectionTask: WiFi disconnected, waiting...");
      vTaskDelay(pdMS_TO_TICKS(DETECTION_RECONNECT_DELAY_MS));
      continue;
    }

    http.begin(secureClient, DETECTION_URL);
    http.setTimeout(DETECTION_TIMEOUT_MS);

    int httpCode = http.GET();
    if (httpCode != 200) {
      Serial.printf("Detection HTTP failed: %d\n", httpCode);
      http.end();
      vTaskDelay(pdMS_TO_TICKS(DETECTION_RECONNECT_DELAY_MS));
      continue;
    }

    Serial.println("Detection stream connected");
    Stream &stream = http.getStream();

    // Server-Sent Events (SSE) format: "data: {...}\n"
    while (http.connected() && WiFi.status() == WL_CONNECTED) {
      if (stream.available()) {
        line = stream.readStringUntil('\n');
        line.trim();

        // SSE data line starts with "data: "
        if (line.startsWith("data: ")) {
          String jsonStr = line.substring(6); // Remove "data: " prefix
          if (jsonStr.length() > 0) {
            parseDetectionJson(jsonStr.c_str(), data);
          }
        }
      } else {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }

    Serial.println("Detection stream disconnected");
    http.end();
    vTaskDelay(pdMS_TO_TICKS(DETECTION_RECONNECT_DELAY_MS));
  }
#else
  Serial.println("DetectionTask: DETECTION_URL not defined, task idle");
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
#endif
}

} // namespace

void DetectionTask::start(PipelineContext &ctx, UBaseType_t priority,
                          BaseType_t core) {
  xTaskCreatePinnedToCore(detectionTask, "Detection", 8192, &ctx, priority,
                          nullptr, core);
}
