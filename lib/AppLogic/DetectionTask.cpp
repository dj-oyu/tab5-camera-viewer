#include "DetectionTask.h"
#include "DetectionData.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>

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

    // Try multiple possible field names (use modern ArduinoJson API)
    const char *label = nullptr;
    if (det["class_name"].is<const char *>()) {
      label = det["class_name"].as<const char *>();
    } else if (det["label"].is<const char *>()) {
      label = det["label"].as<const char *>();
    } else if (det["name"].is<const char *>()) {
      label = det["name"].as<const char *>();
    }
    if (!label || label[0] == '\0') {
      label = "unknown";
    }
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
  WiFiClient httpClient;
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

    Serial.println("DetectionTask: begin()...");
    http.begin(httpClient, DETECTION_URL);
    http.setTimeout(DETECTION_TIMEOUT_MS);
    http.setReuse(false);  // Don't reuse connection for SSE

    Serial.println("DetectionTask: GET()...");
    uint32_t startTime = millis();
    int httpCode = http.GET();
    Serial.printf("DetectionTask: GET() took %lu ms, code=%d\n", millis() - startTime, httpCode);
    if (httpCode != 200) {
      Serial.printf("Detection HTTP failed: %d\n", httpCode);
      http.end();
      vTaskDelay(pdMS_TO_TICKS(DETECTION_RECONNECT_DELAY_MS));
      continue;
    }

    Serial.println("Detection stream connected");
    WiFiClient *client = http.getStreamPtr();
    client->setTimeout(1);  // Non-blocking read with short timeout

    // Server-Sent Events (SSE) format: "data: {...}\n"
    // Keepalive format: ": keepalive\n"
    uint32_t lastActivity = millis();
    String lastDataLine;  // Keep only the most recent data line
    while (http.connected() && WiFi.status() == WL_CONNECTED) {
      if (client->available()) {
        // Read ALL available lines, keep only the LAST "data:" line
        // This ensures we always display the most recent detection
        lastDataLine = "";
        int linesRead = 0;
        while (client->available()) {
          line = client->readStringUntil('\n');
          line.trim();
          linesRead++;
          if (line.startsWith("data: ")) {
            lastDataLine = line.substring(6);
          }
        }
        lastActivity = millis();

        // Process only the most recent data line
        if (lastDataLine.length() > 0) {
          if (linesRead > 1) {
            Serial.printf("Detection: processed latest of %d lines\n", linesRead);
          }
          parseDetectionJson(lastDataLine.c_str(), data);
        }
      } else {
        // Check for stream timeout (no data for 60 seconds)
        if (millis() - lastActivity > 60000) {
          Serial.println("DetectionTask: Stream idle timeout");
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
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
