#include "DetectionTask.h"
#include "DetectionData.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>

#ifdef TAILSCALE_AUTH_KEY
#include "TailscaleTask.h"
#endif

namespace {

bool tryReadInt(const JsonObjectConst &obj, const char *key, int &out) {
  JsonVariantConst value = obj[key];
  if (value.is<int>()) {
    out = value.as<int>();
    return true;
  }
  if (value.is<long>()) {
    out = static_cast<int>(value.as<long>());
    return true;
  }
  if (value.is<float>()) {
    out = static_cast<int>(value.as<float>());
    return true;
  }
  if (value.is<double>()) {
    out = static_cast<int>(value.as<double>());
    return true;
  }
  return false;
}

int readBoxValue(const JsonObjectConst &det, const char *key1,
                 const char *key2) {
  int v = 0;
  if (tryReadInt(det, key1, v) || (key2 && tryReadInt(det, key2, v))) {
    return v;
  }

  JsonVariantConst bboxValue = det["bbox"];
  if (bboxValue.is<JsonObjectConst>()) {
    JsonObjectConst bbox = bboxValue.as<JsonObjectConst>();
    if (tryReadInt(bbox, key1, v) || (key2 && tryReadInt(bbox, key2, v))) {
      return v;
    }
  }

  return 0;
}

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
    int x = readBoxValue(det, "x", "left");
    int y = readBoxValue(det, "y", "top");
    int w = readBoxValue(det, "w", "width");
    int h = readBoxValue(det, "h", "height");

    data.setDetection(count, label, x, y, w, h, confidence);
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

#ifdef TAILSCALE_AUTH_KEY
  {
    EventGroupHandle_t vpn_eg = TailscaleTask::eventGroup();
    if (vpn_eg) {
      Serial.println("DetectionTask: Waiting for Tailscale VPN...");
      xEventGroupWaitBits(vpn_eg, VPN_CONNECTED_BIT, pdFALSE, pdTRUE,
                          portMAX_DELAY);
      Serial.println("DetectionTask: VPN ready");
    }
  }
#endif

#ifdef DETECTION_URL
  WiFiClient httpClient;
  HTTPClient http;
  String line;
  line.reserve(1024);
  uint32_t lastAppliedAt = 0;

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("DetectionTask: WiFi disconnected, waiting...");
      vTaskDelay(pdMS_TO_TICKS(DETECTION_RECONNECT_DELAY_MS));
      continue;
    }

    http.begin(httpClient, DETECTION_URL);
    http.setTimeout(DETECTION_TIMEOUT_MS);
    http.setReuse(false);  // Don't reuse connection for SSE

    int httpCode = http.GET();
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
          SideLoadProfile profile = ctx->sideLoadProfile();
          uint32_t now = millis();
          if (profile.detection_min_interval_ms == 0 ||
              (now - lastAppliedAt) >= profile.detection_min_interval_ms) {
            parseDetectionJson(lastDataLine.c_str(), data);
            lastAppliedAt = now;
          }
        }
      } else {
        // Check for stream timeout (safety net for dead TCP connections)
        uint32_t idle_s = (millis() - lastActivity) / 1000;
        if (idle_s * 1000 >= DETECTION_SSE_IDLE_TIMEOUT_MS) {
          Serial.printf("DetectionTask: SSE idle %us, reconnecting\n", idle_s);
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
  xTaskCreatePinnedToCore(detectionTask, "Detection", 12288, &ctx, priority,
                          nullptr, core);
}
