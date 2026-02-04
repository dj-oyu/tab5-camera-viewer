#include "ConnectionTask.h"
#include "ConnectionData.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

namespace {

void parseConnectionJson(const char *json, ConnectionData &data) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.printf("Connection JSON parse error: %s\n", error.c_str());
    return;
  }

  int total = doc["total"] | 0;
  int webrtc = doc["webrtc"] | 0;
  int mjpeg = doc["mjpeg"] | 0;

  data.update(total, webrtc, mjpeg);
  Serial.printf("Connection parsed: total=%d, webrtc=%d, mjpeg=%d\n",
                total, webrtc, mjpeg);
}

void connectionTask(void *pvParameters) {
  auto *ctx = static_cast<PipelineContext *>(pvParameters);
  ConnectionData &data = ctx->connectionData();

  // Wait for WiFi (initialized by FetchTask)
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  Serial.println("ConnectionTask: WiFi ready");

#ifdef CONNECTIONS_URL
  WiFiClientSecure secureClient;
  secureClient.setInsecure(); // Skip certificate verification

  HTTPClient http;
  String line;
  line.reserve(256);

  Serial.printf("ConnectionTask: URL = %s\n", CONNECTIONS_URL);

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("ConnectionTask: WiFi disconnected, waiting...");
      data.setState(ConnectionState::Error, -1);  // -1 indicates WiFi error
      vTaskDelay(pdMS_TO_TICKS(DETECTION_RECONNECT_DELAY_MS));
      continue;
    }

    data.setState(ConnectionState::Connecting);
    Serial.println("ConnectionTask: begin()...");
    http.begin(secureClient, CONNECTIONS_URL);
    http.setTimeout(DETECTION_TIMEOUT_MS);
    http.setReuse(false);  // Don't reuse connection for SSE

    Serial.println("ConnectionTask: GET()...");
    uint32_t startTime = millis();
    int httpCode = http.GET();
    Serial.printf("ConnectionTask: GET() took %lu ms, code=%d\n",
                  millis() - startTime, httpCode);
    if (httpCode != 200) {
      Serial.printf("Connection HTTP failed: %d\n", httpCode);
      data.setState(ConnectionState::Error, httpCode);
      http.end();
      vTaskDelay(pdMS_TO_TICKS(DETECTION_RECONNECT_DELAY_MS));
      continue;
    }

    data.setState(ConnectionState::Connected);
    Serial.println("Connection stream connected");
    WiFiClient *client = http.getStreamPtr();
    client->setTimeout(1);  // Non-blocking read with short timeout

    // Server-Sent Events (SSE) format: "data: {...}\n"
    uint32_t lastActivity = millis();
    while (http.connected() && WiFi.status() == WL_CONNECTED) {
      if (client->available()) {
        line = client->readStringUntil('\n');
        line.trim();
        lastActivity = millis();

        // SSE data line starts with "data: "
        if (line.startsWith("data: ")) {
          String jsonStr = line.substring(6); // Remove "data: " prefix
          if (jsonStr.length() > 0) {
            parseConnectionJson(jsonStr.c_str(), data);
          }
        }
        // SSE comment (keepalive) starts with ":"
        // Just acknowledge it to keep connection alive
      } else {
        // Check for stream timeout (no data for 60 seconds)
        if (millis() - lastActivity > 60000) {
          Serial.println("ConnectionTask: Stream idle timeout");
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
      }
    }

    Serial.println("Connection stream disconnected");
    data.setState(ConnectionState::Error, 0);  // 0 indicates stream disconnected
    http.end();
    vTaskDelay(pdMS_TO_TICKS(DETECTION_RECONNECT_DELAY_MS));
  }
#else
  Serial.println("ConnectionTask: CONNECTIONS_URL not defined, task idle");
  data.setState(ConnectionState::Error, -2);  // -2 indicates URL not configured
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
#endif
}

} // namespace

void ConnectionTask::start(PipelineContext &ctx, UBaseType_t priority,
                           BaseType_t core) {
  xTaskCreatePinnedToCore(connectionTask, "Connection", 8192, &ctx, priority,
                          nullptr, core);
}
