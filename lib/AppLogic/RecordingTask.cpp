#include "RecordingTask.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include "RecordingData.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>

namespace {

constexpr uint32_t STATUS_POLL_INTERVAL_MS = 2000;   // Poll status every 2 seconds
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 2000;    // Send heartbeat every 2 seconds (HTTP is fast now)
constexpr uint32_t HTTP_TIMEOUT_MS = 3000;          // Timeout for start operation
constexpr uint32_t STOP_TIMEOUT_MS = 30000;         // Long timeout for stop (ffmpeg mux takes time)
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 2000;     // Short timeout for heartbeat

WiFiClient httpClient;
uint32_t recordingStartTime = 0;

// Returns: 1 = success, 0 = failed, -1 = server says not recording
int sendHeartbeat() {
#ifdef RECORDING_BASE_URL
  HTTPClient http;
  String url = String(RECORDING_BASE_URL) + "/heartbeat";
  http.begin(httpClient, url);
  http.setTimeout(HEARTBEAT_TIMEOUT_MS);  // Short timeout for heartbeat

  int httpCode = http.POST("");
  http.end();

  if (httpCode == 200) {
    return 1;  // Success
  } else if (httpCode == 400) {
    // Server says not recording - we're out of sync
    Serial.println("RecordingTask: Heartbeat 400 - server not recording");
    return -1;
  } else {
    Serial.printf("RecordingTask: Heartbeat failed, HTTP %d\n", httpCode);
    return 0;
  }
#else
  return 0;
#endif
}

bool startRecording(RecordingData &data) {
#ifdef RECORDING_BASE_URL
  HTTPClient http;
  String url = String(RECORDING_BASE_URL) + "/start";
  http.begin(httpClient, url);
  http.setTimeout(HTTP_TIMEOUT_MS);

  Serial.println("RecordingTask: Sending start request...");
  int httpCode = http.POST("");
  Serial.printf("RecordingTask: Start response code: %d\n", httpCode);

  if (httpCode == 200) {
    String response = http.getString();
    Serial.printf("RecordingTask: Start response body: %s\n", response.c_str());
    http.end();

    JsonDocument doc;
    DeserializationError jsonErr = deserializeJson(doc, response);
    if (jsonErr == DeserializationError::Ok) {
      // Server returns "file" key, not "filename"
      const char *filename = doc["file"] | "";
      data.setRecording(true, filename);
      Serial.printf("RecordingTask: Started recording: %s\n", filename);
      return true;
    } else {
      Serial.printf("RecordingTask: JSON parse error: %s\n", jsonErr.c_str());
      data.setError("JSON error");
    }
  } else {
    Serial.printf("RecordingTask: Start failed, HTTP %d\n", httpCode);
    data.setError("Start failed");
  }
  http.end();
  return false;
#else
  return false;
#endif
}

bool stopRecording(RecordingData &data) {
#ifdef RECORDING_BASE_URL
  HTTPClient http;
  String url = String(RECORDING_BASE_URL) + "/stop";
  http.begin(httpClient, url);
  http.setTimeout(STOP_TIMEOUT_MS);  // Long timeout for ffmpeg mux

  Serial.println("RecordingTask: Sending stop request...");
  int httpCode = http.POST("");
  Serial.printf("RecordingTask: Stop response code: %d\n", httpCode);

  if (httpCode == 200) {
    String response = http.getString();
    Serial.printf("RecordingTask: Stop response body: %s\n", response.c_str());
    http.end();
    data.setRecording(false);
    Serial.println("RecordingTask: Stopped recording, state set to Idle");
    return true;
  } else if (httpCode == 400) {
    // Server says "not recording" - that's fine, just sync to Idle
    String response = http.getString();
    Serial.printf("RecordingTask: Stop 400 (not recording), body: %s\n", response.c_str());
    http.end();
    data.setRecording(false);
    Serial.println("RecordingTask: Server not recording, state set to Idle");
    return true;  // Not an error - we wanted to stop, it's stopped
  } else {
    String response = http.getString();
    Serial.printf("RecordingTask: Stop failed, HTTP %d, body: %s\n", httpCode, response.c_str());
    data.setError("Stop failed");
  }
  http.end();
  return false;
#else
  return false;
#endif
}

bool fetchStatus(RecordingData &data) {
#ifdef RECORDING_BASE_URL
  HTTPClient http;
  String url = String(RECORDING_BASE_URL) + "/status";
  http.begin(httpClient, url);
  http.setTimeout(HTTP_TIMEOUT_MS);

  int httpCode = http.GET();
  Serial.printf("RecordingTask: Status response code: %d\n", httpCode);

  if (httpCode != 200) {
    http.end();
    return false;
  }

  String response = http.getString();
  Serial.printf("RecordingTask: Status response body: %s\n", response.c_str());
  http.end();

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, response);
  if (error) {
    Serial.printf("RecordingTask: Status JSON parse error: %s\n", error.c_str());
    return false;
  }

  bool recording = doc["recording"] | false;
  const char *filename = doc["file"] | "";  // Server uses "file" key
  Serial.printf("RecordingTask: Status parsed - recording=%d, file=%s\n", recording, filename);

  if (recording) {
    data.setRecording(true, filename);
  } else {
    data.setRecording(false);
  }

  return true;
#else
  return false;
#endif
}

void recordingTask(void *pvParameters) {
  auto *ctx = static_cast<PipelineContext *>(pvParameters);
  RecordingData &data = ctx->recordingData();

  // Wait for WiFi
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  Serial.println("RecordingTask: WiFi ready");

#ifdef RECORDING_BASE_URL
  Serial.printf("RecordingTask: URL = %s\n", RECORDING_BASE_URL);

  uint32_t lastHeartbeat = 0;
  bool wasRecording = false;
  bool initialStatusFetched = false;

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    uint32_t now = millis();

    // Fetch initial status once at startup
    if (!initialStatusFetched) {
      fetchStatus(data);
      initialStatusFetched = true;
      Serial.println("RecordingTask: Initial status fetched");
    }

    // Check for pending action from touch input
    PendingAction action = data.checkPendingAction();
    if (action != PendingAction::None) {
      Serial.printf("RecordingTask: Processing action=%d\n", (int)action);

      if (action == PendingAction::Stop) {
        Serial.println("RecordingTask: Calling stopRecording...");
        bool result = stopRecording(data);
        Serial.printf("RecordingTask: stopRecording result=%d\n", result);
      } else if (action == PendingAction::Start) {
        Serial.println("RecordingTask: Calling startRecording...");
        bool result = startRecording(data);
        Serial.printf("RecordingTask: startRecording result=%d\n", result);
      }
    }

    RecordingState state = data.getState();
    bool isRecording = (state == RecordingState::Recording);

    // Detect recording state change
    if (isRecording && !wasRecording) {
      // Recording just started
      recordingStartTime = now;
      Serial.println("RecordingTask: Recording started, tracking duration");
    } else if (!isRecording && wasRecording) {
      // Recording just stopped
      recordingStartTime = 0;
      Serial.println("RecordingTask: Recording stopped");
    }
    wasRecording = isRecording;

    if (isRecording) {
      // Update duration locally
      if (recordingStartTime > 0) {
        float duration = (now - recordingStartTime) / 1000.0f;
        data.setDuration(duration);
      }

      // Send heartbeat - but skip if there's a pending stop action
      if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        // Check for pending stop before slow HTTP call
        PendingAction pendingBeforeHttp = data.getPendingAction();
        if (pendingBeforeHttp == PendingAction::Stop) {
          Serial.println("RecordingTask: Skipping heartbeat - stop pending");
        } else {
          int hbResult = sendHeartbeat();
          if (hbResult == -1) {
            // Server says not recording - sync our state
            Serial.println("RecordingTask: Server not recording, syncing to Idle");
            data.setRecording(false);
          } else if (hbResult == 0) {
            Serial.println("RecordingTask: Heartbeat failed");
          }
        }
        lastHeartbeat = now;
      }
    }

    // No continuous status polling - only triggered by user actions
    vTaskDelay(pdMS_TO_TICKS(100));
  }
#else
  Serial.println("RecordingTask: RECORDING_BASE_URL not defined, task idle");
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
#endif
}

} // namespace

void RecordingTask::start(PipelineContext &ctx, UBaseType_t priority,
                          BaseType_t core) {
  xTaskCreatePinnedToCore(recordingTask, "Recording", 8192, &ctx, priority,
                          nullptr, core);
}
