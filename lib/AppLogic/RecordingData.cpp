#include "RecordingData.h"
#include <Arduino.h>
#include <cstring>

bool RecordingData::init() {
  mutex_ = xSemaphoreCreateMutex();
  return mutex_ != nullptr;
}

void RecordingData::setRecording(bool recording, const char *filename) {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    pendingAction_ = PendingAction::None;  // Clear pending action
    if (recording) {
      state_ = RecordingState::Recording;
      duration_ = 0.0f;
      if (filename) {
        strncpy(filename_, filename, sizeof(filename_) - 1);
        filename_[sizeof(filename_) - 1] = '\0';
      }
      error_[0] = '\0';
    } else {
      state_ = RecordingState::Idle;
    }
    xSemaphoreGive(mutex_);
  }
}

void RecordingData::setDuration(float duration) {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    duration_ = duration;
    xSemaphoreGive(mutex_);
  }
}

void RecordingData::setError(const char *error) {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    state_ = RecordingState::Error;
    pendingAction_ = PendingAction::None;  // Clear pending action
    if (error) {
      strncpy(error_, error, sizeof(error_) - 1);
      error_[sizeof(error_) - 1] = '\0';
    }
    xSemaphoreGive(mutex_);
    Serial.printf("RecordingData: Error set: %s\n", error ? error : "(null)");
  }
}

void RecordingData::clearError() {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    error_[0] = '\0';
    if (state_ == RecordingState::Error) {
      state_ = RecordingState::Idle;
    }
    xSemaphoreGive(mutex_);
  }
}

RecordingState RecordingData::getState() {
  RecordingState state = RecordingState::Idle;
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    state = state_;
    xSemaphoreGive(mutex_);
  }
  return state;
}

float RecordingData::getDuration() {
  float duration = 0.0f;
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    duration = duration_;
    xSemaphoreGive(mutex_);
  }
  return duration;
}

bool RecordingData::hasError(char *out_error, size_t maxLen) {
  bool hasErr = false;
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    hasErr = (error_[0] != '\0');
    if (hasErr && out_error && maxLen > 0) {
      strncpy(out_error, error_, maxLen - 1);
      out_error[maxLen - 1] = '\0';
    }
    xSemaphoreGive(mutex_);
  }
  return hasErr;
}

PendingAction RecordingData::getPendingAction() {
  PendingAction action = PendingAction::None;
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    action = pendingAction_;
    xSemaphoreGive(mutex_);
  }
  return action;
}

void RecordingData::requestToggle() {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    // Determine action based on current state
    if (state_ == RecordingState::Recording || state_ == RecordingState::Error) {
      pendingAction_ = PendingAction::Stop;
    } else if (state_ == RecordingState::Idle) {
      pendingAction_ = PendingAction::Start;
    }
    // Only change state if not already pending
    if (state_ != RecordingState::Pending && pendingAction_ != PendingAction::None) {
      prevState_ = state_;
      state_ = RecordingState::Pending;
      Serial.printf("RecordingData: Pending action=%d (prev state=%d)\n",
                    (int)pendingAction_, (int)prevState_);
    }
    xSemaphoreGive(mutex_);
  }
}

PendingAction RecordingData::checkPendingAction() {
  PendingAction action = PendingAction::None;
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    action = pendingAction_;
    // Don't clear here - will be cleared when setRecording/setError is called
    xSemaphoreGive(mutex_);
  }
  return action;
}
