#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>

enum class RecordingState
{
  Idle,      // Not recording
  Recording, // Recording in progress
  Pending,   // Processing request (immediate UI feedback)
  Error      // Recording error
};

enum class PendingAction
{
  None,
  Start,
  Stop
};

class RecordingData
{
public:
  [[nodiscard]] bool init();

  // Writer interface (RecordingTask)
  void setRecording(bool recording, const char *filename = nullptr);
  void setDuration(float duration);
  void setError(const char *error);
  void clearError();

  // Reader interface (OverlayRenderer)
  [[nodiscard]] RecordingState getState() const;
  [[nodiscard]] float getDuration() const;
  [[nodiscard]] bool hasError() const;
  template <size_t N>
  [[nodiscard]] bool hasError(char (&out_error)[N]) const
  {
    return hasError(out_error, N);
  }
  [[nodiscard]] PendingAction getPendingAction() const; // What action is pending

  // Touch interface (AppLogic -> RecordingTask)
  void requestToggle();                                   // Request start/stop toggle (sets Pending state immediately)
  [[nodiscard]] PendingAction checkPendingAction() const; // Check and clear pending action

private:
  [[nodiscard]] bool hasError(char *out_error, size_t maxLen) const;
  SemaphoreHandle_t mutex_ = nullptr;
  RecordingState state_ = RecordingState::Idle;
  RecordingState prevState_ = RecordingState::Idle; // State before Pending
  PendingAction pendingAction_ = PendingAction::None;
  float duration_ = 0.0f;
  char filename_[64] = {0};
  char error_[64] = {0};
};
