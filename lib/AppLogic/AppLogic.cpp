#include "AppLogic.h"
#include "BatteryTask.h"
#include "ConnectionTask.h"
#include "DetectionTask.h"
#include "DisplayInit.h"
#include "FetchTask.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include "RecordingData.h"
#include "RecordingTask.h"
#include "RenderPipelineTask.h"
#include "TailscaleTask.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <PPAPipeline.h>


namespace {
PipelineContext *g_ctx = nullptr;

constexpr int REC_BUTTON_X_CENTER = 540;
constexpr int REC_BUTTON_Y_CENTER = 80;
constexpr int REC_BUTTON_HALF_W = 50;
constexpr int REC_BUTTON_HALF_H = 80;
constexpr int TOUCH_MARGIN = 30;

bool isInRecButton(int x, int y) {
  int dx = x - REC_BUTTON_X_CENTER;
  int dy = y - REC_BUTTON_Y_CENTER;
  return (dx >= -(REC_BUTTON_HALF_W + TOUCH_MARGIN) &&
          dx <= (REC_BUTTON_HALF_W + TOUCH_MARGIN) &&
          dy >= -(REC_BUTTON_HALF_H + TOUCH_MARGIN) &&
          dy <= (REC_BUTTON_HALF_H + TOUCH_MARGIN));
}
} // namespace

void AppLogic::begin() {
  auto cfg = M5.config();
  cfg.output_power = true;
  M5.begin(cfg);

  Serial.println("AppLogic v70: SPSC ring, decode/PPA overlap");
  Serial.printf("Panel: %dx%d\n", PANEL_WIDTH, PANEL_HEIGHT);

  if (!PPAPipeline::begin()) {
    Serial.println("Failed to initialize PPA Pipeline!");
  }

  static PipelineContext ctx;
  g_ctx = &ctx;
  if (!ctx.init()) {
    Serial.println("PipelineContext init failed");
    return;
  }

  if (!DisplayInit::init(ctx)) {
    Serial.println("Display init failed");
    return;
  }

  vTaskDelay(1000);

  // RenderPipeline must be created first (Fetch needs its task handle)
  // Core 0: RenderPipeline(P6) — mostly DMA-bound (JPEG HW + PPA)
  // Core 1: Fetch(P5) — WiFi init + TCP recv + wg_mgr on same core
  RenderPipelineTask::start(ctx, 6, 0);
  FetchTask::start(ctx, 5, 1);
  TailscaleTask::start(ctx, 3, 0);
  if (VERIFY_FETCH_ONLY_MODE) {
    Serial.println("Verify mode: fetch-only");
  } else {
    DetectionTask::start(ctx, 4, 1);
    ConnectionTask::start(ctx, 4, 1);
    RecordingTask::start(ctx, 1, 1);
    BatteryTask::start(ctx, 1, 1);
  }
}

void AppLogic::update() {
  M5.update();
  if (g_ctx != nullptr) {
    auto touch = M5.Touch.getDetail();
    if (touch.wasPressed() && isInRecButton(touch.x, touch.y)) {
      g_ctx->recordingData().requestToggle();
    }
  }
  vTaskDelay(1);
}
