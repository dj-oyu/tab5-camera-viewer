#include "AppLogic.h"
#include "BatteryTask.h"
#include "ConnectionTask.h"
#include "DecodeTask.h"
#include "DetectionTask.h"
#include "DisplayInit.h"
#include "FetchTask.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include "RecordingData.h"
#include "RecordingTask.h"
#include "RenderTask.h"
#include "TailscaleTask.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <PPAPipeline.h>

namespace {
PipelineContext *g_ctx = nullptr;

// REC button touch area (framebuffer coordinates - portrait 720x1280)
// Calculated from OverlayRenderer:
//   - Tile 2 starts at X = 2 * 240 = 480
//   - Button in tile: BTN_X=15, BTN_Y=20, BTN_W=130, BTN_H=80
//   - With rotation 3: fb_x = tileOffset + tile_y, fb_y = tile_x
//   - Button fb_x: 480 + 20 = 500 to 480 + 100 = 580
//   - Button fb_y: 15 to 145
constexpr int REC_BUTTON_X_CENTER = 540;  // (500 + 580) / 2
constexpr int REC_BUTTON_Y_CENTER = 80;   // (15 + 145) / 2
constexpr int REC_BUTTON_HALF_W = 50;     // Half width (80/2 + margin)
constexpr int REC_BUTTON_HALF_H = 80;     // Half height (130/2 + margin)
constexpr int TOUCH_MARGIN = 30;          // Extra margin for easier touch

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

  Serial.println("AppLogic v66: PPA direct-to-FB, DMA2D gate=2");
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

  // Core 0: Decode (HW JPEG decoder)
  // Core 1: Render + Fetch + control tasks
  // Render is kept at highest priority on Core 1 to avoid starvation from network parsing.
  RenderTask::start(ctx, 6, 1);      // Core 1 - PPA + DSI submit
  FetchTask::start(ctx, 5, 1);       // Core 1 - MJPEG stream parser (WiFi init here)
  DecodeTask::start(ctx, 5, 0);      // Core 0 - JPEG HW decode
  TailscaleTask::start(ctx, 3, 0);   // Core 0 - Tailscale VPN (waits for WiFi)
  if (VERIFY_FETCH_ONLY_MODE) {
    Serial.println("Verify mode: fetch-only (Detection/Connection/Recording disabled)");
  } else {
    DetectionTask::start(ctx, 4, 1);   // Detection API (SSE stream)
    ConnectionTask::start(ctx, 4, 1);  // Connection API (SSE stream)
    RecordingTask::start(ctx, 1, 1);   // Recording API (low priority, won't block touch)
    BatteryTask::start(ctx, 1, 1);     // Battery monitor (local M5.Power polling)
  }
}

void AppLogic::update() {
  M5.update();

  // Handle touch input for REC button
  if (g_ctx != nullptr) {
    auto touch = M5.Touch.getDetail();
    if (touch.wasPressed() && isInRecButton(touch.x, touch.y)) {
      g_ctx->recordingData().requestToggle();
    }
  }

  // Yield to IDLE task on Core 1 to prevent task WDT timeout
  vTaskDelay(1);
}
