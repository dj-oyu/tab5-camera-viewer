#include "AppLogic.h"
#include "DecodeTask.h"
#include "DisplayInit.h"
#include "FetchTask.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include "RenderTask.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <PPAPipeline.h>

void AppLogic::begin() {
  auto cfg = M5.config();
  cfg.output_power = true;
  M5.begin(cfg);

  Serial.println("AppLogic v64: Dual Framebuffer + PPA Zero-copy");
  Serial.printf("Panel: %dx%d\n", PANEL_WIDTH, PANEL_HEIGHT);

  if (!PPAPipeline::begin()) {
    Serial.println("Failed to initialize PPA Pipeline!");
  }

  static PipelineContext ctx;
  if (!ctx.init()) {
    Serial.println("PipelineContext init failed");
    return;
  }

  if (!DisplayInit::init(ctx)) {
    Serial.println("Display init failed");
    return;
  }

  vTaskDelay(1000);

  // Core 0: Decode (HW JPEG decoder, low CPU usage)
  // Core 1: Fetch + Render (Network I/O + display)
  FetchTask::start(ctx, 6, 1);  // Core 1
  DecodeTask::start(ctx, 5, 0); // Core 0 - HW decoder doesn't block WiFi
  RenderTask::start(ctx, 5, 1); // Core 1
}

void AppLogic::update() { M5.update(); }
