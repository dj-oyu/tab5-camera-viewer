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

  FetchTask::start(ctx);
  DecodeTask::start(ctx);
  RenderTask::start(ctx);
}

void AppLogic::update() { M5.update(); }
