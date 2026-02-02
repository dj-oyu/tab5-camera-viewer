#include "RenderTask.h"
#include "OverlayRenderer.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <PPAPipeline.h>
#include <Arduino.h>
#include <esp_cache.h>

namespace {
struct ScalePlan {
  ppa_srm_rotation_angle_t rotation;
  float scale;
};

ScalePlan computeScalePlan() {
  constexpr float scale_x = static_cast<float>(PANEL_WIDTH) / STREAM_WIDTH;
  constexpr float scale_y = static_cast<float>(PANEL_HEIGHT) / STREAM_HEIGHT;
  constexpr float base_scale = scale_x < scale_y ? scale_x : scale_y;

  constexpr bool panel_is_portrait = PANEL_HEIGHT > PANEL_WIDTH;
  constexpr bool stream_is_landscape = STREAM_WIDTH > STREAM_HEIGHT;

  if (panel_is_portrait && stream_is_landscape) {
    constexpr float rot_scale_x =
        static_cast<float>(PANEL_WIDTH) / STREAM_HEIGHT;
    constexpr float rot_scale_y =
        static_cast<float>(PANEL_HEIGHT) / STREAM_WIDTH;
    float scale = rot_scale_x < rot_scale_y ? rot_scale_x : rot_scale_y;
    return {PPA_SRM_ROTATION_ANGLE_90, scale};
  }

  return {PPA_SRM_ROTATION_ANGLE_0, base_scale};
}

void renderTask(void *pvParameters) {
  auto *ctx = static_cast<PipelineContext *>(pvParameters);

  Serial.println("RenderTask: Starting...");

  // Initialize overlay renderer with framebuffer address
  OverlayRenderer::init(ctx->framebuffer());

  Serial.println("RenderTask: Initialized");

  uint32_t frame_count = 0;
  uint32_t last_fps_time = millis();
  DecodedFrameData dfd;

  while (1) {
    if (xQueueReceive(ctx->decodedFrameQueue(), &dfd, pdMS_TO_TICKS(1000)) ==
        pdTRUE) {
      if (dfd.has_linear_buf) {
        ctx->releaseLinear(dfd.linear_buf);
      }

      if (xSemaphoreTake(ctx->displayDoneSema(), pdMS_TO_TICKS(500)) ==
          pdTRUE) {
        ScalePlan plan = computeScalePlan();
        float scale = plan.scale;

        if (scale > 4.0f) {
          scale = 4.0f;
        } else if (scale < 0.125f) {
          scale = 0.125f;
        }

        // Output to video area only (skip top overlay bar)
        uint16_t *videoBuffer = ctx->framebuffer() + VIDEO_Y_OFFSET * PANEL_WIDTH;

        bool ppa_ok = PPAPipeline::transform(
            reinterpret_cast<const uint8_t *>(
                ctx->decodeBuffer(dfd.buf_idx)),
            reinterpret_cast<uint8_t *>(videoBuffer), STREAM_WIDTH,
            STREAM_HEIGHT, PANEL_WIDTH, VIDEO_HEIGHT,
            PPA_SRM_COLOR_MODE_RGB565, PPA_SRM_COLOR_MODE_RGB565, plan.rotation,
            scale, scale, ctx->ppaDoneSema());

        if (ppa_ok) {
          if (xSemaphoreTake(ctx->ppaDoneSema(), pdMS_TO_TICKS(100)) == pdTRUE) {
            // Render overlays (handles its own cache sync)
            OverlayRenderer::render(ctx->detectionData());

            esp_lcd_panel_draw_bitmap(ctx->panelHandle(), 0, 0, PANEL_WIDTH,
                                      PANEL_HEIGHT, ctx->framebuffer());
          } else {
            Serial.println("Render: PPA timeout");
            xSemaphoreGive(ctx->displayDoneSema());
          }
        } else {
          xSemaphoreGive(ctx->displayDoneSema());
        }

        frame_count++;
        if (frame_count >= 100) {
          uint32_t now = millis();
          Serial.printf("Render FPS: %.1f\n",
                        100.0f * 1000.0f / (now - last_fps_time));
          last_fps_time = now;
          frame_count = 0;
        }
      } else {
        xSemaphoreGive(ctx->displayDoneSema());
      }
    } else {
      vTaskDelay(1);
    }
  }
}
} // namespace

void RenderTask::start(PipelineContext &ctx, UBaseType_t priority,
                       BaseType_t core) {
  xTaskCreatePinnedToCore(renderTask, "Render", STACK_DEPTH, &ctx, priority,
                          nullptr, core);
}
