#include "RenderTask.h"
#include "OverlayRenderer.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <PPAPipeline.h>
#include <Arduino.h>
#include <esp_timer.h>

namespace {
struct ScalePlan {
  ppa_srm_rotation_angle_t rotation;
  float scale;
};

struct RenderPerfStats {
  uint32_t frames = 0;
  uint32_t render_buf_drops = 0;
  uint32_t ppa_timeouts = 0;
  uint32_t display_timeouts = 0;
  uint64_t render_buf_wait_us = 0;
  uint64_t ppa_us = 0;
  uint64_t overlay_us = 0;
  uint64_t display_wait_us = 0;
  uint64_t submit_us = 0;
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

  OverlayRenderer::init();
  ScalePlan plan = computeScalePlan();
  if (plan.scale > 4.0f) {
    plan.scale = 4.0f;
  } else if (plan.scale < 0.125f) {
    plan.scale = 0.125f;
  }

  Serial.println("RenderTask: Initialized");

  // displayDoneSema is primed in PipelineContext::init().
  // Drain that bootstrap token so subsequent takes correspond to actual submits.
  xSemaphoreTake(ctx->displayDoneSema(), 0);

  RenderPerfStats perf;
  uint32_t perf_window_start = millis();

  uint16_t *inflight_buf = nullptr;
  DecodedFrameData dfd;

  auto flushRenderPerf = [&](uint32_t now) {
    uint32_t window_ms = now - perf_window_start;
    if (window_ms == 0) {
      window_ms = 1;
    }
    uint64_t frame_div = perf.frames > 0 ? perf.frames : 1;
    float window_fps = (1000.0f * perf.frames) / window_ms;
    ctx->updateRenderFps(window_fps, now);
    Serial.printf("Render FPS: %.1f\n", window_fps);
    Serial.printf(
        "Render Perf: fps=%.1f buf_wait=%lluus ppa=%lluus overlay=%lluus "
        "disp_wait=%lluus submit=%lluus drops=%u ppa_to=%u disp_to=%u "
        "queues(decoded=%u render_free=%u)\n",
        window_fps,
        static_cast<unsigned long long>(perf.render_buf_wait_us / frame_div),
        static_cast<unsigned long long>(perf.ppa_us / frame_div),
        static_cast<unsigned long long>(perf.overlay_us / frame_div),
        static_cast<unsigned long long>(perf.display_wait_us / frame_div),
        static_cast<unsigned long long>(perf.submit_us / frame_div),
        perf.render_buf_drops, perf.ppa_timeouts, perf.display_timeouts,
        uxQueueMessagesWaiting(ctx->decodedFrameQueue()),
        uxQueueMessagesWaiting(ctx->renderFreeQueue()));
    perf = {};
    perf_window_start = now;
  };

  while (1) {
    if (xQueueReceive(ctx->decodedFrameQueue(), &dfd, pdMS_TO_TICKS(1000)) ==
        pdTRUE) {
      if (dfd.has_linear_buf) {
        ctx->releaseLinear(dfd.linear_buf);
      }

      uint16_t *render_buf = nullptr;
      int64_t render_buf_wait_start = esp_timer_get_time();
      while (!ctx->acquireRenderBuffer(&render_buf, pdMS_TO_TICKS(1))) {
        if (!inflight_buf) {
          break;
        }

        int64_t wait_start = esp_timer_get_time();
        if (xSemaphoreTake(ctx->displayDoneSema(), pdMS_TO_TICKS(120)) !=
            pdTRUE) {
          perf.display_timeouts++;
          break;
        }
        perf.display_wait_us +=
            static_cast<uint64_t>(esp_timer_get_time() - wait_start);
        ctx->releaseRenderBuffer(inflight_buf);
        inflight_buf = nullptr;
      }
      perf.render_buf_wait_us +=
          static_cast<uint64_t>(esp_timer_get_time() - render_buf_wait_start);

      if (!render_buf) {
        perf.render_buf_drops++;
        continue;
      }

      // Output to video area only (skip top/bottom overlay bars).
      uint16_t *videoBuffer = render_buf + VIDEO_Y_OFFSET * PANEL_WIDTH;

      int64_t ppa_start = esp_timer_get_time();
      bool ppa_ok = PPAPipeline::transform(
          reinterpret_cast<const uint8_t *>(ctx->decodeBuffer(dfd.buf_idx)),
          reinterpret_cast<uint8_t *>(videoBuffer), STREAM_WIDTH, STREAM_HEIGHT,
          PANEL_WIDTH, VIDEO_HEIGHT, PPA_SRM_COLOR_MODE_RGB565,
          PPA_SRM_COLOR_MODE_RGB565, plan.rotation, plan.scale, plan.scale,
          ctx->ppaDoneSema());

      if (!ppa_ok ||
          xSemaphoreTake(ctx->ppaDoneSema(), pdMS_TO_TICKS(120)) != pdTRUE) {
        perf.ppa_timeouts++;
        ctx->releaseRenderBuffer(render_buf);
        continue;
      }
      perf.ppa_us += static_cast<uint64_t>(esp_timer_get_time() - ppa_start);

      int64_t overlay_start = esp_timer_get_time();
      OverlayRenderer::render(ctx->detectionData(), ctx->connectionData(),
                              ctx->recordingData(), render_buf);
      perf.overlay_us +=
          static_cast<uint64_t>(esp_timer_get_time() - overlay_start);

      if (inflight_buf) {
        int64_t wait_start = esp_timer_get_time();
        if (xSemaphoreTake(ctx->displayDoneSema(), pdMS_TO_TICKS(120)) !=
            pdTRUE) {
          perf.display_timeouts++;
          ctx->releaseRenderBuffer(render_buf);
          continue;
        }
        perf.display_wait_us +=
            static_cast<uint64_t>(esp_timer_get_time() - wait_start);
        ctx->releaseRenderBuffer(inflight_buf);
        inflight_buf = nullptr;
      }

      int64_t submit_start = esp_timer_get_time();
      esp_err_t draw_ret = esp_lcd_panel_draw_bitmap(
          ctx->panelHandle(), 0, 0, PANEL_WIDTH, PANEL_HEIGHT, render_buf);
      perf.submit_us +=
          static_cast<uint64_t>(esp_timer_get_time() - submit_start);

      if (draw_ret != ESP_OK) {
        Serial.printf("Render: draw_bitmap failed (%d)\n", static_cast<int>(draw_ret));
        ctx->releaseRenderBuffer(render_buf);
        continue;
      }

      inflight_buf = render_buf;
      perf.frames++;

      uint32_t now = millis();
      if (perf.frames >= PERF_LOG_WINDOW_FRAMES ||
          (now - perf_window_start) >= PERF_LOG_INTERVAL_MS) {
        flushRenderPerf(now);
      }
    } else {
      uint32_t now = millis();
      if ((now - perf_window_start) >= PERF_LOG_INTERVAL_MS) {
        flushRenderPerf(now);
      }
      if (inflight_buf &&
          xSemaphoreTake(ctx->displayDoneSema(), 0) == pdTRUE) {
        ctx->releaseRenderBuffer(inflight_buf);
        inflight_buf = nullptr;
      }
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
