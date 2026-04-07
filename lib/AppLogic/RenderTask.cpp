#include "RenderTask.h"
#include "OverlayRenderer.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <PPAPipeline.h>
#include <Arduino.h>
#include <esp_timer.h>

namespace
{
  struct ScalePlan
  {
    ppa_srm_rotation_angle_t rotation;
    float scale;
  };

  ScalePlan computeScalePlan()
  {
    // Video area dimensions (excludes overlay bars)
    constexpr uint32_t VIEW_W = PANEL_WIDTH;
    constexpr uint32_t VIEW_H = VIDEO_HEIGHT;

    constexpr bool panel_is_portrait = PANEL_HEIGHT > PANEL_WIDTH;
    constexpr bool stream_is_landscape = STREAM_WIDTH > STREAM_HEIGHT;

    if (panel_is_portrait && stream_is_landscape)
    {
      // Rotate 90° then fit into video area
      constexpr float rot_scale_x =
          static_cast<float>(VIEW_W) / STREAM_HEIGHT;
      constexpr float rot_scale_y =
          static_cast<float>(VIEW_H) / STREAM_WIDTH;
      float scale = rot_scale_x < rot_scale_y ? rot_scale_x : rot_scale_y;
      return {PPA_SRM_ROTATION_ANGLE_90, scale};
    }

    constexpr float scale_x = static_cast<float>(VIEW_W) / STREAM_WIDTH;
    constexpr float scale_y = static_cast<float>(VIEW_H) / STREAM_HEIGHT;
    constexpr float base_scale = scale_x < scale_y ? scale_x : scale_y;
    return {PPA_SRM_ROTATION_ANGLE_0, base_scale};
  }

  struct RenderPerfStats
  {
    uint32_t frames = 0;
    uint32_t ppa_timeouts = 0;
    uint64_t ppa_us = 0;
    uint64_t overlay_us = 0;
  };

  void renderTask(void *pvParameters)
  {
    auto *ctx = static_cast<PipelineContext *>(pvParameters);
    auto dma2d_gate = ctx->dma2dGate();
    auto *panel_fb = ctx->panelFramebuffer();

    Serial.println("RenderTask: Starting...");

    if (!panel_fb)
    {
      Serial.println("RenderTask: FATAL - panel framebuffer is null!");
      vTaskDelete(nullptr);
      return;
    }

    OverlayRenderer::init();

    // PPA setup
    ScalePlan plan = computeScalePlan();
    if (plan.scale > 4.0f)
    {
      plan.scale = 4.0f;
    }
    else if (plan.scale < 0.125f)
    {
      plan.scale = 0.125f;
    }

    ppa_srm_oper_config_t ppa_config = {};
    PPAPipeline::prepareConfig(
        ppa_config, STREAM_WIDTH, STREAM_HEIGHT, PANEL_WIDTH, VIDEO_HEIGHT,
        PPA_SRM_COLOR_MODE_RGB565, PPA_SRM_COLOR_MODE_RGB565, plan.rotation,
        plan.scale, plan.scale);

    TaskHandle_t self = xTaskGetCurrentTaskHandle();

    Serial.printf("RenderTask: Initialized (PPA→FB direct, overlay, gate=%u)\n",
                  DMA2D_GATE_COUNT);

    RenderPerfStats perf;
    uint32_t perf_window_start = millis();
    float current_render_fps = -1.0f;  // For overlay display

    // PPA writes to video area of panel framebuffer directly
    uint16_t *videoBuffer = panel_fb + VIDEO_Y_OFFSET * PANEL_WIDTH;

    auto flushRenderPerf = [&](uint32_t now)
    {
      uint32_t window_ms = now - perf_window_start;
      if (window_ms == 0)
      {
        window_ms = 1;
      }
      uint64_t frame_div = perf.frames > 0 ? perf.frames : 1;
      float window_fps = (1000.0f * perf.frames) / window_ms;
      current_render_fps = window_fps;
      ctx->updateRenderFps(window_fps, now);
      if (VERBOSE_PERF_LOG)
      {
        Serial.printf("Render FPS: %.1f\n", window_fps);
        Serial.printf(
            "Render Perf: fps=%.1f ppa=%lluus overlay=%lluus "
            "ppa_to=%u queues(decoded=%u)\n",
            window_fps,
            static_cast<unsigned long long>(perf.ppa_us / frame_div),
            static_cast<unsigned long long>(perf.overlay_us / frame_div),
            perf.ppa_timeouts,
            ctx->decodedFramesPending());
      }
      perf = {};
      perf_window_start = now;
    };

    while (1)
    {
      uint8_t *decoded_buf = ctx->waitDecodedFrame(pdMS_TO_TICKS(1000));
      if (decoded_buf)
      {
        // PPA: decoded_buf → panel FB video area (DMA2D)
        int64_t ppa_start = esp_timer_get_time();

        xSemaphoreTake(dma2d_gate, portMAX_DELAY);

        bool ppa_ok = PPAPipeline::submit(
            ppa_config, decoded_buf,
            reinterpret_cast<uint8_t *>(videoBuffer), self);

        if (!ppa_ok || !ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(120)))
        {
          xSemaphoreGive(dma2d_gate);
          ctx->releaseDecodedFrame();
          perf.ppa_timeouts++;

          uint32_t now = millis();
          if (perf.frames >= PERF_LOG_WINDOW_FRAMES ||
              (now - perf_window_start) >= PERF_LOG_INTERVAL_MS)
          {
            flushRenderPerf(now);
          }
          continue;
        }

        xSemaphoreGive(dma2d_gate);

        // Release decode buffer after PPA completes (DMA2D reads asynchronously)
        ctx->releaseDecodedFrame();

        perf.ppa_us +=
            static_cast<uint64_t>(esp_timer_get_time() - ppa_start);

        // Overlay (CPU writes to bar areas, does its own esp_cache_msync)
        int64_t overlay_start = esp_timer_get_time();
        OverlayRenderer::render(ctx->detectionData(), ctx->connectionData(),
                                ctx->recordingData(), ctx->batteryData(),
                                panel_fb, current_render_fps);
        perf.overlay_us +=
            static_cast<uint64_t>(esp_timer_get_time() - overlay_start);

        perf.frames++;

        uint32_t now = millis();
        if (perf.frames >= PERF_LOG_WINDOW_FRAMES ||
            (now - perf_window_start) >= PERF_LOG_INTERVAL_MS)
        {
          flushRenderPerf(now);
        }
      }
      else
      {
        // No decoded frame available — still render overlay so status
        // (battery, WiFi, VPN) is visible during startup / network wait
        OverlayRenderer::render(ctx->detectionData(), ctx->connectionData(),
                                ctx->recordingData(), ctx->batteryData(),
                                panel_fb, current_render_fps);

        uint32_t now = millis();
        if ((now - perf_window_start) >= PERF_LOG_INTERVAL_MS)
        {
          flushRenderPerf(now);
        }
        vTaskDelay(1);
      }
    }
  }
} // namespace

void RenderTask::start(PipelineContext &ctx, UBaseType_t priority,
                       BaseType_t core)
{
  TaskHandle_t handle = nullptr;
  xTaskCreatePinnedToCore(renderTask, "Render", STACK_DEPTH_RENDER, &ctx, priority,
                          &handle, core);
  ctx.setRenderTaskHandle(handle);
}
