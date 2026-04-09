#include "RenderPipelineTask.h"
#include "OverlayRenderer.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <PPAPipeline.h>
#include <Arduino.h>
#include <driver/jpeg_decode.h>
#include <esp_cache.h>
#include <esp_log.h>
#include <esp_timer.h>

namespace
{
  // --- From DecodeTask: JPEG validation ---
  bool hasEoiMarker(const uint8_t *buf, size_t len)
  {
    if (len < 2) return false;
    size_t search_start = (len > 16) ? len - 16 : 0;
    for (size_t i = search_start; i < len - 1; i++)
    {
      if (buf[i] == 0xFF && buf[i + 1] == 0xD9) return true;
    }
    return false;
  }

  // --- From RenderTask: PPA scale planning ---
  struct ScalePlan
  {
    ppa_srm_rotation_angle_t rotation;
    float scale;
  };

  ScalePlan computeScalePlan()
  {
    constexpr uint32_t VIEW_W = PANEL_WIDTH;
    constexpr uint32_t VIEW_H = VIDEO_HEIGHT;
    constexpr bool panel_is_portrait = PANEL_HEIGHT > PANEL_WIDTH;
    constexpr bool stream_is_landscape = STREAM_WIDTH > STREAM_HEIGHT;

    if (panel_is_portrait && stream_is_landscape)
    {
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

  struct PipelinePerfStats
  {
    uint32_t frames = 0;
    uint32_t decode_errors = 0;
    uint32_t truncated = 0;
    uint32_t ppa_timeouts = 0;
    uint64_t decode_us = 0;
    uint64_t ppa_us = 0;
    uint64_t overlay_us = 0;
  };

  void renderPipelineTask(void *pvParameters)
  {
    auto *ctx = static_cast<PipelineContext *>(pvParameters);
    auto *panel_fb = ctx->panelFramebuffer();

    Serial.println("RenderPipeline: Starting...");

    if (!panel_fb)
    {
      Serial.println("RenderPipeline: FATAL - panel framebuffer is null!");
      vTaskDelete(nullptr);
      return;
    }

    // --- JPEG decoder init (from DecodeTask) ---
    esp_log_level_set("jpeg.decoder", ESP_LOG_WARN);
    jpeg_decoder_handle_t decoder = nullptr;
    jpeg_decode_engine_cfg_t eng_cfg = {.intr_priority = 0, .timeout_ms = 100};
    jpeg_decode_cfg_t dec_cfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };
    if (jpeg_new_decoder_engine(&eng_cfg, &decoder) != ESP_OK || !decoder)
    {
      Serial.println("RenderPipeline: Failed to create JPEG decoder");
      vTaskDelete(nullptr);
      return;
    }

    // --- Overlay init ---
    OverlayRenderer::init();

    // --- PPA setup ---
    ScalePlan plan = computeScalePlan();
    if (plan.scale > 4.0f) plan.scale = 4.0f;
    else if (plan.scale < 0.125f) plan.scale = 0.125f;

    ppa_srm_oper_config_t ppa_config = {};
    PPAPipeline::prepareConfig(
        ppa_config, STREAM_WIDTH, STREAM_HEIGHT, PANEL_WIDTH, VIDEO_HEIGHT,
        PPA_SRM_COLOR_MODE_RGB565, PPA_SRM_COLOR_MODE_RGB565, plan.rotation,
        plan.scale, plan.scale);

    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    uint8_t *decode_buf = ctx->decodeBuf();
    uint16_t *videoBuffer = panel_fb + VIDEO_Y_OFFSET * PANEL_WIDTH;

    Serial.println("RenderPipeline: Initialized (SPSC ring, JPEG→PPA→Overlay)");

    PipelinePerfStats perf;
    uint32_t perf_window_start = millis();
    float current_render_fps = -1.0f;

    auto flushPerf = [&](uint32_t now)
    {
      uint32_t window_ms = now - perf_window_start;
      if (window_ms == 0) window_ms = 1;
      uint64_t frame_div = perf.frames > 0 ? perf.frames : 1;
      float window_fps = (1000.0f * perf.frames) / window_ms;
      current_render_fps = window_fps;
      ctx->updateRenderFps(window_fps, now);
      Serial.printf(
          "Decode Perf: fps=%.1f decode=%lluus errors=%u truncated=%u "
          "queues(in=%u out=0)\n",
          window_fps,
          static_cast<unsigned long long>(perf.decode_us / frame_div),
          perf.decode_errors, perf.truncated,
          ctx->ringWrite().load(std::memory_order_relaxed) -
              ctx->ringRead().load(std::memory_order_relaxed));
      if (VERBOSE_PERF_LOG)
      {
        Serial.printf(
            "Render Perf: ppa=%lluus overlay=%lluus ppa_to=%u\n",
            static_cast<unsigned long long>(perf.ppa_us / frame_div),
            static_cast<unsigned long long>(perf.overlay_us / frame_div),
            perf.ppa_timeouts);
      }
      perf = {};
      perf_window_start = now;
    };

    while (1)
    {
      // --- Check SPSC ring for available frames ---
      uint32_t r = ctx->ringRead().load(std::memory_order_relaxed);
      uint32_t w = ctx->ringWrite().load(std::memory_order_acquire);

      if (r >= w)
      {
        // Ring empty — wait for Fetch to produce a frame
        xSemaphoreTake(ctx->frameReadySema(), pdMS_TO_TICKS(1000));

        // Render overlay even when idle (battery, WiFi, VPN status)
        OverlayRenderer::render(ctx->detectionData(), ctx->connectionData(),
                                ctx->recordingData(), ctx->batteryData(),
                                panel_fb, current_render_fps);

        uint32_t now = millis();
        if ((now - perf_window_start) >= PERF_LOG_INTERVAL_MS)
        {
          if (perf.frames == 0 && perf.decode_errors == 0 && perf.truncated == 0)
          {
            Serial.printf("Decode IDLE: recv=0 ok=0 err=0 trunc=0 queues(in=%u out=0)\n",
                          w - r);
            perf_window_start = now;
          }
          else
          {
            flushPerf(now);
          }
        }
        continue;
      }

      FrameSlot *slot = ctx->ringSlot(r);

      // --- JPEG validation (from DecodeTask) ---
      if (slot->len < 4 || slot->buf[0] != 0xFF || slot->buf[1] != 0xD8)
      {
        perf.decode_errors++;
        ctx->ringRead().store(r + 1, std::memory_order_release);
        xTaskNotifyGive(ctx->fetchTaskHandle());
        continue;
      }

      if (!hasEoiMarker(slot->buf, slot->len))
      {
        perf.truncated++;
        ctx->ringRead().store(r + 1, std::memory_order_release);
        xTaskNotifyGive(ctx->fetchTaskHandle());
        continue;
      }

      // --- JPEG HW Decode ---
      uint32_t out_size = 0;
      int64_t decode_start = esp_timer_get_time();
      esp_err_t ret = jpeg_decoder_process(
          decoder, &dec_cfg, slot->buf, slot->aligned_len,
          decode_buf, DECODE_BUF_SIZE, &out_size);
      perf.decode_us +=
          static_cast<uint64_t>(esp_timer_get_time() - decode_start);

      // Release ring slot ASAP (Fetch can reuse buffer immediately)
      ctx->ringRead().store(r + 1, std::memory_order_release);
      xTaskNotifyGive(ctx->fetchTaskHandle());

      if (ret != ESP_OK || out_size == 0)
      {
        perf.decode_errors++;
        if (perf.decode_errors <= 3)
        {
          Serial.printf("Decode: jpeg failed err=%d(%s) out=%u\n",
                        static_cast<int>(ret), esp_err_to_name(ret), out_size);
        }
        continue;
      }

      // --- PPA Scale/Rotate → Framebuffer ---
      int64_t ppa_start = esp_timer_get_time();

      bool ppa_ok = PPAPipeline::submit(
          ppa_config, decode_buf,
          reinterpret_cast<uint8_t *>(videoBuffer), self);

      if (!ppa_ok || !ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(120)))
      {
        perf.ppa_timeouts++;
        uint32_t now = millis();
        if (perf.frames >= PERF_LOG_WINDOW_FRAMES ||
            (now - perf_window_start) >= PERF_LOG_INTERVAL_MS)
        {
          flushPerf(now);
        }
        continue;
      }

      perf.ppa_us +=
          static_cast<uint64_t>(esp_timer_get_time() - ppa_start);

      // --- Overlay ---
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
        flushPerf(now);
      }
    }
  }
} // namespace

void RenderPipelineTask::start(PipelineContext &ctx, UBaseType_t priority,
                               BaseType_t core)
{
  TaskHandle_t handle = nullptr;
  BaseType_t ret = xTaskCreatePinnedToCore(renderPipelineTask, "RenderPL",
                          STACK_DEPTH_RENDER, &ctx, priority, &handle, core);
  if (ret != pdPASS || !handle) {
    Serial.println("FATAL: Failed to create RenderPipeline task!");
    return;
  }
  ctx.setRenderPipelineTaskHandle(handle);
}
