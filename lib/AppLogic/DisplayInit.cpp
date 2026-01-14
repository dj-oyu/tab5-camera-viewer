#include "DisplayInit.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <esp_heap_caps.h>
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>

// Forward declaration for DPI panel event callbacks
extern "C" esp_err_t esp_lcd_dpi_panel_register_event_callbacks(
    esp_lcd_panel_handle_t panel,
    const esp_lcd_dpi_panel_event_callbacks_t *cbs, void *user_ctx);

namespace {
PipelineContext *s_ctx = nullptr;

class Panel_DSI_Accessor : public lgfx::Panel_DSI {
public:
  esp_lcd_panel_handle_t getHandle() { return _disp_panel_handle; }
};

bool IRAM_ATTR on_color_trans_done(esp_lcd_panel_handle_t panel,
                                   esp_lcd_dpi_panel_event_data_t *edata,
                                   void *user_ctx) {
  BaseType_t high_priority_task_awoken = pdFALSE;
  if (s_ctx && s_ctx->displayDoneSema()) {
    xSemaphoreGiveFromISR(s_ctx->displayDoneSema(),
                          &high_priority_task_awoken);
  }
  return high_priority_task_awoken == pdTRUE;
}
} // namespace

bool DisplayInit::init(PipelineContext &ctx) {
  s_ctx = &ctx;

  auto dsi = static_cast<lgfx::Panel_DSI *>(M5.Display.getPanel());
  auto accessor = static_cast<Panel_DSI_Accessor *>(dsi);
  ctx.setPanelHandle(accessor->getHandle());

  uint16_t *fb = nullptr;
  esp_lcd_dpi_panel_get_frame_buffer(ctx.panelHandle(), 1,
                                     reinterpret_cast<void **>(&fb));

  if (!fb) {
    Serial.println("Failed to get framebuffer, allocating dedicated buffer");
    fb = static_cast<uint16_t *>(heap_caps_aligned_alloc(
        64, PANEL_WIDTH * PANEL_HEIGHT * 2,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  }
  ctx.setFramebuffer(fb);
  Serial.printf("Framebuffer: %p\n", fb);

  esp_lcd_dpi_panel_event_callbacks_t cbs = {
      .on_color_trans_done = on_color_trans_done,
  };
  esp_lcd_dpi_panel_register_event_callbacks(ctx.panelHandle(), &cbs, nullptr);
  Serial.println("DSI transfer callback registered");

  return true;
}
