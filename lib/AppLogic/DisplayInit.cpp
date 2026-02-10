#include "DisplayInit.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>

namespace {
class Panel_DSI_Accessor : public lgfx::Panel_DSI {
public:
  esp_lcd_panel_handle_t getHandle() { return _disp_panel_handle; }
};
} // namespace

bool DisplayInit::init(PipelineContext &ctx) {
  auto dsi = static_cast<lgfx::Panel_DSI *>(M5.Display.getPanel());
  auto accessor = static_cast<Panel_DSI_Accessor *>(dsi);
  ctx.setPanelHandle(accessor->getHandle());

  uint16_t *panel_fb = nullptr;
  esp_lcd_dpi_panel_get_frame_buffer(ctx.panelHandle(), 1,
                                     reinterpret_cast<void **>(&panel_fb));
  ctx.setPanelFramebuffer(panel_fb);
  Serial.printf("Panel framebuffer: %p (direct PPA target)\n", panel_fb);

  return true;
}
