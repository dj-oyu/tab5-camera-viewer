#include "OverlayRenderer.h"
#include "DetectionData.h"
#include "PipelineConfig.h"
#include <M5GFX.h>
#include <cstring>
#include <esp_cache.h>

namespace {

// Pre-calculated addresses (set once at init)
uint16_t *topBar = nullptr;     // Camera side (rows 0-159)
uint16_t *bottomBar = nullptr;  // USB side (rows 1120-1279)
bool updateTop = true;          // Toggle: true=top, false=bottom

// BGR565 format
constexpr uint8_t FILL_GRAY = 0x42;
constexpr uint8_t FILL_WHITE = 0xFF;

constexpr size_t BAR_BYTES = PANEL_WIDTH * OVERLAY_BAR_SIZE * sizeof(uint16_t);

} // namespace

void OverlayRenderer::init(uint16_t *framebuffer) {
  if (topBar != nullptr)
    return;

  topBar = framebuffer;
  bottomBar = framebuffer + (PANEL_HEIGHT - OVERLAY_BAR_SIZE) * PANEL_WIDTH;

  Serial.printf("OverlayRenderer: top=%p, bottom=%p\n", topBar, bottomBar);
}

void OverlayRenderer::render(DetectionData &data) {
  (void)data;

  if (updateTop) {
    memset(topBar, FILL_GRAY, BAR_BYTES);
    esp_cache_msync(topBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  } else {
    memset(bottomBar, FILL_WHITE, BAR_BYTES);
    esp_cache_msync(bottomBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  }
  updateTop = !updateTop;
}
