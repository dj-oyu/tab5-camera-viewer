#include "OverlayRenderer.h"
#include "DetectionData.h"
#include "PipelineConfig.h"
#include <Arduino.h>
#include <M5GFX.h>
#include <cstring>
#include <esp_cache.h>

namespace {

// Pre-calculated framebuffer addresses
uint16_t *topBar = nullptr;     // Camera side (rows 0-159)
uint16_t *bottomBar = nullptr;  // USB side (rows 1120-1279)

// Single sprite in internal SRAM (only one 720x160 fits)
// Pre-rendered content at init time
LGFX_Sprite sprite;
bool spriteReady = false;

constexpr size_t BAR_BYTES = PANEL_WIDTH * OVERLAY_BAR_SIZE * sizeof(uint16_t);

} // namespace

void OverlayRenderer::init(uint16_t *framebuffer) {
  if (topBar != nullptr)
    return;

  topBar = framebuffer;
  bottomBar = framebuffer + (PANEL_HEIGHT - OVERLAY_BAR_SIZE) * PANEL_WIDTH;

  // Create sprite in internal SRAM
  sprite.setPsram(false);
  sprite.setColorDepth(16);
  if (!sprite.createSprite(PANEL_WIDTH, OVERLAY_BAR_SIZE)) {
    Serial.println("OverlayRenderer: sprite FAILED");
    return;
  }

  // Pre-render top bar content
  sprite.fillScreen(TFT_RED);
  sprite.fillRect(100, 20, 200, 100, TFT_WHITE);

  // Copy to top framebuffer directly (init time only)
  memcpy(topBar, sprite.getBuffer(), BAR_BYTES);
  esp_cache_msync(topBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  // Pre-render bottom bar content
  sprite.fillScreen(TFT_BLUE);
  sprite.fillRect(100, 20, 200, 100, TFT_YELLOW);

  // Copy to bottom framebuffer directly (init time only)
  memcpy(bottomBar, sprite.getBuffer(), BAR_BYTES);
  esp_cache_msync(bottomBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  spriteReady = true;
  Serial.printf("OverlayRenderer: top=%p, bottom=%p, sprite=OK\n",
                topBar, bottomBar);
}

void OverlayRenderer::render(DetectionData &data) {
  (void)data;
  // Static content - nothing to do each frame
  // Content was pre-rendered in init()
}
