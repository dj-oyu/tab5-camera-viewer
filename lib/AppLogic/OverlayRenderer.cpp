#include "OverlayRenderer.h"
#include "DetectionData.h"
#include "PipelineConfig.h"
#include <M5GFX.h>
#include <cstring>

namespace {

// Cached detection data for display
Detection cachedDetections[MAX_DETECTIONS];
int cachedCount = 0;
uint32_t cachedTimestamp = 0;
uint32_t lastUpdateTime = 0;

bool initialized = false;

// Frame counter for periodic updates
uint32_t frameCounter = 0;

} // namespace

void OverlayRenderer::init() {
  if (initialized)
    return;

  initialized = true;
  Serial.println("OverlayRenderer initialized");
}

void OverlayRenderer::renderLeftBar(uint16_t *framebuffer, DetectionData &data) {
  // TODO: Render detection stats overlay on left bar (top 160 rows in framebuffer)
  // Note: Direct PSRAM writes are slow - consider using DMA or internal SRAM sprite
  (void)framebuffer;
  (void)data;
}

void OverlayRenderer::renderRightBar(uint16_t *framebuffer, DetectionData &data) {
  // TODO: Render detection list overlay on right bar (bottom 160 rows in framebuffer)
  // Note: Direct PSRAM writes are slow - consider using DMA or internal SRAM sprite
  (void)framebuffer;
  (void)data;
}
