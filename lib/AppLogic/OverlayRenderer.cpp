#include "OverlayRenderer.h"
#include "DetectionData.h"
#include "PipelineConfig.h"
#include <M5GFX.h>

namespace {

constexpr int SPRITE_W = 160;
constexpr int SPRITE_H = 200;
LGFX_Sprite textSprite;

bool initialized = false;

} // namespace

void OverlayRenderer::init() {
  if (initialized)
    return;

  // Test: Create sprite in internal SRAM
  textSprite.setPsram(false);
  textSprite.setColorDepth(16);
  if (textSprite.createSprite(SPRITE_W, SPRITE_H)) {
    Serial.printf("OverlayRenderer: Sprite %dx%d created\n", SPRITE_W, SPRITE_H);
  } else {
    Serial.println("OverlayRenderer: Sprite creation failed!");
  }

  initialized = true;
  Serial.println("OverlayRenderer initialized");
}

void OverlayRenderer::renderLeftBar(uint16_t *framebuffer, DetectionData &data) {
  (void)framebuffer;
  (void)data;
}

void OverlayRenderer::renderRightBar(uint16_t *framebuffer, DetectionData &data) {
  (void)framebuffer;
  (void)data;
}
