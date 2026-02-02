#include "OverlayRenderer.h"
#include "DetectionData.h"
#include "PipelineConfig.h"
#include <Arduino.h>
#include <M5GFX.h>
#include <cstring>
#include <esp_cache.h>
#include <esp_heap_caps.h>

namespace {

// Pre-calculated framebuffer addresses
uint16_t *topBar = nullptr;     // Camera side (rows 0-159)
uint16_t *bottomBar = nullptr;  // USB side (rows 1120-1279)

// Tile-based sprite in internal SRAM for rotated text rendering
// 160 (bar height) x 240 (tile width) = 76.8KB (fits in internal SRAM)
// 3 tiles cover the full 720 pixel width
constexpr int TILE_HEIGHT = OVERLAY_BAR_SIZE;  // 160
constexpr int TILE_WIDTH = 240;
constexpr int NUM_TILES = 3;  // 240 * 3 = 720
LGFX_Sprite tileSprite;

bool initialized = false;

constexpr size_t BAR_BYTES = PANEL_WIDTH * OVERLAY_BAR_SIZE * sizeof(uint16_t);

} // namespace

void OverlayRenderer::init(uint16_t *framebuffer) {
  if (topBar != nullptr)
    return;

  topBar = framebuffer;
  bottomBar = framebuffer + (PANEL_HEIGHT - OVERLAY_BAR_SIZE) * PANEL_WIDTH;

  // Create tile sprite in internal SRAM
  // Sprite dimensions: TILE_WIDTH x TILE_HEIGHT (240 x 160)
  // With setRotation(3), drawing coordinates become 160 x 240 (landscape text)
  tileSprite.setPsram(false);  // Force internal SRAM
  tileSprite.setColorDepth(16);
  if (!tileSprite.createSprite(TILE_WIDTH, TILE_HEIGHT)) {
    Serial.println("OverlayRenderer: tileSprite FAILED");
    return;
  }
  tileSprite.setRotation(3);  // 270° for landscape text (reads bottom-to-top in framebuffer)
  tileSprite.setTextSize(2);
  Serial.printf("OverlayRenderer: tileSprite OK (%dx%d, rot=3, internal SRAM)\n",
                TILE_WIDTH, TILE_HEIGHT);

  // Clear bars initially
  memset(topBar, 0x00, BAR_BYTES);
  esp_cache_msync(topBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  memset(bottomBar, 0x00, BAR_BYTES);
  esp_cache_msync(bottomBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  initialized = true;
  Serial.printf("OverlayRenderer: top=%p, bottom=%p, initialized\n",
                topBar, bottomBar);
}

// Copy tile sprite to framebuffer bar at specified X offset
void copyTileToBar(uint16_t *bar, int xOffset) {
  uint16_t *src = (uint16_t *)tileSprite.getBuffer();
  for (int y = 0; y < TILE_HEIGHT; y++) {
    memcpy(bar + y * PANEL_WIDTH + xOffset,
           src + y * TILE_WIDTH,
           TILE_WIDTH * sizeof(uint16_t));
  }
}

void OverlayRenderer::render(DetectionData &data) {
  if (!initialized)
    return;

  // Check for detection data update
  static Detection cachedDetections[MAX_DETECTIONS];
  static int cachedCount = 0;
  static uint32_t cachedTimestamp = 0;
  static bool needsRender = true;  // Initial render needed

  Detection detections[MAX_DETECTIONS];
  int count = 0;
  uint32_t timestamp = 0;
  if (data.tryRead(detections, &count, &timestamp)) {
    memcpy(cachedDetections, detections, sizeof(Detection) * count);
    cachedCount = count;
    cachedTimestamp = timestamp;
    needsRender = true;  // New data arrived
  }

  // Only render when new detection data arrives
  if (!needsRender) {
    return;
  }

  static uint32_t lastRender = 0;
  static bool renderTopBar = true;  // Alternate between bars
  uint32_t now = millis();

  // Rate limit to 500ms minimum between renders
  if ((now - lastRender) < 500) {
    return;
  }
  lastRender = now;
  needsRender = false;  // Mark as rendered

  if (renderTopBar) {
    // === TOP BAR (camera side, right in landscape) ===
    memset(topBar, 0x00, BAR_BYTES);

    // Tile 0: Title
    tileSprite.fillScreen(TFT_BLACK);
    tileSprite.setTextColor(TFT_WHITE);
    tileSprite.setCursor(10, 20);
    tileSprite.printf("Det: %d", cachedCount);
    copyTileToBar(topBar, 0);

    // Tile 1: Status
    tileSprite.fillScreen(TFT_BLACK);
    tileSprite.setTextColor(TFT_GREEN);
    tileSprite.setCursor(10, 20);
    tileSprite.print("OK");
    copyTileToBar(topBar, TILE_WIDTH);

    // Tile 2: Timestamp
    tileSprite.fillScreen(TFT_BLACK);
    tileSprite.setTextColor(TFT_CYAN);
    tileSprite.setCursor(10, 20);
    tileSprite.printf("T:%lu", cachedTimestamp / 1000);
    copyTileToBar(topBar, TILE_WIDTH * 2);

    esp_cache_msync(topBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  } else {
    // === BOTTOM BAR (USB side, left in landscape) ===
    memset(bottomBar, 0x00, BAR_BYTES);

    // Display detection list (or empty tiles)
    for (int i = 0; i < NUM_TILES; i++) {
      tileSprite.fillScreen(TFT_BLACK);
      if (i < cachedCount) {
        tileSprite.setTextColor(TFT_YELLOW);
        tileSprite.setCursor(10, 20);
        tileSprite.printf("%s", cachedDetections[i].label);
        tileSprite.setCursor(10, 60);
        tileSprite.setTextColor(TFT_WHITE);
        tileSprite.printf("%.0f%%", cachedDetections[i].confidence * 100);
      }
      copyTileToBar(bottomBar, TILE_WIDTH * i);
    }

    esp_cache_msync(bottomBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  }

  renderTopBar = !renderTopBar;
}
