#include "OverlayRenderer.h"
#include "ConnectionData.h"
#include "DetectionData.h"
#include "PipelineConfig.h"
#include <Arduino.h>
#include <M5GFX.h>
#include <cstring>
#include <esp_cache.h>
#include <esp_heap_caps.h>

namespace {

// Pre-calculated framebuffer addresses
uint16_t *topBar = nullptr;     // Camera side (rows 0-159) - Right in landscape
uint16_t *bottomBar = nullptr;  // USB side (rows 1120-1279) - Left in landscape

// Tile-based sprite in internal SRAM for rotated text rendering
// Bar: 160px wide × 720px long (in landscape)
// Tile: 160px wide × 240px tall = 76.8KB (fits in internal SRAM)
// 3 tiles cover the full 720 pixel length
constexpr int TILE_WIDTH = OVERLAY_BAR_SIZE;  // 160 (bar width)
constexpr int TILE_HEIGHT = 240;               // Each tile covers 240px of bar length
constexpr int NUM_TILES = 3;                   // 240 * 3 = 720
LGFX_Sprite tileSprite;

bool initialized = false;

constexpr size_t BAR_BYTES = PANEL_WIDTH * OVERLAY_BAR_SIZE * sizeof(uint16_t);

// Tile layout constants (matching ui-simulator README)
// padding_top + (row_height × 4) + (gap × 3) + padding_bottom = 240px
// 6px + (48px × 4) + (12px × 3) + 6px = 240px
constexpr int TILE_PADDING = 6;
constexpr int ROW_HEIGHT = 48;
constexpr int ROW_GAP = 12;
constexpr int ROWS_PER_TILE = 4;

// Colors (matching ui-simulator)
constexpr uint16_t BG_COLOR = 0x0841;        // Dark gray (#111111)
constexpr uint16_t METER_COLOR = 0x2444;     // Dark green (#228822)
constexpr uint16_t TEXT_COLOR = TFT_WHITE;
constexpr uint16_t CONN_COLOR = TFT_CYAN;

// Detection display timeout (clear display if no data for this long)
constexpr uint32_t DETECTION_DISPLAY_TIMEOUT_MS = 3000;

// State tracking for optimized rendering
int prevDetectionCount = 0;
uint32_t lastDetectionTime = 0;
int cachedConnectionTotal = 0;
int cachedConnectionWebrtc = 0;
int cachedConnectionMjpeg = 0;

// Calculate Y position for a row within a tile
int getRowY(int rowIndex) {
  return TILE_PADDING + rowIndex * (ROW_HEIGHT + ROW_GAP);
}

// Draw a detection row with confidence meter bar
void drawDetectionRow(int rowY, const char *label, float confidence) {
  // Draw confidence meter bar as background
  int meterWidth = (int)(TILE_WIDTH * confidence);
  tileSprite.fillRect(0, rowY, meterWidth, ROW_HEIGHT, METER_COLOR);

  // Draw label text (centered vertically in row)
  tileSprite.setTextColor(TEXT_COLOR);
  int textY = rowY + (ROW_HEIGHT - 16) / 2;  // 16 = approx text height at size 2
  tileSprite.setCursor(4, textY);
  tileSprite.print(label);
}

// Render a single detection tile (4 rows per tile)
void renderDetectionTile(int tileIndex, Detection *detections, int totalCount) {
  tileSprite.fillSprite(BG_COLOR);

  int startIdx = tileIndex * ROWS_PER_TILE;
  for (int i = 0; i < ROWS_PER_TILE; i++) {
    int detIdx = startIdx + i;
    if (detIdx < totalCount) {
      int rowY = getRowY(i);
      drawDetectionRow(rowY, detections[detIdx].label, detections[detIdx].confidence);
    }
  }
}

// Render connection count tile (Right bar, Tile 0 only)
void renderConnectionTile() {
  tileSprite.fillSprite(BG_COLOR);
  tileSprite.setTextColor(CONN_COLOR);

  // Display connection counts
  int y = TILE_PADDING;
  tileSprite.setCursor(4, y);
  tileSprite.printf("Conn:%d", cachedConnectionTotal);

  y += ROW_HEIGHT + ROW_GAP;
  tileSprite.setTextColor(TFT_GREEN);
  tileSprite.setCursor(4, y);
  tileSprite.printf("WRT:%d", cachedConnectionWebrtc);

  y += ROW_HEIGHT + ROW_GAP;
  tileSprite.setTextColor(TFT_YELLOW);
  tileSprite.setCursor(4, y);
  tileSprite.printf("MJP:%d", cachedConnectionMjpeg);
}

// Copy tile sprite to framebuffer bar
// The sprite is drawn with text in normal orientation (rotation 3)
// but stored in buffer as 160(W) x 240(H)
// When copied to the 720-wide bar, each tile fills 240 columns
void copyTileToBar(uint16_t *bar, int tileIndex) {
  uint16_t *src = (uint16_t *)tileSprite.getBuffer();
  // After rotation 3, the sprite buffer is still TILE_WIDTH x TILE_HEIGHT
  // but the logical drawing coordinates are swapped
  // We need to copy row by row, accounting for the layout

  // Sprite buffer layout (after rotation): 160 x 240 pixels
  // Each row in the sprite corresponds to a column in the bar
  // tileIndex determines X offset in the bar (tileIndex * 240)

  int xOffset = tileIndex * TILE_HEIGHT;  // 240 per tile

  // The sprite is 160 wide (bar height) by 240 tall (tile width)
  // Copy to bar: bar is 720 wide, 160 tall in framebuffer terms
  for (int y = 0; y < TILE_WIDTH; y++) {  // 0-159 (bar height)
    memcpy(bar + y * PANEL_WIDTH + xOffset,
           src + y * TILE_HEIGHT,
           TILE_HEIGHT * sizeof(uint16_t));
  }
}

} // namespace

void OverlayRenderer::init(uint16_t *framebuffer) {
  if (topBar != nullptr)
    return;

  topBar = framebuffer;
  bottomBar = framebuffer + (PANEL_HEIGHT - OVERLAY_BAR_SIZE) * PANEL_WIDTH;

  // Create tile sprite in internal SRAM
  // Sprite: 160(W) x 240(H) with rotation 3
  // After rotation, drawing coordinates become 240(W) x 160(H)
  tileSprite.setPsram(false);
  tileSprite.setColorDepth(16);
  if (!tileSprite.createSprite(TILE_HEIGHT, TILE_WIDTH)) {  // 240 x 160
    Serial.println("OverlayRenderer: tileSprite FAILED");
    return;
  }
  tileSprite.setRotation(3);  // 270° CCW - text appears correct in landscape
  tileSprite.setTextSize(2);
  Serial.printf("OverlayRenderer: tileSprite OK (%dx%d, rot=3, internal SRAM)\n",
                TILE_HEIGHT, TILE_WIDTH);

  // Clear bars initially (dark gray background)
  for (int y = 0; y < OVERLAY_BAR_SIZE; y++) {
    for (int x = 0; x < PANEL_WIDTH; x++) {
      topBar[y * PANEL_WIDTH + x] = BG_COLOR;
      bottomBar[y * PANEL_WIDTH + x] = BG_COLOR;
    }
  }
  esp_cache_msync(topBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  esp_cache_msync(bottomBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  initialized = true;
  lastDetectionTime = millis();
  Serial.printf("OverlayRenderer: top=%p, bottom=%p, initialized\n",
                topBar, bottomBar);
}

void OverlayRenderer::render(DetectionData &detectionData, ConnectionData &connectionData) {
  if (!initialized)
    return;

  // Read detection data
  static Detection cachedDetections[MAX_DETECTIONS];
  static int cachedCount = 0;
  static uint32_t cachedTimestamp = 0;

  Detection detections[MAX_DETECTIONS];
  int count = 0;
  uint32_t timestamp = 0;

  if (detectionData.tryRead(detections, &count, &timestamp)) {
    memcpy(cachedDetections, detections, sizeof(Detection) * count);
    cachedCount = count;
    cachedTimestamp = timestamp;
    lastDetectionTime = millis();
  }

  // Read connection data
  int connTotal = 0, connWebrtc = 0, connMjpeg = 0;
  bool connDataReceived = connectionData.tryRead(&connTotal, &connWebrtc, &connMjpeg);
  if (connDataReceived) {
    cachedConnectionTotal = connTotal;
    cachedConnectionWebrtc = connWebrtc;
    cachedConnectionMjpeg = connMjpeg;
  }

  // Check for detection timeout - clear display if no data received
  uint32_t now = millis();
  if ((now - lastDetectionTime) > DETECTION_DISPLAY_TIMEOUT_MS) {
    if (cachedCount > 0) {
      cachedCount = 0;  // Clear cached detections
      Serial.println("OverlayRenderer: Detection timeout, clearing display");
    }
  }

  // === RIGHT BAR (Camera side, top in framebuffer) - Connection Count ===
  // Process this FIRST, independent of detection updates
  static bool rightBarInitialized = false;
  static int prevConnTotal = -1;
  static int prevConnWebrtc = -1;
  static int prevConnMjpeg = -1;

  bool connChanged = (cachedConnectionTotal != prevConnTotal ||
                      cachedConnectionWebrtc != prevConnWebrtc ||
                      cachedConnectionMjpeg != prevConnMjpeg);

  if (!rightBarInitialized || connChanged) {
    renderConnectionTile();
    copyTileToBar(topBar, 0);

    // Clear tiles 1 and 2 (only once)
    if (!rightBarInitialized) {
      tileSprite.fillSprite(BG_COLOR);
      copyTileToBar(topBar, 1);
      copyTileToBar(topBar, 2);
    }

    esp_cache_msync(topBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    rightBarInitialized = true;
    prevConnTotal = cachedConnectionTotal;
    prevConnWebrtc = cachedConnectionWebrtc;
    prevConnMjpeg = cachedConnectionMjpeg;
    Serial.printf("OverlayRenderer: Connection updated: %d/%d/%d\n",
                  cachedConnectionTotal, cachedConnectionWebrtc, cachedConnectionMjpeg);
  }

  // === LEFT BAR (USB side, bottom in framebuffer) - Detection List ===
  // Calculate which tiles need updating
  int currTiles = (cachedCount + ROWS_PER_TILE - 1) / ROWS_PER_TILE;  // Round up
  int prevTiles = (prevDetectionCount + ROWS_PER_TILE - 1) / ROWS_PER_TILE;
  int tilesToUpdate = max(currTiles, prevTiles);

  // Only render if detection count changed
  if (cachedCount == prevDetectionCount && tilesToUpdate == 0) {
    return;  // No detection changes, skip left bar render
  }

  for (int i = 0; i < tilesToUpdate; i++) {
    renderDetectionTile(i, cachedDetections, cachedCount);
    copyTileToBar(bottomBar, i);
  }

  // Clear remaining tiles if detection count decreased
  if (currTiles < prevTiles) {
    for (int i = currTiles; i < prevTiles; i++) {
      tileSprite.fillSprite(BG_COLOR);
      copyTileToBar(bottomBar, i);
    }
  }

  esp_cache_msync(bottomBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  prevDetectionCount = cachedCount;
}
