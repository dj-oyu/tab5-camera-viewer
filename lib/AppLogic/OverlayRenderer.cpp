#include "OverlayRenderer.h"
#include "ConnectionData.h"
#include "DetectionData.h"
#include "PipelineConfig.h"
#include "RecordingData.h"
#include <Arduino.h>
#include <M5GFX.h>
#include <cstring>
#include <esp_cache.h>

namespace
{

  // Tile-based sprite in internal SRAM for rotated text rendering
  // Bar: 160px wide × 720px long (in landscape)
  // Tile: 160px wide × 240px tall = 76.8KB (fits in internal SRAM)
  // 3 tiles cover the full 720 pixel length
  constexpr int TILE_WIDTH = OVERLAY_BAR_SIZE; // 160 (bar width)
  constexpr int TILE_HEIGHT = 240;             // Each tile covers 240px of bar length
  constexpr int NUM_TILES = 3;                 // 240 * 3 = 720
  LGFX_Sprite tileSprite;
  uint16_t *tileBuffer = nullptr;

  bool initialized = false;

  constexpr size_t BAR_BYTES = PANEL_WIDTH * OVERLAY_BAR_SIZE * sizeof(uint16_t);

  // Tile layout constants (matching ui-simulator README)
  // padding_top + (row_height × 4) + (gap × 3) + padding_bottom = 240px
  // 6px + (48px × 4) + (12px × 3) + 6px = 240px
  constexpr int TILE_PADDING = 6;
  constexpr int ROW_HEIGHT = 48;
  constexpr int ROW_GAP = 12;
  constexpr int ROWS_PER_TILE = 4;

  constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
  {
    return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
  }

  constexpr uint16_t panel565(uint16_t color)
  {
    return static_cast<uint16_t>((color << 8) | (color >> 8));
  }

  constexpr uint16_t panel565(uint8_t r, uint8_t g, uint8_t b)
  {
    return panel565(rgb565(r, g, b));
  }

  // Colors (matching ui-simulator)
  constexpr uint16_t BG_COLOR = panel565(0x11, 0x11, 0x11);    // Dark gray (#111111)
  constexpr uint16_t METER_COLOR = panel565(0x22, 0x88, 0x22); // Dark green (#228822)
  constexpr uint16_t TEXT_COLOR = panel565(0xFF, 0xFF, 0xFF);
  constexpr uint16_t CONN_COLOR = panel565(0x00, 0xFF, 0xFF);
  constexpr uint16_t ERROR_COLOR = panel565(0xFF, 0x00, 0x00);
  constexpr uint16_t CONNECTING_COLOR = panel565(0xFF, 0xA5, 0x00);
  constexpr uint16_t REC_COLOR = panel565(0xFF, 0x4D, 0x4D);      // Bright, lighter red for REC indicator
  constexpr uint16_t REC_IDLE_COLOR = panel565(0x44, 0x44, 0x44); // Dark gray for idle REC
  constexpr uint16_t REC_BG_IDLE = panel565(0xCC, 0x00, 0x00);    // REC idle button fill (#cc0000)
  constexpr uint16_t REC_BG_RECORD = panel565(0x44, 0x44, 0x44);  // Recording button fill (dark gray)
  constexpr uint16_t OK_GREEN = panel565(0x00, 0xFF, 0x00);
  constexpr uint16_t WARN_YELLOW = panel565(0xFF, 0xFF, 0x00);
  constexpr uint16_t DARK_GREY = panel565(0x55, 0x55, 0x55);
  constexpr uint16_t DIM_RED = panel565(0x80, 0x00, 0x00);

  // Detection display timeout (clear display if no data for this long)
  constexpr uint32_t DETECTION_DISPLAY_TIMEOUT_MS = 3000;

  struct OverlayBufferSnapshot
  {
    uint16_t *framebuffer = nullptr;
    bool bars_initialized = false;
    int detection_count = -1;
    int conn_total = -1;
    int conn_webrtc = -1;
    int conn_mjpeg = -1;
    ConnectionState conn_state = ConnectionState::Connecting;
    int conn_http_code = 0;
    RecordingState rec_state = RecordingState::Idle;
    int rec_duration_sec = -1;
    bool rec_blink = true;
  };

  OverlayBufferSnapshot overlay_snapshots[RENDER_BUF_COUNT];
  uint32_t lastDetectionTime = 0;
  int cachedConnectionTotal = 0;
  int cachedConnectionWebrtc = 0;
  int cachedConnectionMjpeg = 0;
  ConnectionState cachedConnectionState = ConnectionState::Connecting;
  int cachedConnectionHttpCode = 0;

  // Recording state cache
  RecordingState cachedRecordingState = RecordingState::Idle;
  float cachedRecordingDuration = 0.0f;
  uint32_t lastRecordingBlinkTime = 0;
  bool recordingBlinkOn = true;

  OverlayBufferSnapshot &snapshotFor(uint16_t *framebuffer)
  {
    for (int i = 0; i < RENDER_BUF_COUNT; ++i)
    {
      if (overlay_snapshots[i].framebuffer == framebuffer)
      {
        return overlay_snapshots[i];
      }
    }

    for (int i = 0; i < RENDER_BUF_COUNT; ++i)
    {
      if (overlay_snapshots[i].framebuffer == nullptr)
      {
        overlay_snapshots[i] = {};
        overlay_snapshots[i].framebuffer = framebuffer;
        return overlay_snapshots[i];
      }
    }

    // Should not happen with RENDER_BUF_COUNT framebuffers, fallback to slot 0.
    overlay_snapshots[0] = {};
    overlay_snapshots[0].framebuffer = framebuffer;
    return overlay_snapshots[0];
  }

  // Calculate Y position for a row within a tile
  int getRowY(int rowIndex)
  {
    return TILE_PADDING + rowIndex * (ROW_HEIGHT + ROW_GAP);
  }

  // Draw a detection row with confidence meter bar
  void drawDetectionRow(int rowY, const char *label, float confidence)
  {
    // Draw confidence meter bar as background
    int meterWidth = (int)(TILE_WIDTH * confidence);
    tileSprite.fillRect(0, rowY, meterWidth, ROW_HEIGHT, METER_COLOR);

    // Draw label text (centered vertically in row)
    tileSprite.setTextColor(TEXT_COLOR);
    int textY = rowY + (ROW_HEIGHT - 16) / 2; // 16 = approx text height at size 2
    tileSprite.setCursor(4, textY);
    tileSprite.print(label);
  }

  // Render a single detection tile (4 rows per tile)
  void renderDetectionTile(int tileIndex, Detection *detections, int totalCount)
  {
    tileSprite.fillSprite(BG_COLOR);

    int startIdx = tileIndex * ROWS_PER_TILE;
    for (int i = 0; i < ROWS_PER_TILE; i++)
    {
      int detIdx = startIdx + i;
      if (detIdx < totalCount)
      {
        int rowY = getRowY(i);
        drawDetectionRow(rowY, detections[detIdx].label, detections[detIdx].confidence);
      }
    }
  }

  // Render connection count tile (Right bar, Tile 0 only)
  void renderConnectionTile()
  {
    tileSprite.fillSprite(BG_COLOR);

    int y = TILE_PADDING;

    // Handle error and connecting states
    if (cachedConnectionState == ConnectionState::Error)
    {
      tileSprite.setTextColor(ERROR_COLOR);
      tileSprite.setCursor(4, y);
      tileSprite.print("CONN ERR");

      y += ROW_HEIGHT + ROW_GAP;
      tileSprite.setCursor(4, y);
      if (cachedConnectionHttpCode == -1)
      {
        tileSprite.print("WiFi NG");
      }
      else if (cachedConnectionHttpCode == -2)
      {
        tileSprite.print("No URL");
      }
      else if (cachedConnectionHttpCode == 0)
      {
        tileSprite.print("Disconn");
      }
      else
      {
        tileSprite.printf("HTTP %d", cachedConnectionHttpCode);
      }
      return;
    }

    if (cachedConnectionState == ConnectionState::Connecting)
    {
      tileSprite.setTextColor(CONNECTING_COLOR);
      tileSprite.setCursor(4, y);
      tileSprite.print("Conn...");
      return;
    }

    // Connected state - display connection counts
    tileSprite.setTextColor(CONN_COLOR);
    tileSprite.setCursor(4, y);
    tileSprite.printf("Conn:%d", cachedConnectionTotal);

    y += ROW_HEIGHT + ROW_GAP;
    tileSprite.setTextColor(OK_GREEN);
    tileSprite.setCursor(4, y);
    tileSprite.printf("WRT:%d", cachedConnectionWebrtc);

    y += ROW_HEIGHT + ROW_GAP;
    tileSprite.setTextColor(WARN_YELLOW);
    tileSprite.setCursor(4, y);
    tileSprite.printf("MJP:%d", cachedConnectionMjpeg);
  }

  // Render recording tile (Right bar, Tile 2) - Button design
  void renderRecordingTile()
  {
    tileSprite.fillSprite(BG_COLOR);

    bool isRecording = (cachedRecordingState == RecordingState::Recording);
    bool isPending = (cachedRecordingState == RecordingState::Pending);

    // Button area: centered, 130x80 pixels
    constexpr int BTN_W = 130;
    constexpr int BTN_H = 80;
    constexpr int BTN_X = (TILE_WIDTH - BTN_W) / 2; // Center horizontally
    constexpr int BTN_Y = 20;                       // Top padding
    constexpr int BTN_RADIUS = 12;

    if (isPending)
    {
      // Pending state: grayed button with "..." - immediate feedback
      tileSprite.fillRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, BTN_RADIUS, REC_BG_RECORD);
      tileSprite.drawRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, BTN_RADIUS, DARK_GREY);

      // Animated dots based on blink state
      tileSprite.setTextSize(3);
      tileSprite.setTextColor(WARN_YELLOW);
      tileSprite.setCursor(BTN_X + 45, BTN_Y + (BTN_H - 24) / 2);
      tileSprite.print(recordingBlinkOn ? "..." : " . ");
    }
    else if (isRecording)
    {
      // Recording state: STOP button with red border
      // Button background
      tileSprite.fillRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, BTN_RADIUS, REC_BG_RECORD);
      // Blinking red border
      if (recordingBlinkOn)
      {
        tileSprite.drawRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, BTN_RADIUS, REC_COLOR);
        tileSprite.drawRoundRect(BTN_X + 1, BTN_Y + 1, BTN_W - 2, BTN_H - 2, BTN_RADIUS - 1, REC_COLOR);
      }
      else
      {
        tileSprite.drawRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, BTN_RADIUS, DIM_RED);
      }

      // Blinking red circle indicator
      int dotX = BTN_X + 25;
      int dotY = BTN_Y + BTN_H / 2;
      if (recordingBlinkOn)
      {
        tileSprite.fillCircle(dotX, dotY, 10, REC_COLOR);
      }
      else
      {
        tileSprite.drawCircle(dotX, dotY, 10, REC_COLOR);
      }

      // "STOP" text (large font)
      tileSprite.setTextSize(3);
      tileSprite.setTextColor(TEXT_COLOR);
      tileSprite.setCursor(BTN_X + 45, BTN_Y + (BTN_H - 24) / 2);
      tileSprite.print("STOP");
    }
    else
    {
      // Idle state: REC button
      // Red button background
      tileSprite.fillRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, BTN_RADIUS, REC_BG_IDLE);
      tileSprite.drawRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, BTN_RADIUS, REC_COLOR);

      // Red circle indicator
      int dotX = BTN_X + 25;
      int dotY = BTN_Y + BTN_H / 2;
      tileSprite.fillCircle(dotX, dotY, 10, REC_COLOR);

      // "REC" text (large font)
      tileSprite.setTextSize(3);
      tileSprite.setTextColor(TEXT_COLOR);
      tileSprite.setCursor(BTN_X + 50, BTN_Y + (BTN_H - 24) / 2);
      tileSprite.print("REC");
    }

    // Duration display below button (large font)
    tileSprite.setTextSize(3);
    int durationY = BTN_Y + BTN_H + 15;

    if (isRecording || cachedRecordingDuration > 0)
    {
      int totalSeconds = (int)cachedRecordingDuration;
      int minutes = totalSeconds / 60;
      int seconds = totalSeconds % 60;

      tileSprite.setTextColor(isRecording ? OK_GREEN : TEXT_COLOR);
      tileSprite.setCursor(BTN_X + 20, durationY);
      tileSprite.printf("%02d:%02d", minutes, seconds);
    }
    else
    {
      tileSprite.setTextColor(REC_IDLE_COLOR);
      tileSprite.setCursor(BTN_X + 20, durationY);
      tileSprite.print("--:--");
    }

    // Error display
    if (cachedRecordingState == RecordingState::Error)
    {
      tileSprite.setTextSize(2);
      tileSprite.setTextColor(ERROR_COLOR);
      tileSprite.setCursor(BTN_X + 30, durationY + 35);
      tileSprite.print("ERROR");
    }

    // Reset text size for other tiles
    tileSprite.setTextSize(2);
  }

  // Copy tile sprite to framebuffer bar
  // The sprite is drawn with text in normal orientation (rotation 3)
  // but stored in buffer as 160(W) x 240(H)
  // When copied to the 720-wide bar, each tile fills 240 columns
  void copyTileToBar(uint16_t *bar, int tileIndex)
  {
    uint16_t *src = tileBuffer;
    if (!src)
    {
      return;
    }
    constexpr size_t kTileRowBytes = TILE_HEIGHT * sizeof(uint16_t);
    // After rotation 3, the sprite buffer is still TILE_WIDTH x TILE_HEIGHT
    // but the logical drawing coordinates are swapped
    // We need to copy row by row, accounting for the layout

    // Sprite buffer layout (after rotation): 160 x 240 pixels
    // Each row in the sprite corresponds to a column in the bar
    // tileIndex determines X offset in the bar (tileIndex * 240)

    int xOffset = tileIndex * TILE_HEIGHT; // 240 per tile

    // The sprite is 160 wide (bar height) by 240 tall (tile width)
    // Copy to bar: bar is 720 wide, 160 tall in framebuffer terms
    uint16_t *dst = bar + xOffset;
    for (int y = 0; y < TILE_WIDTH; y++)
    { // 0-159 (bar height)
      memcpy(dst, src, kTileRowBytes);
      dst += PANEL_WIDTH;
      src += TILE_HEIGHT;
    }
  }

} // namespace

void OverlayRenderer::init()
{
  if (initialized)
    return;

  // Create tile sprite in internal SRAM
  // Sprite: 160(W) x 240(H) with rotation 3
  // After rotation, drawing coordinates become 240(W) x 160(H)
  tileSprite.setPsram(false);
  tileSprite.setColorDepth(16);
  if (!tileSprite.createSprite(TILE_HEIGHT, TILE_WIDTH))
  { // 240 x 160
    Serial.println("OverlayRenderer: tileSprite FAILED");
    return;
  }
  tileSprite.setSwapBytes(false);
  tileSprite.setRotation(3); // 270° CCW - text appears correct in landscape
  tileSprite.setTextSize(2);
  Serial.printf("OverlayRenderer: tileSprite OK (%dx%d, rot=3, internal SRAM)\n",
                TILE_HEIGHT, TILE_WIDTH);
  tileBuffer = (uint16_t *)tileSprite.getBuffer();

  initialized = true;
  lastDetectionTime = millis();
  Serial.println("OverlayRenderer: initialized");
}

void OverlayRenderer::render(DetectionData &detectionData, ConnectionData &connectionData,
                             RecordingData &recordingData,
                             uint16_t *framebuffer)
{
  if (!initialized || framebuffer == nullptr)
    return;

  uint16_t *topBar = framebuffer;
  uint16_t *bottomBar = framebuffer + (PANEL_HEIGHT - OVERLAY_BAR_SIZE) * PANEL_WIDTH;

  OverlayBufferSnapshot &snapshot = snapshotFor(framebuffer);
  if (!snapshot.bars_initialized)
  {
    uint16_t bg = BG_COLOR;
    for (int y = 0; y < OVERLAY_BAR_SIZE; ++y)
    {
      for (int x = 0; x < PANEL_WIDTH; ++x)
      {
        topBar[y * PANEL_WIDTH + x] = bg;
        bottomBar[y * PANEL_WIDTH + x] = bg;
      }
    }
    esp_cache_msync(topBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    esp_cache_msync(bottomBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    snapshot.bars_initialized = true;
  }

  // Read detection data
  static Detection cachedDetections[MAX_DETECTIONS];
  static int cachedCount = 0;

  int count = 0;

  if (detectionData.tryRead(cachedDetections, count))
  {
    cachedCount = count;
    lastDetectionTime = millis();
  }

  // Read connection data and state
  int connTotal = 0, connWebrtc = 0, connMjpeg = 0;
  bool connDataReceived =
      connectionData.tryRead(connTotal, connWebrtc, connMjpeg);
  if (connDataReceived)
  {
    cachedConnectionTotal = connTotal;
    cachedConnectionWebrtc = connWebrtc;
    cachedConnectionMjpeg = connMjpeg;
  }

  // Always read connection state (independent of data updates)
  int httpCode = 0;
  ConnectionState connState = connectionData.getState(httpCode);
  cachedConnectionState = connState;
  cachedConnectionHttpCode = httpCode;

  // Check for detection timeout - clear display if no data received
  uint32_t now = millis();
  if ((now - lastDetectionTime) > DETECTION_DISPLAY_TIMEOUT_MS)
  {
    if (cachedCount > 0)
    {
      cachedCount = 0; // Clear cached detections
      Serial.println("OverlayRenderer: Detection timeout, clearing display");
    }
  }

  // Read recording data
  RecordingState recState = recordingData.getState();
  float recDuration = recordingData.getDuration();
  cachedRecordingState = recState;
  cachedRecordingDuration = recDuration;

  // Update blink state for recording/pending indicator (faster blink for pending)
  bool needsBlink = (recState == RecordingState::Recording || recState == RecordingState::Pending);
  uint32_t blinkInterval = (recState == RecordingState::Pending) ? 250 : 500; // Faster blink for pending
  if (needsBlink)
  {
    if (now - lastRecordingBlinkTime >= blinkInterval)
    {
      recordingBlinkOn = !recordingBlinkOn;
      lastRecordingBlinkTime = now;
    }
  }
  else
  {
    recordingBlinkOn = true;
    lastRecordingBlinkTime = now;
  }

  // === RIGHT BAR (Camera side, top in framebuffer) - Connection Count ===
  bool connChanged = (cachedConnectionTotal != snapshot.conn_total ||
                      cachedConnectionWebrtc != snapshot.conn_webrtc ||
                      cachedConnectionMjpeg != snapshot.conn_mjpeg ||
                      cachedConnectionState != snapshot.conn_state ||
                      cachedConnectionHttpCode != snapshot.conn_http_code);

  const int recDurationSec = static_cast<int>(recDuration);
  bool recChanged =
      (recState != snapshot.rec_state ||
       recDurationSec != snapshot.rec_duration_sec ||
       ((recState == RecordingState::Recording ||
         recState == RecordingState::Pending) &&
        recordingBlinkOn != snapshot.rec_blink));

  if (connChanged)
  {
    renderConnectionTile();
    copyTileToBar(topBar, 0);
  }

  if (recChanged)
  {
    renderRecordingTile();
    copyTileToBar(topBar, 2);
  }

  if (connChanged || recChanged)
  {
    tileSprite.fillSprite(BG_COLOR);
    copyTileToBar(topBar, 1);
    esp_cache_msync(topBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    snapshot.conn_total = cachedConnectionTotal;
    snapshot.conn_webrtc = cachedConnectionWebrtc;
    snapshot.conn_mjpeg = cachedConnectionMjpeg;
    snapshot.conn_state = cachedConnectionState;
    snapshot.conn_http_code = cachedConnectionHttpCode;
    snapshot.rec_state = recState;
    snapshot.rec_duration_sec = recDurationSec;
    snapshot.rec_blink = recordingBlinkOn;
  }

  // === LEFT BAR (USB side, bottom in framebuffer) - Detection List ===
  int currTiles = (cachedCount + ROWS_PER_TILE - 1) / ROWS_PER_TILE; // Round up
  int prevCount = snapshot.detection_count < 0 ? 0 : snapshot.detection_count;
  int prevTiles = (prevCount + ROWS_PER_TILE - 1) / ROWS_PER_TILE;
  if (snapshot.detection_count < 0)
  {
    prevTiles = NUM_TILES;
  }
  int tilesToUpdate = max(currTiles, prevTiles);
  bool detectionChanged =
      (snapshot.detection_count < 0 || cachedCount != snapshot.detection_count);

  if (detectionChanged || currTiles != prevTiles)
  {
    for (int i = 0; i < tilesToUpdate; ++i)
    {
      renderDetectionTile(i, cachedDetections, cachedCount);
      copyTileToBar(bottomBar, i);
    }

    if (currTiles < prevTiles)
    {
      for (int i = currTiles; i < prevTiles; ++i)
      {
        tileSprite.fillSprite(BG_COLOR);
        copyTileToBar(bottomBar, i);
      }
    }

    esp_cache_msync(bottomBar, BAR_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    snapshot.detection_count = cachedCount;
  }
}
