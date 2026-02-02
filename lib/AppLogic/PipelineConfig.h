#pragma once

#include <cstdint>

constexpr int STACK_DEPTH = 16384;
constexpr int BITSTREAM_PAD = 64;

constexpr uint32_t STREAM_WIDTH = 640;
constexpr uint32_t STREAM_HEIGHT = 480;

constexpr uint32_t PANEL_WIDTH = 720;
constexpr uint32_t PANEL_HEIGHT = 1280;

constexpr uint32_t DECODE_BUF_SIZE =
    ((STREAM_WIDTH * STREAM_HEIGHT) > (PANEL_WIDTH * PANEL_HEIGHT)
         ? (STREAM_WIDTH * STREAM_HEIGHT)
         : (PANEL_WIDTH * PANEL_HEIGHT)) *
    2;

constexpr uint32_t LINEAR_BUF_SIZE = 83558;   // 82KB (max 66KB + 25%margin)
constexpr uint32_t LINEAR_BUF_COUNT = 3;      // トリプルバッファ
constexpr uint32_t HEADER_BUF_SIZE = 64;      // MJPEGヘッダー解析用

// Overlay configuration (for landscape viewing)
constexpr uint32_t OVERLAY_BAR_SIZE = 160;    // 黒帯サイズ (top/bottom in framebuffer)
constexpr uint32_t VIDEO_Y_OFFSET = 160;      // 映像の開始Y座標
constexpr uint32_t VIDEO_HEIGHT = 960;        // 映像の高さ (480 * 1.5 * rotated)

// Detection API configuration
constexpr uint32_t DETECTION_TIMEOUT_MS = 30000;      // SSE streams need longer timeout
constexpr uint32_t DETECTION_RECONNECT_DELAY_MS = 3000;
