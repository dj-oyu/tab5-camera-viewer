#pragma once

#include <cstdint>

constexpr int STACK_DEPTH_RENDER = 10240;
constexpr int STACK_DEPTH_FETCH  = 12288;
constexpr int STACK_DEPTH_DECODE = 8192;
constexpr int BITSTREAM_PAD = 64;

constexpr uint32_t STREAM_WIDTH = 640;
constexpr uint32_t STREAM_HEIGHT = 480;

constexpr uint32_t PANEL_WIDTH = 720;
constexpr uint32_t PANEL_HEIGHT = 1280;

constexpr uint32_t DECODE_BUF_SIZE = STREAM_WIDTH * STREAM_HEIGHT * 2; // RGB565 at stream resolution

constexpr uint32_t LINEAR_BUF_SIZE = 83558;   // 82KB (max 66KB + 25%margin)
constexpr uint32_t LINEAR_BUF_COUNT = 3;      // トリプルバッファ
constexpr uint32_t LINEAR_INTERNAL_CACHE_COUNT = 1; // Linear bufferのうち内部SRAM優先数
constexpr uint32_t INTERNAL_CACHE_GUARD_BYTES = 128 * 1024; // 内部SRAMを使い切らないための最低確保量
constexpr uint32_t HEADER_BUF_SIZE = 64;      // MJPEGヘッダー解析用
constexpr uint32_t RENDER_BUF_COUNT = 2;      // Display submit用ダブルバッファ
constexpr uint32_t FETCH_RX_BUF_SIZE = 32768; // Fetch socket read size (internal SRAM cache)
constexpr uint32_t FETCH_TCP_RCVBUF_BYTES = 64 * 1024;
constexpr uint32_t FETCH_BLOCK_TIMEOUT_MS = 4;
constexpr uint32_t FETCH_COALESCE_MIN_BYTES = 8192;
constexpr uint32_t FETCH_COALESCE_WAIT_US = 1600;
constexpr uint32_t FETCH_COALESCE_POLL_US = 50;
constexpr uint32_t FETCH_IDLE_BACKOFF_US = 200;

// Verification toggles
constexpr bool VERIFY_FETCH_ONLY_MODE = false; // Step-1 validation: disable side API tasks
constexpr bool VERIFY_WIFI_DIAG_LOG = false;   // Step-3 validation: dump WiFi driver diag

// Performance logging / control
constexpr bool VERBOSE_PERF_LOG = true;        // Render/Fetch FPS periodic logs
constexpr uint32_t PERF_LOG_WINDOW_FRAMES = 60;
constexpr uint32_t PERF_LOG_INTERVAL_MS = 2000;
constexpr float FPS_THROTTLE_ON = 28.0f;
constexpr float FPS_THROTTLE_OFF = 29.5f;

// Overlay configuration (for landscape viewing)
constexpr uint32_t OVERLAY_BAR_SIZE = 160;    // 黒帯サイズ (top/bottom in framebuffer)
constexpr uint32_t VIDEO_Y_OFFSET = 160;      // 映像の開始Y座標
constexpr uint32_t VIDEO_HEIGHT = 960;        // 映像の高さ (480 * 1.5 * rotated)

// Detection API configuration
constexpr uint32_t DETECTION_TIMEOUT_MS = 30000;      // SSE streams need longer timeout
constexpr uint32_t DETECTION_RECONNECT_DELAY_MS = 3000;
