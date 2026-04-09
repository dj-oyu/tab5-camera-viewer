#pragma once

#include <cstdint>

constexpr int STACK_DEPTH_RENDER = 16384; // Merged decode+PPA+overlay pipeline
constexpr int STACK_DEPTH_FETCH  = 12288;
constexpr int BITSTREAM_PAD = 64;

constexpr uint32_t STREAM_WIDTH = 768;
constexpr uint32_t STREAM_HEIGHT = 432;

constexpr uint32_t PANEL_WIDTH = 720;
constexpr uint32_t PANEL_HEIGHT = 1280;

constexpr uint32_t DECODE_BUF_SIZE = STREAM_WIDTH * STREAM_HEIGHT * 2; // RGB565 at stream resolution

constexpr uint32_t LINEAR_BUF_SIZE = 83558;   // 82KB (max 66KB + 25%margin)
constexpr uint32_t RING_DEPTH = 3;            // SPSC ring depth (fetch/decode pipeline)
constexpr uint32_t LINEAR_INTERNAL_CACHE_COUNT = 0; // 内部SRAM節約: 全てSPIRAM
constexpr uint32_t INTERNAL_CACHE_GUARD_BYTES = 128 * 1024;
constexpr uint32_t HEADER_BUF_SIZE = 64;

constexpr uint32_t FETCH_RX_BUF_SIZE = 32768;
constexpr uint32_t FETCH_TCP_RCVBUF_BYTES = 64 * 1024;
constexpr uint32_t FETCH_BLOCK_TIMEOUT_MS = 4;

// Verification toggles
constexpr bool VERIFY_FETCH_ONLY_MODE = false;
constexpr bool VERIFY_WIFI_DIAG_LOG = false;

// Performance logging / control
constexpr bool VERBOSE_PERF_LOG = true;
constexpr uint32_t PERF_LOG_WINDOW_FRAMES = 60;
constexpr uint32_t PERF_LOG_INTERVAL_MS = 2000;
constexpr float FPS_THROTTLE_ON = 24.0f;
constexpr float FPS_THROTTLE_OFF = 26.0f;

// Overlay configuration (for landscape viewing)
constexpr uint32_t OVERLAY_BAR_SIZE = 160;
constexpr uint32_t VIDEO_Y_OFFSET = OVERLAY_BAR_SIZE;
constexpr uint32_t VIDEO_HEIGHT = 960;

// Detection API configuration
constexpr uint32_t DETECTION_TIMEOUT_MS = 30000;
constexpr uint32_t DETECTION_RECONNECT_DELAY_MS = 3000;
constexpr uint32_t DETECTION_SSE_IDLE_TIMEOUT_MS = 300000;

// Tailscale VPN task
constexpr int STACK_DEPTH_TAILSCALE = 16384;
