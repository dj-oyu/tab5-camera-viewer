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

constexpr uint32_t RING_BUF_SIZE = 1024 * 1024;
constexpr uint32_t LINEAR_BUF_SIZE = 256 * 1024;
