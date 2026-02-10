# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-P4 embedded system for displaying MJPEG video streams with real-time object detection overlay on M5Stack Tab5 (MIPI DSI display, 720x1280).

## Build Commands

```bash
pio run                         # Build
pio run --target upload         # Build and flash
pio run --target monitor        # Serial monitor (115200 bps)
pio run --target clean && pio run  # Clean build (recommended for first build)
```

## Environment Setup

Create `.env` file in project root (loaded by `env_loader.py` as build flags):
```
WIFI_SSID="<ssid>"
WIFI_PASS="<password>"
MJPEG_URL="http://<server>/stream"
DETECTION_URL="http://<server>/api/detections/stream?format=json"
CONNECTIONS_URL="http://<server>/api/connections/stream"
RECORDING_BASE_URL="http://<server>/api/recording"
TAILSCALE_AUTH_KEY="tskey-auth-xxxxx"  # Optional: enables Tailscale VPN
```

## Architecture

### 3-Stage FreeRTOS Pipeline

```
Fetch (P6, Core1) → frameQueue → Decode (P5, Core0) → decodedFrameQueue → Render (P5, Core1)
     ↑                                                                          │
     └─────────────────── linearFreeQueue (buffer return) ──────────────────────┘
```

- **Fetch**: HTTP MJPEG stream → Linear buffer
- **Decode**: JPEG hardware decoder → RGB565
- **Render**: PPA scale/rotate → DSI display

Additional tasks: `DetectionTask`, `ConnectionTask` (both P4, Core1) for SSE API streams.
Optional: `TailscaleTask` (P3, Core0) for Tailscale VPN via MicroLink (`components/microlink/`).

### Key Files

| File | Purpose |
|------|---------|
| `lib/AppLogic/PipelineConfig.h` | Buffer sizes, resolution, timeouts |
| `lib/AppLogic/PipelineContext.h` | Queues, semaphores, buffer management |
| `lib/AppLogic/FetchTask.cpp` | HTTP chunked stream parsing |
| `lib/AppLogic/OverlayRenderer.cpp` | Detection/connection display (side bars) |
| `lib/AppLogic/TailscaleTask.cpp` | Tailscale VPN client (MicroLink wrapper) |
| `lib/PPAPipeline/` | ESP32-P4 PPA hardware abstraction |

### Memory Layout (SPIRAM 16MB)

- Linear Buffers: 82KB × 3 (JPEG frame storage)
- Decode Buffers: 614KB × 2 (RGB565 dual buffer)
- Framebuffer: 1.76MB (720×1280×2, single buffer)

### Display Layout (Landscape)

```
┌────────────┬───────────┬────────────┐
│ Left 160px │ Video 960 │ Right 160px│
│ Detections │  ×720px   │ Connections│
└────────────┴───────────┴────────────┘
```

## Research Tools

**UI Simulator** - Test overlay without hardware:
```bash
cd research/ui-simulator && uv run app.py
# Open http://localhost:5000
```

## Known Issues

**Detection Flicker**: Detection list changes every frame (no temporal smoothing). See `research/detection-display-stability.md` for proposed solutions.

**esptool.py / click 互換性**: `click` 8.2+ で `ParamType.get_metavar()` のシグネチャが変わり、`esptool.py` v5.0.0-dev1 のブートローダービルドが失敗する。`click` 8.1.x にダウングレードすれば解消:
```bash
# PlatformIO 仮想環境の click をダウングレード
& "$env:USERPROFILE\.platformio\penv\Scripts\pip.exe" install "click<8.2"
```

## Development Rules

- **実装後のドキュメント更新**: アーキテクチャやバッファ構成を変更した場合は、`docs/` 配下のドキュメントも必ず更新すること
  - `docs/pipeline-architecture.md` - パイプライン構造、メモリレイアウト
  - `docs/task-interactions.md` - タスク間通信、キュー/セマフォ
- **gitブランチルール**: `main`はプロテクションあり PRでしかマージできない
  - Gitflowに準拠
  - edit, writeを伴うタスクでは最初に今いるブランチが最適かどうか**必ずチェック**

## Detailed Documentation

- `docs/pipeline-architecture.md` - Complete pipeline design (日本語)
- `docs/task-interactions.md` - Task communication flows
- `research/mjpeg-stream-analysis/FINDINGS.md` - HTTP chunked stream analysis
