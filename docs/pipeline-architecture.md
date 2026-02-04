# MJPEG Streaming Pipeline Architecture

## 概要

M5Stack Tab5でHTTP MJPEGストリームを高速表示するための3ステージパイプラインアーキテクチャ。

## システム構成

### ハードウェア
- **SoC**: ESP32-P4 (Dual Core RISC-V, 360MHz)
- **ディスプレイ**: ILI9881C (720x1280, MIPI DSI)
- **メモリ**: 512KB RAM + 16MB SPIRAM
- **アクセラレータ**:
  - JPEG Hardware Decoder
  - PPA (Pixel Processing Accelerator) - スケーリング・回転

### ソフトウェア
- **フレームワーク**: Arduino ESP32 (PlatformIO)
- **RTOS**: FreeRTOS
- **ストリーム**: HTTP MJPEG (Transfer-Encoding: chunked)

## パイプライン構造

### 3ステージパイプライン

```
┌─────────────────────────────────────────────────────────────┐
│                    3-Stage Pipeline                         │
└─────────────────────────────────────────────────────────────┘

Stage 1: Fetch Task (Priority 6, Core 1)
  [HTTP Stream] → [Chunked Decoder] → [Linear Buffer]
                                              ↓
                                        frameQueue (2 slots)
                                              ↓
Stage 2: Decode Task (Priority 5, Core 0)
  [JPEG HW Decoder] → decode_bufs[0/1]
                                              ↓
                                    decodedFrameQueue (2 slots)
                                              ↓
Stage 3: Render Task (Priority 5, Core 1)
  [PPA Transform] → [DSI Transfer] → Display
```

### タイムライン

```
Time:    0ms    10ms   20ms   30ms   40ms   50ms   60ms
         │      │      │      │      │      │      │
Fetch:   |--F1--|--F2--|--F3--|--F4--|--F5--|--F6--|
Decode:         |--D1--|--D2--|--D3--|--D4--|--D5--|
Render:                |--R1--|--R2--|--R3--|--R4--|

理論的スループット: 3倍改善
```

## データ構造

### FrameData (Fetch → Decode)
```cpp
struct FrameData {
    uint8_t *buf;      // JPEG圧縮データへのポインタ (Linear buffer)
    size_t len;        // データ長
    bool is_linear;    // 常にtrue（現在の実装）
};
```

### DecodedFrameData (Decode → Render)
```cpp
struct DecodedFrameData {
    int buf_idx;            // decode_bufs[0/1]のインデックス
    uint8_t *linear_buf;    // 返却するlinear buffer
    bool has_linear_buf;    // linear bufferの有無
};
```

## バッファ管理

### メモリレイアウト

```
┌──────────────────────────────────────────────────────────┐
│                    SPIRAM (16MB)                         │
├──────────────────────────────────────────────────────────┤
│ Linear Buffers [0] (82KB)                                │
│   - JPEGフレーム格納用                                   │
├──────────────────────────────────────────────────────────┤
│ Linear Buffers [1] (82KB)                                │
├──────────────────────────────────────────────────────────┤
│ Linear Buffers [2] (82KB)                                │
│   - トリプルバッファリング                               │
├──────────────────────────────────────────────────────────┤
│ Decode Buffers [0] (640×480×2 = 614KB)                  │
│   - JPEG decode出力 (RGB565)                             │
├──────────────────────────────────────────────────────────┤
│ Decode Buffers [1] (640×480×2 = 614KB)                  │
│   - ダブルバッファリング                                 │
├──────────────────────────────────────────────────────────┤
│ Device Framebuffer (720×1280×2 = 1.76MB)                │
│   - DSIパネルから取得（シングルバッファ）                │
└──────────────────────────────────────────────────────────┘
```

### バッファフロー

```
HTTP Stream
    ↓
Linear Buffer [0/1/2] ←───────────┐
    │                             │
    │ (JPEG圧縮データ)            │
    ↓                             │
frameQueue                        │
    ↓                             │
JPEG Decoder                      │
    ↓                             │
Decode Buffers [0/1]              │
    ↓                             │
decodedFrameQueue                 │
    ↓                             │
PPA Transform                     │
    ↓                             │
Device Framebuffer                │
    ↓                             │
DSI Transfer                      │
    ↓                             │
Display                           │
                                  │
Linear Buffer Pool ───────────────┘
(linearFreeQueue)
```

## 同期メカニズム

### キュー (FreeRTOS Queue)

| キュー名 | サイズ | 送信元 | 受信先 | 用途 |
|---------|-------|--------|--------|------|
| `frameQueue` | 2 | Fetch | Decode | JPEG圧縮データ |
| `decodedFrameQueue` | 2 | Decode | Render | デコード済みRGB565 |
| `linearFreeQueue` | 3 | Render | Fetch | Linear buffer再利用 |

### セマフォ (FreeRTOS Semaphore)

| セマフォ名 | 種類 | 用途 |
|-----------|------|------|
| `ppaDoneSema` | Binary | PPA処理完了通知 |
| `displayDoneSema` | Binary | DSI転送完了通知 |

### 同期フロー

```
Render Task (Frame N):
    │
    ├─ xSemaphoreTake(displayDoneSema) ← Frame N-1のDSI完了待ち
    │
    ├─ PPAPipeline::transform() (非同期開始)
    │
    ├─ xSemaphoreTake(ppaDoneSema) ← PPA完了待ち
    │
    ├─ esp_lcd_panel_draw_bitmap() (DSI転送開始、非同期)
    │
    └─ (次のループへ)

ISR (DSI Transfer Done):
    │
    └─ xSemaphoreGiveFromISR(displayDoneSema) ← Frame N完了通知

ISR (PPA Done):
    │
    └─ xSemaphoreGiveFromISR(ppaDoneSema)
```

## Transfer-Encoding: Chunked対応

### Chunkedフォーマット

```
<chunk-size-hex>\r\n
<chunk-data>
\r\n
<chunk-size-hex>\r\n
<chunk-data>
\r\n
...
0\r\n
\r\n
```

### 状態機械

```cpp
enum class MjpegState {
    CHUNK_SIZE,     // サイズ読み取り中
    MJPEG_HEADER,   // Content-Type/Content-Length解析中
    JPEG_BODY,      // JPEGデータ読み取り中
    CHUNK_TRAILER   // \r\n読み取り中
};
```

### 処理ロジック

```
CHUNK_SIZE state:
    ├─ 16進数文字を蓄積
    ├─ '\n'検出 → サイズ解析
    ├─ chunk_remaining = strtoul(...)
    └─ → MJPEG_HEADER state

MJPEG_HEADER state:
    ├─ Content-Length解析
    ├─ "\r\n\r\n"検出 → ヘッダ終了
    └─ → JPEG_BODY state

JPEG_BODY state:
    ├─ Linear bufferにデータ蓄積
    ├─ jpeg_remaining == 0 → フレーム完了
    └─ → CHUNK_TRAILER or MJPEG_HEADER state

CHUNK_TRAILER state:
    ├─ \r\n を読み飛ばし
    └─ → CHUNK_SIZE state
```

## パフォーマンス特性

### 理論値

- **JPEG Decode**: ~5ms (640x480)
- **PPA Transform**: ~3ms (スケーリング+回転)
- **DSI Transfer**: ~8ms (720x1280@60Hz)

### パイプライン効果

**パイプライン前:**
```
1フレーム = 5ms + 3ms + 8ms = 16ms
FPS = 1000ms / 16ms = 62.5 FPS
```

**パイプライン後:**
```
スループット = 1000ms / max(5ms, 3ms, 8ms) = 125 FPS
実効FPS = ~60 FPS (DSI律速)
```

### ボトルネック

1. **DSI Transfer** (8ms) - 最も時間がかかる
2. HTTP Fetch速度 - ネットワーク帯域による
3. SPIRAM速度 - キャッシュミス時

## エラーハンドリング

### タイムアウト

| 操作 | タイムアウト | 動作 |
|------|-------------|------|
| frameQueue受信 | 1000ms | vTaskDelay(1) |
| decodedFrameQueue受信 | 1000ms | vTaskDelay(1) |
| displayDoneSema | 500ms | セマフォ返却してスキップ |
| ppaDoneSema | 無制限 | 待機 |

### リソース枯渇

**Linear Buffer枯渇:**
```cpp
if (!ctx->acquireLinear(&active_buf)) {
    taskYIELD();
    continue;  // バッファ確保まで待機
}
```

**Queue満杯:**
```cpp
xQueueSend(frameQueue, &fd, 0);  // タイムアウト0
// 満杯ならフレームドロップ
```

## メモリ使用量

### 静的割り当て

- Linear Buffers: 82KB × 3 = 246KB
- Decode Buffers: 614KB × 2 = 1.2MB
- Device Framebuffer: 1.76MB (DSIパネルが管理)

**合計: ~3.2MB (SPIRAM)**

### FreeRTOS

- Stack (Fetch Task): 16KB
- Stack (Decode Task): 16KB
- Stack (Render Task): 16KB
- Stack (Detection Task): 16KB
- Stack (Connection Task): 16KB
- Queues: < 1KB
- Semaphores: < 1KB

**合計: ~82KB (RAM)**

## ビルド設定

### platformio.ini

```ini
[env:esp32p4_pioarduino]
platform = espressif32
board = esp32-p4-evboard
framework = arduino
```

### コンパイル時定数 (PipelineConfig.h)

```cpp
// 解像度
STREAM_WIDTH = 640
STREAM_HEIGHT = 480
PANEL_WIDTH = 720
PANEL_HEIGHT = 1280

// バッファサイズ
LINEAR_BUF_SIZE = 83558  // 82KB (max 66KB + 25% margin)
LINEAR_BUF_COUNT = 3     // トリプルバッファ
DECODE_BUF_SIZE = max(STREAM, PANEL) * 2

// タスク設定
STACK_DEPTH = 16384
```

## デバッグ

### パフォーマンスモニタリング

```cpp
// Render Task内
frame_count++;
if (frame_count >= 100) {
    uint32_t now = millis();
    Serial.printf("Render FPS: %.1f\n",
                  100.0f * 1000.0f / (now - last_fps_time));
    last_fps_time = now;
    frame_count = 0;
}
```

### ログ出力

- Stream connected - HTTP接続成功
- Render FPS: XX.X - 100フレームごとのFPS

## 参考資料

- ESP32-P4 Technical Reference Manual
- ILI9881C Datasheet
- MIPI DSI Specification
- RFC 7230 (HTTP/1.1 Chunked Transfer Encoding)
