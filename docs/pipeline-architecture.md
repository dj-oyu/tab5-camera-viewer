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
  [JPEG HW Decoder] → decode_bufs[0/1] (セマフォでダブルバッファ管理)
                                              ↓
                                    decode semaphores (2 slots)
                                              ↓
Stage 3: Render Task (Priority 5, Core 1)
  [PPA Transform → Panel FB直接書込み] → [Overlay → Panel FB bar領域]
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
    size_t aligned_len; // 64-byte境界に揃えたデコード用長
    bool is_linear;    // 常にtrue（現在の実装）
};
```

### Decode → Render (セマフォベース)

FreeRTOS Queue の代わりに `PipelineContext` の API でダブルバッファを管理:

```cpp
// DecodeTask:
uint8_t *buf = ctx->acquireDecodeBuf();   // slot取得 (空きなければブロック)
// ... JPEG HW decode into buf ...
ctx->commitDecodedFrame();                 // RenderTaskに通知
// (失敗時は ctx->discardDecodeBuf() でslot返却)

// RenderTask:
uint8_t *buf = ctx->waitDecodedFrame(timeout);  // フレーム待ち
// ... PPA transform from buf → panel FB ...
ctx->releaseDecodedFrame();                      // slot返却
```

内部実装: カウンティングセマフォ2個 (`decode_slot_sema_`, `decoded_frame_sema_`) で
デコード前にslotを確保し、PPA完了後に解放。バッファ上書きレースを防止。

## バッファ管理

### メモリレイアウト

```
┌──────────────────────────────────────────────────────────┐
│                  INTERNAL SRAM (優先領域)                 │
├──────────────────────────────────────────────────────────┤
│ Fetch RX buffer (32KB, SPIRAM優先・フォールバック内部)      │
│   - ソケットread受信キャッシュ                             │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│                    SPIRAM (16MB+)                        │
├──────────────────────────────────────────────────────────┤
│ Linear Buffers [0..1] (82KB × 2)                         │
│   - JPEG圧縮フレーム用ダブルバッファ                       │
├──────────────────────────────────────────────────────────┤
│ Decode Buffers [0] (640×480×2 = 614KB)                  │
│   - JPEG decode出力 (RGB565)                             │
├──────────────────────────────────────────────────────────┤
│ Decode Buffers [1] (640×480×2 = 614KB)                  │
│   - ダブルバッファリング                                 │
├──────────────────────────────────────────────────────────┤
│ Panel Framebuffer (720×1280×2 = 1.76MB × 1)             │
│   - PPA DMA2D出力先 + Overlay描画先 (DSI直接読取)         │
├──────────────────────────────────────────────────────────┤
│ MicroLink Coord Buffer (64KB)                            │
│   - Tailscale MapResponse解析用 (VPN有効時のみ)          │
└──────────────────────────────────────────────────────────┘
```

### バッファフロー

```
HTTP Stream
    ↓
Linear Buffer [0/1] ←────────────┐
    │                             │
    │ (JPEG圧縮データ)            │
    ↓                             │
frameQueue                        │
    ↓                             │
JPEG HW Decoder (DMA2D)          │
    ↓                             │
Decode Buffers [0/1]              │
    │  (セマフォでダブルバッファ管理) │
    ↓                             │
PPA Transform (DMA2D) → Panel FB  │
    ↓                             │
Overlay → Panel FB bar領域        │
    ↓                             │
Display (DSI がFBを直接読取)       │
                                  │
Linear Buffer Pool ───────────────┘
(linearFreeQueue)
```

## 同期メカニズム

### キュー (FreeRTOS Queue)

| キュー名            | サイズ | 送信元 | 受信先 | 用途                |
| ------------------- | ------ | ------ | ------ | ------------------- |
| `frameQueue`        | 2      | Fetch  | Decode | JPEG圧縮データ      |
| `linearFreeQueue`   | 2      | Decode | Fetch  | Linear buffer再利用 |

### 同期プリミティブ

| 名前                   | 種類                 | 用途                          |
| ---------------------- | -------------------- | ----------------------------- |
| `decode_slot_sema_`    | Counting Semaphore(2) | デコードバッファ空きslot管理   |
| `decoded_frame_sema_`  | Counting Semaphore(2) | デコード済みフレーム通知       |
| `dma2d_gate_`          | Counting Semaphore(2) | JPEG+PPA DMA2D並列制御        |
| PPA完了通知            | Task Notification    | PPA処理完了通知 (index 0)     |

> **Note**: PPA完了通知は `vTaskNotifyGiveFromISR` / `ulTaskNotifyTake` を使用。

### 同期フロー

```
Decode Task:
    │
    ├─ acquireDecodeBuf() ← decode_slot_sema_ 取得 (空きバッファ待ち)
    │
    ├─ dma2d_gate_ 取得
    ├─ JPEG HW Decode → decode_buf[write_idx]
    ├─ dma2d_gate_ 解放
    │
    ├─ commitDecodedFrame() ← decoded_frame_sema_ 通知
    │
    └─ (次のループへ)

Render Task:
    │
    ├─ waitDecodedFrame() ← decoded_frame_sema_ 取得 (フレーム待ち)
    │
    ├─ dma2d_gate_ 取得
    ├─ PPAPipeline::submit() → Panel FB video領域 (DMA2D非同期)
    ├─ ulTaskNotifyTake(pdTRUE, 120ms) ← PPA完了待ち
    ├─ dma2d_gate_ 解放
    │
    ├─ releaseDecodedFrame() ← decode_slot_sema_ 返却
    │
    ├─ OverlayRenderer::render() → Panel FB bar領域
    │   (esp_cache_msync で PSRAM キャッシュフラッシュ)
    │
    └─ (draw_bitmap不要: DSI がPanel FBを直接読取)

ISR (PPA Done):
    │
    └─ vTaskNotifyGiveFromISR(renderTaskHandle) ← PPA完了通知
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

| 操作                     | タイムアウト | 動作                     |
| ------------------------ | ------------ | ------------------------ |
| frameQueue受信           | 1000ms       | vTaskDelay(1)            |
| waitDecodedFrame         | 1000ms       | vTaskDelay(1)            |
| PPA Task Notification    | 120ms        | バッファ返却してスキップ |

### リソース枯渇

**Linear Buffer枯渇:**

```cpp
if (!ctx->acquireLinear(active_buf)) {
    taskYIELD();
    continue;  // バッファ確保まで待機
}
```

**Queue満杯:**

```cpp
if (xQueueSend(frameQueue, &fd, pdMS_TO_TICKS(1)) != pdTRUE) {
    releaseLinear(buf);  // 満杯時は即返却してドロップ
}
```

## Overlay検出差分判定

- `DetectionTask` はSSE payloadから `class_name` と bbox (`x,y,w,h`) を取り出して共有データへ反映する。
- `OverlayRenderer` は `class_name + x,y,w,h` を元に、`xor/rotate/shift` ベースの軽量シグネチャを算出する。
- bboxは4px量子化して微小ジッタを吸収し、件数が同じでも実検出が変わった場合のみ再描画する。

## メモリ使用量

### 静的割り当て

- Linear Buffers: 82KB × 2 = 164KB
- Decode Buffers: 614KB × 2 = 1.2MB
- Panel Framebuffer: 1.76MB × 1

**合計: ~3.1MB (SPIRAM)**

### FreeRTOS

- Stack (Render Task): 10KB
- Stack (Fetch Task): 12KB
- Stack (Decode Task): 8KB
- Stack (Detection Task): 8KB
- Stack (Connection Task): 8KB
- Queues: < 1KB
- Semaphores: < 1KB

**合計: ~47KB (RAM)**

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
LINEAR_BUF_SIZE = 83558          // 82KB (max 66KB + 25% margin)
LINEAR_BUF_COUNT = 2             // ダブルバッファ
LINEAR_INTERNAL_CACHE_COUNT = 0  // 全てSPIRAM (内部SRAM節約)
INTERNAL_CACHE_GUARD_BYTES = 131072 // 内部SRAM最低残量ガード
DECODE_BUF_SIZE = STREAM_WIDTH * STREAM_HEIGHT * 2  // 614,400 bytes
DECODE_BUF_COUNT = 2             // ダブルデコードバッファ (JPEG/PPA並列用)
DMA2D_GATE_COUNT = 2             // JPEG+PPA並列 (PPA直接FB書込みでdisplay DMA2D不使用)
FETCH_RX_BUF_SIZE = 32768
FETCH_TCP_RCVBUF_BYTES = 65536
FETCH_BLOCK_TIMEOUT_MS = 4
FETCH_COALESCE_MIN_BYTES = 8192
FETCH_COALESCE_WAIT_US = 1600
FETCH_COALESCE_POLL_US = 50
FETCH_IDLE_BACKOFF_US = 200
VERIFY_FETCH_ONLY_MODE = false
VERIFY_WIFI_DIAG_LOG = false

// 性能ログ / 負荷制御
PERF_LOG_INTERVAL_MS = 2000
FPS_THROTTLE_ON = 24.0
FPS_THROTTLE_OFF = 26.0

// タスクスタック（個別最適化）
STACK_DEPTH_RENDER = 10240
STACK_DEPTH_FETCH  = 12288
STACK_DEPTH_DECODE = 8192
STACK_DEPTH_TAILSCALE = 16384   // TLS + cJSON
```

## デバッグ

### ログ出力

- Stream connected - HTTP接続成功
- MJPEG FPS / Render FPS - 2秒窓ごとのFPS
- Fetchは初回bootstrap後に `recv(MSG_DONTWAIT)` 直読みへ切り替え、read粒度を拡大

## 参考資料

- ESP32-P4 Technical Reference Manual
- ILI9881C Datasheet
- MIPI DSI Specification
- RFC 7230 (HTTP/1.1 Chunked Transfer Encoding)
