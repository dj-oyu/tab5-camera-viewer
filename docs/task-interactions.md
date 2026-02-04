# タスク間インタラクション詳細

## タスク一覧

| タスク名 | 関数名 | Priority | Core | Stack | 役割 |
|---------|--------|----------|------|-------|------|
| Fetch | `fetchTask` | 6 | 1 | 16KB | HTTP MJPEGストリーム取得 |
| Decode | `decodeTask` | 5 | 0 | 16KB | JPEG Hardware Decode |
| Render | `renderTask` | 5 | 1 | 16KB | PPA Transform + DSI Transfer |
| Detection | `detectionTask` | 4 | 1 | 16KB | Detection API (SSE) |
| Connection | `connectionTask` | 4 | 1 | 16KB | Connection API (SSE) |

## 合流ポイント詳細

### 1. frameQueue (Fetch → Decode)

#### 送信側: Fetch Task

**ファイル**: [lib/AppLogic/FetchTask.cpp](../lib/AppLogic/FetchTask.cpp)

**送信** (submitFrame関数):
```cpp
void submitFrame(PipelineContext &ctx, uint8_t *buf, size_t len) {
  FrameData fd;
  fd.buf = buf;
  fd.len = len;
  fd.is_linear = true;
  xQueueSend(ctx.frameQueue(), &fd, 0);
}
```

#### 受信側: Decode Task

**ファイル**: [lib/AppLogic/DecodeTask.cpp](../lib/AppLogic/DecodeTask.cpp)

**受信**:
```cpp
FrameData fd;
if (xQueueReceive(ctx->frameQueue(), &fd, pdMS_TO_TICKS(1000)) == pdTRUE) {
    // JPEG decode処理
    ...
}
```

#### データフロー図

```
Fetch Task Thread (Core 1):
    │
    ├─ HTTP stream読み取り
    │   └─ Chunked decoder処理
    │
    ├─ Content-Length解析
    │
    ├─ Linear bufferにJPEGデータ蓄積
    │
    ├─ フレーム完了検出
    │
    ├─ FrameData構築
    │   ├─ buf: Linear bufferポインタ
    │   ├─ len: バイト数
    │   └─ is_linear: true
    │
    └─ xQueueSend(frameQueue, &fd, 0)
              ↓
        [frameQueue]
              ↓
Decode Task Thread (Core 0):
    │
    └─ xQueueReceive(frameQueue, &fd, 1000ms)
```

---

### 2. decodedFrameQueue (Decode → Render)

#### 送信側: Decode Task

**ファイル**: [lib/AppLogic/DecodeTask.cpp](../lib/AppLogic/DecodeTask.cpp)

**送信**:
```cpp
int current_idx = decode_idx;
// JPEG decode完了
decode_idx ^= 1;  // バッファ切り替え

DecodedFrameData dfd;
dfd.buf_idx = current_idx;              // decode_bufs[0 or 1]
dfd.linear_buf = fd.buf;
dfd.has_linear_buf = true;
xQueueSend(ctx->decodedFrameQueue(), &dfd, portMAX_DELAY);
```

#### 受信側: Render Task

**ファイル**: [lib/AppLogic/RenderTask.cpp](../lib/AppLogic/RenderTask.cpp)

**受信**:
```cpp
DecodedFrameData dfd;
if (xQueueReceive(ctx->decodedFrameQueue(), &dfd, pdMS_TO_TICKS(1000)) == pdTRUE) {
    // Linear buffer返却
    if (dfd.has_linear_buf) {
        ctx->releaseLinear(dfd.linear_buf);
    }
    // PPA + DSI処理
    ...
}
```

#### データフロー図

```
Decode Task Thread (Core 0):
    │
    ├─ xQueueReceive(frameQueue, &fd, 1000ms)
    │
    ├─ JPEG Hardware Decode
    │   ├─ 入力: fd.buf (JPEG compressed)
    │   └─ 出力: decode_bufs[decode_idx] (RGB565)
    │
    ├─ Cache sync
    │
    ├─ decode_idx切り替え (0 ↔ 1)
    │
    ├─ DecodedFrameData構築
    │   ├─ buf_idx: 現在のインデックス
    │   ├─ linear_buf: 返却対象バッファ
    │   └─ has_linear_buf: 返却フラグ
    │
    └─ xQueueSend(decodedFrameQueue, &dfd, FOREVER)
              ↓
        [decodedFrameQueue]
              ↓
Render Task Thread (Core 1):
    │
    └─ xQueueReceive(decodedFrameQueue, &dfd, 1000ms)
```

---

### 3. linearFreeQueue (Render → Fetch)

#### 送信側: Render Task

**ファイル**: [lib/AppLogic/RenderTask.cpp](../lib/AppLogic/RenderTask.cpp)

**返却**:
```cpp
if (dfd.has_linear_buf) {
    ctx->releaseLinear(dfd.linear_buf);
}
```

#### 受信側: Fetch Task

**ファイル**: [lib/AppLogic/FetchTask.cpp](../lib/AppLogic/FetchTask.cpp)

**取得**:
```cpp
if (!ctx->acquireLinear(&active_buf)) {
    taskYIELD();
    continue;
}
```

#### バッファライフサイクル

```
初期化 (PipelineContext::init):
    linear_bufs[0] → linearFreeQueue
    linear_bufs[1] → linearFreeQueue
    linear_bufs[2] → linearFreeQueue

実行時:
    ┌─────────────────────────────────────────┐
    │                                         ↓
    linearFreeQueue ← Render Task ← decodedFrameQueue
        ↓                                     ↑
    Fetch Task                                │
        ↓                                     │
    (使用中)                                  │
        ↓                                     │
    frameQueue → Decode Task → (JPEG decode完了)
```

---

### 4. displayDoneSema (DSI完了同期)

#### Give側: DSI Transfer Done Callback

**ファイル**: [lib/AppLogic/DisplayInit.cpp](../lib/AppLogic/DisplayInit.cpp)

**コールバック**:
```cpp
bool IRAM_ATTR on_color_trans_done(
    esp_lcd_panel_handle_t panel,
    esp_lcd_dpi_panel_event_data_t *edata,
    void *user_ctx) {
    BaseType_t high_priority_task_awoken = pdFALSE;
    if (s_ctx && s_ctx->displayDoneSema()) {
        xSemaphoreGiveFromISR(s_ctx->displayDoneSema(),
                              &high_priority_task_awoken);
    }
    return high_priority_task_awoken == pdTRUE;
}
```

#### Take側: Render Task

**ファイル**: [lib/AppLogic/RenderTask.cpp](../lib/AppLogic/RenderTask.cpp)

**待機**:
```cpp
if (xSemaphoreTake(ctx->displayDoneSema(), pdMS_TO_TICKS(500)) == pdTRUE) {
    // PPA処理開始
    ...
}
```

#### タイムライン

```
Time:     0ms        8ms        16ms       24ms
          │          │          │          │
Frame N-1:├─────PPA──┼────DSI───┤          │
          │          │          ↓          │
          │          │     on_color_trans_done()
          │          │          ↓          │
          │          │     Give(displayDoneSema)
          │          │                     │
Frame N:  │          ├─Take(displayDoneSema)
          │          │          ↓          │
          │          ├─────PPA──┼────DSI───┤
                                           ↓
                                    on_color_trans_done()
                                           ↓
                                    Give(displayDoneSema)
```

---

### 5. ppaDoneSema (PPA完了同期)

#### Give側: PPA Done Callback

**ファイル**: [lib/PPAPipeline/PPAPipeline.cpp](../lib/PPAPipeline/PPAPipeline.cpp)

**コールバック**:
```cpp
bool PPAPipeline::ppa_event_cb(ppa_client_handle_t ppa_client,
                               ppa_event_data_t *event_data,
                               void *user_data) {
    SemaphoreHandle_t sem = (SemaphoreHandle_t)user_data;
    if (sem) {
        BaseType_t task_woken = pdFALSE;
        xSemaphoreGiveFromISR(sem, &task_woken);
        return task_woken == pdTRUE;
    }
    return false;
}
```

#### Take側: Render Task

**ファイル**: [lib/AppLogic/RenderTask.cpp](../lib/AppLogic/RenderTask.cpp)

**開始+待機**:
```cpp
bool ppa_ok = PPAPipeline::transform(
    (const uint8_t *)ctx->decodeBuffer(dfd.buf_idx),
    (uint8_t *)videoBuffer,
    STREAM_WIDTH, STREAM_HEIGHT,
    VIDEO_HEIGHT, PANEL_WIDTH,  // rotated dimensions
    ...
    ctx->ppaDoneSema()
);

if (ppa_ok) {
    xSemaphoreTake(ctx->ppaDoneSema(), portMAX_DELAY);
    esp_lcd_panel_draw_bitmap(...);
}
```

#### タイムライン

```
Render Task:
    │
    ├─ PPAPipeline::transform() 呼び出し
    │   └─ ppa_do_scale_rotate_mirror() (非同期開始)
    │
    ├─ 関数即座にreturn
    │
    ├─ xSemaphoreTake(ppaDoneSema, FOREVER)
    │   └─ ブロック
    │
    │   ... PPA Hardware処理中 ...
    │
    │   ┌─ PPA完了 (ISR)
    │   │
    │   └─ ppa_event_cb()
    │       └─ xSemaphoreGiveFromISR(ppaDoneSema)
    │
    ├─ ブロック解除
    │
    └─ esp_lcd_panel_draw_bitmap()
```

---

## 完全な実行シーケンス

### 1フレームの処理 (Frame N)

```
T=0ms: Fetch Task (Core 1)
    ├─ HTTP stream読み取り
    ├─ Chunked decoder処理
    ├─ Content-Length解析
    ├─ Linear bufferにJPEGデータ蓄積
    ├─ フレーム完了検出
    ├─ FrameData作成
    └─ xQueueSend(frameQueue) ─────────┐
                                       ↓
T=2ms: Decode Task (Core 0)           │
    ├─ xQueueReceive(frameQueue) ◄────┘
    ├─ Cache sync
    ├─ jpeg_decoder_process()
    │   └─ JPEG HW Decoder (3-5ms)
    ├─ Cache sync
    ├─ decode_idx切り替え
    ├─ DecodedFrameData作成
    └─ xQueueSend(decodedFrameQueue) ──────┐
                                           ↓
T=7ms: Render Task (Core 1)               │
    ├─ xQueueReceive(decodedFrameQueue) ◄─┘
    ├─ Linear buffer返却
    ├─ xSemaphoreTake(displayDoneSema)
    │   └─ Frame N-1のDSI完了待ち
    ├─ PPAPipeline::transform() 開始
    ├─ xSemaphoreTake(ppaDoneSema)
    │   └─ PPA完了待ち (2-3ms)
    ├─ esp_lcd_panel_draw_bitmap() 開始
    │   └─ DSI Transfer (非同期, 7-8ms)
    └─ ループ先頭へ

T=17ms: DSI Transfer完了 (ISR)
    └─ on_color_trans_done()
        └─ xSemaphoreGiveFromISR(displayDoneSema)
            └─ Frame N+1がブロック解除
```

---

## デッドロック防止

### 潜在的なデッドロック

1. **Queue満杯でブロック**
   - 対策: タイムアウト0でフレームドロップ
   ```cpp
   xQueueSend(frameQueue, &fd, 0);  // タイムアウト0
   ```

2. **Semaphore待ちでスタック**
   - 対策: displayDoneSemaは500msタイムアウト
   ```cpp
   if (xSemaphoreTake(displayDoneSema, pdMS_TO_TICKS(500)) == pdTRUE) {
       // 処理
   } else {
       xSemaphoreGive(displayDoneSema);  // リセット
   }
   ```

3. **Linear buffer枯渇**
   - 対策: バッファなしは待機（トリプルバッファで緩和）
   ```cpp
   if (!ctx->acquireLinear(&active_buf)) {
       taskYIELD();
       continue;
   }
   ```

---

## リソース競合

### decode_bufs[]のアクセス

```
Decode Task (書き込み):
    decode_bufs[decode_idx] ← JPEG decoder出力
    decode_idx ^= 1;  // 即座に切り替え

Render Task (読み込み):
    decode_bufs[dfd.buf_idx] ← PPA読み込み

競合なし: decode_idxとdfd.buf_idxは異なる値
```

### Linear Bufferのアクセス

```
Fetch Task (書き込み):
    acquireLinear() でバッファ取得
    JPEGデータ書き込み
    submitFrame() でキューに送信

Decode Task (読み込み):
    frameQueue経由でポインタ受信
    JPEG decode処理

Render Task (返却):
    releaseLinear() でプールに返却

競合なし: キュー経由で排他制御
```

---

## パフォーマンスメトリクス

### Queue待機時間

**正常時:**
- frameQueue: ~0ms (Fetchが先行)
- decodedFrameQueue: ~0ms (Decodeが先行)
- linearFreeQueue: ~0ms (トリプルバッファ)

**高負荷時:**
- frameQueue: ~5ms (Decode律速)
- decodedFrameQueue: ~8ms (Render律速)
- linearFreeQueue: ~10ms (バッファ不足)

### Semaphore待機時間

**Frame N-1のDSI完了待ち:**
- 理想: 0ms (既に完了)
- 実測: 0-8ms

**PPA完了待ち:**
- 理想: 3ms
- 実測: 2-4ms

---

## トラブルシューティング

### フレームドロップ

**症状:** FPSが低い

**原因と対策:**
1. frameQueue満杯
   - Decode処理が遅い
   - JPEG画質を下げる

2. decodedFrameQueue満杯
   - Render処理が遅い
   - 解像度を下げる

3. Linear buffer枯渇
   - LINEAR_BUF_COUNTを増やす（現在3）

### タスクスタック

**症状:** Stack overflow

**対策:**
```cpp
// タスク作成時にスタックサイズ増加
xTaskCreatePinnedToCore(fetchTask, "Fetch",
    STACK_DEPTH * 2,  // 32KB
    NULL, 6, NULL, 1);
```

### メモリ不足

**症状:** Allocation失敗

**対策:**
- LINEAR_BUF_SIZE削減
- LINEAR_BUF_COUNT削減 (3 → 2)
- Decode buffersサイズ削減
