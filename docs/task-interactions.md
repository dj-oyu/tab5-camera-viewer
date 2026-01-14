# タスク間インタラクション詳細

## タスク一覧

| タスク名 | 関数名 | Priority | Core | Stack | 役割 |
|---------|--------|----------|------|-------|------|
| Fetch | `mjpegFetchTask` | 6 | 1 | 16KB | HTTP MJPEGストリーム取得 |
| Decode | `mjpegDecodeTask` | 5 | 1 | 16KB | JPEG Hardware Decode |
| Render | `mjpegRenderTask` | 5 | 1 | 16KB | PPA Transform + DSI Transfer |

## 合流ポイント詳細

### 1. frameQueue (Fetch → Decode)

#### 送信側: Fetch Task

**ファイル**: [lib/AppLogic/AppLogic.cpp](../lib/AppLogic/AppLogic.cpp)

**ゼロコピーパス** (Line 308-313):
```cpp
// Ring buffer内で完結したフレーム
FrameData fd;
fd.buf = ring_buf + frame_start_pos;  // ポインタのみ
fd.len = frame_len;
fd.is_linear = false;
xQueueSend(frameQueue, &fd, 0);
```

**コピーパス** (Line 379-384):
```cpp
// Linear bufferにコピーしたフレーム
FrameData fd;
fd.buf = active_lbuf;
fd.len = active_lptr;
fd.is_linear = true;
xQueueSend(frameQueue, &fd, 0);
```

#### 受信側: Decode Task

**ファイル**: [lib/AppLogic/AppLogic.cpp](../lib/AppLogic/AppLogic.cpp)

**受信** (Line 463):
```cpp
FrameData fd;
if (xQueueReceive(frameQueue, &fd, pdMS_TO_TICKS(1000)) == pdTRUE) {
    // JPEG decode処理
    ...
}
```

#### データフロー図

```
Fetch Task Thread:
    │
    ├─ HTTP stream読み取り
    │   └─ Ring bufferに蓄積
    │
    ├─ SOI/EOI検出
    │
    ├─ ゼロコピー判定
    │   ├─ YES: Ring bufferポインタ
    │   └─ NO:  Linear bufferへコピー
    │
    ├─ FrameData構築
    │   ├─ buf: ポインタ
    │   ├─ len: バイト数
    │   └─ is_linear: フラグ
    │
    └─ xQueueSend(frameQueue, &fd, 0)
              ↓
        [frameQueue]
              ↓
Decode Task Thread:
    │
    └─ xQueueReceive(frameQueue, &fd, 1000ms)
```

---

### 2. decodedFrameQueue (Decode → Render)

#### 送信側: Decode Task

**ファイル**: [lib/AppLogic/AppLogic.cpp](../lib/AppLogic/AppLogic.cpp)

**送信** (Line 489-494):
```cpp
int current_idx = decode_idx;
// JPEG decode完了
decode_idx ^= 1;  // バッファ切り替え

DecodedFrameData dfd;
dfd.buf_idx = current_idx;              // decode_bufs[0 or 1]
dfd.linear_buf = fd.is_linear ? fd.buf : nullptr;
dfd.has_linear_buf = fd.is_linear;
xQueueSend(decodedFrameQueue, &dfd, portMAX_DELAY);
```

#### 受信側: Render Task

**ファイル**: [lib/AppLogic/AppLogic.cpp](../lib/AppLogic/AppLogic.cpp)

**受信** (Line 508-512):
```cpp
DecodedFrameData dfd;
if (xQueueReceive(decodedFrameQueue, &dfd, pdMS_TO_TICKS(1000)) == pdTRUE) {
    // Linear buffer返却
    if (dfd.has_linear_buf) {
        xQueueSend(linearFreeQueue, &dfd.linear_buf, 0);
    }
    // PPA + DSI処理
    ...
}
```

#### データフロー図

```
Decode Task Thread:
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
Render Task Thread:
    │
    └─ xQueueReceive(decodedFrameQueue, &dfd, 1000ms)
```

---

### 3. linearFreeQueue (Render → Fetch)

#### 送信側: Render Task

**ファイル**: [lib/AppLogic/AppLogic.cpp](../lib/AppLogic/AppLogic.cpp)

**送信** (Line 510-512):
```cpp
if (dfd.has_linear_buf) {
    xQueueSend(linearFreeQueue, &dfd.linear_buf, 0);
}
```

#### 受信側: Fetch Task

**ファイル**: [lib/AppLogic/AppLogic.cpp](../lib/AppLogic/AppLogic.cpp)

**受信** (Line 349):
```cpp
if (xQueueReceive(linearFreeQueue, &active_lbuf, 0) != pdTRUE) {
    // バッファなし、次回まで待機
} else {
    active_lptr = 0;
    // バッファ使用開始
}
```

#### バッファライフサイクル

```
初期化 (begin):
    linear_bufs[0] → linearFreeQueue
    linear_bufs[1] → linearFreeQueue

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

**ファイル**: [lib/AppLogic/AppLogic.cpp](../lib/AppLogic/AppLogic.cpp)

**コールバック** (Line 120-128):
```cpp
bool IRAM_ATTR AppLogic::on_color_trans_done(
    esp_lcd_panel_handle_t panel,
    esp_lcd_dpi_panel_event_data_t *edata,
    void *user_ctx) {
    BaseType_t high_priority_task_awoken = pdFALSE;
    if (displayDoneSema) {
        xSemaphoreGiveFromISR(displayDoneSema, &high_priority_task_awoken);
    }
    return high_priority_task_awoken == pdTRUE;
}
```

#### Take側: Render Task

**ファイル**: [lib/AppLogic/AppLogic.cpp](../lib/AppLogic/AppLogic.cpp)

**待機** (Line 514):
```cpp
if (xSemaphoreTake(displayDoneSema, pdMS_TO_TICKS(500)) == pdTRUE) {
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

**コールバック** (Line 8-17):
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

**ファイル**: [lib/AppLogic/AppLogic.cpp](../lib/AppLogic/AppLogic.cpp)

**開始+待機** (Line 543-556):
```cpp
bool ppa_ok = PPAPipeline::transform(
    (const uint8_t *)decode_bufs[dfd.buf_idx],
    (uint8_t *)fb,
    STREAM_WIDTH, STREAM_HEIGHT,
    PANEL_WIDTH, PANEL_HEIGHT,
    PPA_SRM_COLOR_MODE_RGB565,
    PPA_SRM_COLOR_MODE_RGB565,
    rotation, scale, scale,
    ppaDoneSema  // ← コールバックで使用
);

if (ppa_ok) {
    xSemaphoreTake(ppaDoneSema, portMAX_DELAY);  // ← PPA完了待ち
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
T=0ms: Fetch Task
    ├─ HTTP stream読み取り
    ├─ Chunked decoder処理
    ├─ SOI検出
    ├─ データ蓄積 (Ring/Linear buffer)
    ├─ EOI検出
    ├─ FrameData作成
    └─ xQueueSend(frameQueue) ─────────┐
                                       ↓
T=2ms: Decode Task                    │
    ├─ xQueueReceive(frameQueue) ◄────┘
    ├─ Cache sync
    ├─ jpeg_decoder_process()
    │   └─ JPEG HW Decoder (3-5ms)
    ├─ Cache sync
    ├─ decode_idx切り替え
    ├─ DecodedFrameData作成
    └─ xQueueSend(decodedFrameQueue) ──────┐
                                           ↓
T=7ms: Render Task                        │
    ├─ xQueueReceive(decodedFrameQueue) ◄─┘
    ├─ Linear buffer返却 (if any)
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
   - 対策: バッファなしは待機、Ring bufferは保持
   ```cpp
   if (xQueueReceive(linearFreeQueue, &active_lbuf, 0) != pdTRUE) {
       // バッファなし、次回まで待機
       // データはRing bufferに残る
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

### Ring Bufferのアクセス

```
Fetch Task (書き込み):
    rb_head: 書き込み位置
    rb_tail: 読み取り位置 (parser)

Decode Task: アクセスなし (FrameDataのポインタ経由)

競合なし: 単一タスクのみアクセス
```

---

## パフォーマンスメトリクス

### Queue待機時間

**正常時:**
- frameQueue: ~0ms (Fetchが先行)
- decodedFrameQueue: ~0ms (Decodeが先行)
- linearFreeQueue: ~0ms (プールあり)

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
   - Chunk境界でフレームが分割されすぎ
   - Ring bufferサイズ増加

### タスクスタック

**症状:** Stack overflow

**対策:**
```cpp
// タスク作成時にスタックサイズ増加
xTaskCreatePinnedToCore(mjpegFetchTask, "Fetch",
    STACK_DEPTH * 2,  // 32KB
    NULL, 6, NULL, 1);
```

### メモリ不足

**症状:** Allocation失敗

**対策:**
- Linear buffer削減 (2 → 1)
- Ring bufferサイズ削減
- Decode buffersサイズ削減
