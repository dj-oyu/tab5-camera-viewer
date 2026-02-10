# タスク間インタラクション詳細

## タスク一覧

| タスク名   | 関数名           | Priority | Core | Stack | 役割                           |
| ---------- | ---------------- | -------- | ---- | ----- | ------------------------------ |
| Render     | `renderTask`     | 6        | 1    | 10KB  | PPA変換 + Overlay + DSI submit |
| Fetch      | `fetchTask`      | 5        | 1    | 12KB  | HTTP MJPEG取得/解析            |
| Decode     | `decodeTask`     | 5        | 0    | 8KB   | JPEG HW Decode                 |
| Detection  | `detectionTask`  | 4        | 1    | 8KB   | Detection API (SSE)            |
| Connection | `connectionTask` | 4        | 1    | 8KB   | Connection API (SSE)           |
| Recording  | `recordingTask`  | 1        | 1    | 8KB   | Recording API                  |
| Battery    | `batteryTask`    | 1        | 1    | 4KB   | Battery monitor (M5.Power)     |
| Tailscale  | `tailscaleTask`  | 3        | 0    | 16KB  | Tailscale VPN (MicroLink)      |
| *ML Coord* | *(内部タスク)*   | 2        | 1    | 8KB   | MicroLink coordination poll    |

RenderをCore1で最優先にして、Fetchの連続パースで描画が飢餓状態になるのを防ぐ。
TailscaleTaskはCore0でDecodeTask(P5)の下、MicroLink内部coordination poll(P2)はCore1で全パイプラインタスクの下。
`VERIFY_FETCH_ONLY_MODE=true` の間は Detection/Connection/Recording を起動しない。

## キュー/セマフォ

| 名前                | サイズ/型              | 送信元  | 受信先 | 用途                   |
| ------------------- | ---------------------- | ------- | ------ | ---------------------- |
| `linearFreeQueue`   | 3 (`uint8_t*`)         | Render  | Fetch  | JPEG入力バッファ再利用 |
| `frameQueue`        | 3 (`FrameData`)        | Fetch   | Decode | JPEG圧縮フレーム       |
| `decodedFrameQueue` | 2 (`DecodedFrameData`) | Decode  | Render | デコード済みフレーム   |
| `renderFreeQueue`   | 2 (`uint16_t*`)        | Render  | Render | 描画バッファ再利用     |
| PPA Task Notification | Task Notification (index 0) | PPA ISR | Render | PPA完了通知     |
| `displayDoneSema`   | Binary Semaphore       | DSI ISR | Render | DSI転送完了通知        |

## 1. Fetch → Decode (`frameQueue`)

```cpp
FrameData fd{.buf = buf, .len = len, .aligned_len = aligned_len, .is_linear = true};
if (xQueueSend(ctx.frameQueue(), &fd, pdMS_TO_TICKS(1)) != pdTRUE) {
  ctx.releaseLinear(buf);  // decodeが詰まっているときはドロップして返却
}
```

- タイムアウト0送信によるバッファリークを解消。
- `no_linear_waits` / `queue_drops` を周期ログで可視化。
- Fetchの受信バッファ(`FETCH_RX_BUF_SIZE=32KB`)は内部SRAM優先で確保し、
  `LINEAR_INTERNAL_CACHE_COUNT`本のlinear bufferも内部SRAM優先で配置する
  （`INTERNAL_CACHE_GUARD_BYTES`を下回る場合/確保失敗時はSPIRAMへフォールバック）。
- ソケットは `SO_RCVBUF` を拡大し、短時間coalescing(`FETCH_COALESCE_*`)で
  `bytes_per_read` を上げる。
- raw pathは `SO_RCVTIMEO` による短いブロッキング受信
  (`FETCH_BLOCK_TIMEOUT_MS`) を使う。
- 初回ブロッキング受信後の追加入力は `MSG_DONTWAIT` で短時間drainし、
  `FETCH_COALESCE_WAIT_US` の範囲でread粒度を高める。
- 起動ログに `SO_RCVBUF` の要求値と実効値(`getsockopt`)を出し、実環境の縮退を確認する。
- HTTP接続直後の初回のみ `WiFiClient::read` で受信をbootstrapし、その後は
  `stream.fd()` から `recv(MSG_DONTWAIT)` 直読みへ切り替える。

## 2. Decode → Render (`decodedFrameQueue`)

```cpp
esp_err_t ret = jpeg_decoder_process(...);
if (ret != ESP_OK || out_size == 0) {
  ctx->releaseLinear(fd.buf);
  continue;
}
DecodedFrameData dfd{.buf_idx = idx, .linear_buf = fd.buf, .has_linear_buf = true};
xQueueSend(ctx->decodedFrameQueue(), &dfd, portMAX_DELAY);
```

- 旧実装の不要な`esp_cache_msync`を削減し、decodeステージの固定コストを低減。
- キュー長をdecodeバッファ数(2)と一致させて上書きリスクを回避。

## 3. Render内部 (`renderFreeQueue` + `displayDoneSema`)

Renderはダブルバッファで以下を実行する。

1. `renderFreeQueue`から描画先を取得
2. `PPA transform`実行
3. `OverlayRenderer::render(..., render_buf)`
4. 直前のin-flightバッファがある場合のみ`displayDoneSema`待機
5. `esp_lcd_panel_draw_bitmap(..., render_buf)`でsubmit

```text
Frame N:   PPA/Overlay on Buffer B   || DSI transfer of Buffer A
Frame N+1: PPA/Overlay on Buffer A   || DSI transfer of Buffer B
```

これにより、PPAとDSIのオーバーラップが可能になり、1フレームあたりの直列待ち時間を短縮する。

## 4. Overlayのバッファ整合性

- Overlayは`framebuffer`引数を毎フレーム受け取る。
- バッファごとにスナップショットを持ち、
  - 接続情報
  - 録画状態
  - 検出件数
    の差分を判定して必要タイルのみ更新する。
- 検出件数が同一でも内容が変わるケースを拾うため、
  `class_name + x,y,w,h` から軽量シグネチャを生成して差分判定する。
  - `x,y,w,h` は4px単位で量子化し、bboxの微小揺れで毎フレーム更新にならないようにする。
- ダブルバッファ切り替え時の「片側だけ古いUI」を防止。

### 4.1 Overlay描画経路と色順序

- Overlayは`LGFX_Sprite`に描画したタイルを`memcpy`でフレームバッファへ転送する。
- `setSwapBytes()`は`pushSprite()`の転送経路に効くが、`getBuffer()`経由の`memcpy`には影響しない。
- このため、UIの色がずれる場合は「スプライト内部の色定義」をパネルの期待順序に合わせて事前にスワップする。
  - 本実装では`panel565()`で色定義時にバイト順を整形し、`memcpy`は生転送に固定する。
- PPAは動画領域(960x720)を毎フレーム更新する一方、Overlayは左右バーのみを任意タイミングで更新する。
  - 色順序の変換はOverlay側で完結させると、PPAのFPSや更新タイミングに影響しない。

## 5. VPN同期 (Tailscale)

`TAILSCALE_AUTH_KEY` が定義されている場合、TailscaleTaskがEventGroupを使ってVPN接続状態を通知する。

| 同期機構 | 型 | 送信元 | 受信先 | 用途 |
|----------|------|--------|--------|------|
| `VPN_CONNECTED_BIT` | EventGroup BIT0 | TailscaleTask | Fetch, Detection, Connection | VPN接続完了通知 |

起動シーケンス:
1. FetchTask: `initWiFi()` → WiFi接続完了
2. TailscaleTask: WiFi待機 → `microlink_init()` → `microlink_connect()` → VPN確立
3. FetchTask/DetectionTask/ConnectionTask: `xEventGroupWaitBits(VPN_CONNECTED_BIT)` → 解除後にHTTP接続開始

`TAILSCALE_AUTH_KEY` 未定義時はゲートをスキップし、従来通りWiFi接続後に即座にHTTP接続を開始する。

## 6. バックプレッシャーとドロップ戦略

- `frameQueue`満杯時: Fetchでドロップして`linearFreeQueue`へ返却
- `renderFreeQueue`枯渇時: Renderは`displayDoneSema`を待って再利用
- PPA Task Notification / `displayDoneSema`待ちにタイムアウト(120ms)を設け、周期ログで異常を検出

## 7. 計測ログ

- Fetch: `Fetch Perf: fps=... mbps=... frame=... read=... parse=... sync=... rwait=... idle=... cwait=... boot=... raw=...`
- Decode: `Decode Perf: fps=... decode=... errors=...`
- Render: `Render Perf: fps=... buf_wait=... ppa=... overlay=... disp_wait=...`

この3系列を同時に見れば、30fps未達時にどこで詰まっているかを即座に切り分けできる。

## 8. 負荷制御

- Renderが`FPS_THROTTLE_ON`未満の状態が2秒続くと、Detection/Connection更新間隔を段階的に拡大。
- Renderが`FPS_THROTTLE_OFF`以上の状態が5秒続くと、更新間隔を通常値へ復帰。
- SSE接続は維持し、表示反映頻度のみを制御するため、機能停止はしない。
