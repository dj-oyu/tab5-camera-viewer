# タスク間インタラクション詳細

## タスク一覧

| タスク名 | 関数名 | Priority | Core | Stack | 役割 |
|---------|--------|----------|------|-------|------|
| Render | `renderTask` | 6 | 1 | 16KB | PPA変換 + Overlay + DSI submit |
| Fetch | `fetchTask` | 5 | 1 | 16KB | HTTP MJPEG取得/解析 |
| Decode | `decodeTask` | 5 | 0 | 16KB | JPEG HW Decode |
| Detection | `detectionTask` | 4 | 1 | 8KB | Detection API (SSE) |
| Connection | `connectionTask` | 4 | 1 | 8KB | Connection API (SSE) |
| Recording | `recordingTask` | 1 | 1 | 8KB | Recording API |

RenderをCore1で最優先にして、Fetchの連続パースで描画が飢餓状態になるのを防ぐ。

## キュー/セマフォ

| 名前 | サイズ/型 | 送信元 | 受信先 | 用途 |
|------|-----------|--------|--------|------|
| `linearFreeQueue` | 3 (`uint8_t*`) | Render | Fetch | JPEG入力バッファ再利用 |
| `frameQueue` | 3 (`FrameData`) | Fetch | Decode | JPEG圧縮フレーム |
| `decodedFrameQueue` | 2 (`DecodedFrameData`) | Decode | Render | デコード済みフレーム |
| `renderFreeQueue` | 2 (`uint16_t*`) | Render | Render | 描画バッファ再利用 |
| `ppaDoneSema` | Binary | PPA ISR | Render | PPA完了通知 |
| `displayDoneSema` | Binary | DSI ISR | Render | DSI転送完了通知 |

## 1. Fetch → Decode (`frameQueue`)

```cpp
FrameData fd{.buf = buf, .len = len, .is_linear = true};
if (xQueueSend(ctx.frameQueue(), &fd, pdMS_TO_TICKS(1)) != pdTRUE) {
  ctx.releaseLinear(buf);  // decodeが詰まっているときはドロップして返却
}
```

- タイムアウト0送信によるバッファリークを解消。
- `no_linear_waits` / `queue_drops` を周期ログで可視化。
- Fetchの受信バッファ(`FETCH_RX_BUF_SIZE=16KB`)は内部SRAM優先で確保し、
  `LINEAR_INTERNAL_CACHE_COUNT`本のlinear bufferも内部SRAM優先で配置する
  （`INTERNAL_CACHE_GUARD_BYTES`を下回る場合/確保失敗時はSPIRAMへフォールバック）。
- ソケットは `SO_RCVBUF` を拡大し、短時間coalescing(`FETCH_COALESCE_*`)で
  `bytes_per_read` を上げる。
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
- ダブルバッファ切り替え時の「片側だけ古いUI」を防止。

## 5. バックプレッシャーとドロップ戦略

- `frameQueue`満杯時: Fetchでドロップして`linearFreeQueue`へ返却
- `renderFreeQueue`枯渇時: Renderは`displayDoneSema`を待って再利用
- `ppaDoneSema`/`displayDoneSema`待ちにタイムアウトを設け、周期ログで異常を検出

## 6. 計測ログ

- Fetch: `Fetch Perf: fps=... mbps=... frame=... read=... parse=... sync=... rwait=... idle=... cwait=... boot=... raw=...`
- Decode: `Decode Perf: fps=... decode=... errors=...`
- Render: `Render Perf: fps=... buf_wait=... ppa=... overlay=... disp_wait=...`

この3系列を同時に見れば、30fps未達時にどこで詰まっているかを即座に切り分けできる。

## 7. 負荷制御

- Renderが`FPS_THROTTLE_ON`未満の状態が2秒続くと、Detection/Connection更新間隔を段階的に拡大。
- Renderが`FPS_THROTTLE_OFF`以上の状態が5秒続くと、更新間隔を通常値へ復帰。
- SSE接続は維持し、表示反映頻度のみを制御するため、機能停止はしない。
