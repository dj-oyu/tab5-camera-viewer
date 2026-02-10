# TODO: バッテリーUI実装

## 完了済み (UIシミュレータ)

- [x] UIシミュレータにバッテリー表示を追加（右バー Tile 0）
- [x] コネクション表示を実機合わせに修正（WRT/MJP 2行、Conn行削除）
- [x] バッテリー＋コネクションをTile 0に統合
- [x] バックエンド: BatteryState + SSE + mockエンドポイント

## 完了済み (組込み側)

- [x] `BatteryData` / `BatteryTask` / `PipelineContext` / `AppLogic` 統合
- [x] `OverlayRenderer` バッテリーアイコン + WRT/MJP表示
- [x] `VERBOSE_PERF_LOG` フラグ追加
- [x] デバッグ用 Tile 1 表示追加

## 完了済み (INA226 キャリブレーション再設定)

- [x] BatteryTask で INA226 キャリブレーションレジスタ(0x05)を 3413 に上書き
  - `M5.In_I2C.writeRegister(0x41, 0x05, buf, 2, 400000)` で直接書き込み
- [x] 電流値を INA226 電流レジスタ(0x04)から直接読み取り
  - `M5.In_I2C.readRegister(0x41, 0x04, buf, 2, 400000)` → `(int16_t)(buf[0]<<8|buf[1]) * 0.3` mA
  - デバッグ Tile 1 で mA 値と電圧を表示
- [x] 充電判定: 電流の符号で判定（正=充電、要実機確認）
- [x] バッテリーレベル: バス電圧(0x02)からLiPoカーブで計算
  - 5点区分線形補間: 4200mV=100%, 4060mV=75%, 3860mV=50%, 3700mV=25%, 3500mV=0%
- [x] `BatteryData` に `currentMa` / `busVoltageMv` フィールドを追加（`chargingRaw` を置換）
- [x] デバッグ Tile 1: BAT%, 電圧(V), 電流(mA), CHRG/DCHR の4行表示

## 既知の問題: M5Unified Power API

### getBatteryLevel() — 不正確（INA226直接読みで回避済み）

- 2日間動作で90%を下回らなかった
- USB接続で即100%に復帰 → 充電電圧をそのまま測定しているため実SOCと乖離
- 原因: `getBusVoltage()` → 電圧→パーセント変換が充電中の電圧上昇を考慮しない

### getBatteryCurrent() — 不正確 + ハング（INA226直接読みで回避済み）

- M5Unified: `max_expected_current=2.0` → currentLSB=0.0000610, cal=16787
- 公式サンプル: `max_expected_current=8.192` → currentLSB=0.0003, cal=3413
- **キャリブレーションレジスタの値が約5倍違う** → 電流値が実態と異なる
- USBホットプラグでI2Cハング（M5Unified I2C のタイムアウト処理の問題）

### isCharging() — 常に充電中を返す（電流符号で回避済み）

- CHG_STAT pin (IO Expander 1, pin 6) のHW信号が常にHIGH

## 実機テスト（手動確認）

- [ ] デバッグ Tile で電圧/電流の生値が妥当か確認
- [ ] 充電中の電流符号を確認（正=充電 or 負=充電、必要なら `BatteryTask.cpp` の条件を反転）
- [ ] 充電中の電圧上昇による%ズレを確認、必要なら充電時補正を追加
- [ ] USB ホットプラグ耐性テスト
  - M5Unified の In_I2C 経由でもハングするか確認
  - ハングする場合: ポーリング間隔を長くする / I2C読み取りをタイムアウト保護

## 既知の問題: MJPEG FPS 低下 (22-25fps)

サーバーが 30fps で配信しているにもかかわらず、Tab5 側で 22-25fps しか出ない。
INA226 変更以前から発生しており、バッテリー実装とは無関係。

### 計測データ

**サーバー側 (PC で計測)**:
```
FPS: 30.5 | Frame: 43.8KB avg | BW: 11.0Mbps
Receive Time: 1.2-12.5ms (avg 5.2ms)
```

**Tab5 側 (VERBOSE_PERF_LOG)**:
```
MJPEG FPS: 22.5-25.0
Fetch Perf: mbps=8.21-9.24 frame=45453b idle=510-742us cwait=1724-1824us
            no_buf=0 drops=0 ovf=0 queues(frame=0 linear=1-2)
Decode Perf: fps=22.4-25.5 decode=3461-3504us queues(in=0 out=0-1)
Render Perf: fps=22.8-26.1 ppa=17082-17096us overlay=318-371us
```

### 分析

- **no_buf=0, drops=0**: バッファ不足・フレームドロップなし
- **frame queue = 0**: Decode はフレームが来れば即処理 → Fetch が供給不足
- **idle + cwait ≈ 2.5ms/frame**: フレーム周期33msに対して小さく、0にしても30fpsに届かない
- **受信スループット: 8-9Mbps** vs サーバー配信 11Mbps → **WiFi 物理層がボトルネック**
- RSSI=-37dBm（良好）にもかかわらずスループット不足
- Fetch ソフトウェアパラメータ（coalesce, idle backoff）は影響が小さい

### 根本原因特定: LWIP TCP_WND = 5760 bytes

pioarduino プリビルドの LWIP で TCP ウィンドウサイズが 5760 bytes (4×MSS) に制限されていた。

```
TCP スループット上限 = TCP_WND / RTT
= 5760 / 0.005 sec (WiFi RTT ≈ 5ms)
= 9.2 Mbps ← 観測値 8-9Mbps と一致
```

- `CONFIG_LWIP_TCP_WND_DEFAULT=5760` (sdkconfig)
- `SO_RCVBUF=64KB` は設定済みだが TCP ウィンドウとは別（ソケットバッファのみ）
- `tcp_recved()` 内で `TCP_WND_MAX(pcb) = TCP_WND` にキャップされるためランタイム変更不可
- TCP ウィンドウスケーリングも無効

### 対策: custom_sdkconfig (Hybrid Compile) で TCP_WND 拡大

pioarduino の Hybrid Compile で LWIP を再コンパイルし TCP_WND を拡大する。理論上の上限:
```
32768 / 0.005 = 52.4 Mbps（11Mbps に十分）
```

#### Hybrid Compile ワークアラウンド（Windows + Git URL インストール）

pioarduino を Git URL でインストールした場合、Hybrid Compile にはワークアラウンドが必要:
- `build_lib` パス問題: `~/.platformio/platforms/espressif32/builder/build_lib` に pioarduino のファイルをコピー
- `toolchain-riscv32-esp` パス問題: `@src-...` サフィックス付きディレクトリへのジャンクション作成
- `huge_app.csv`: プロジェクトルートにコピー（済）

手順は [README.md](README.md) に記載。

#### Hybrid Compile 実機テスト結果

| custom_sdkconfig 内容 | ビルド | 実機動作 |
|---|---|---|
| フラッシュサイズのみ (`CONFIG_ESPTOOLPY_FLASHSIZE`) (キャッシュビルド) | OK | OK (FPS 20強、変化なし) |
| + `CONFIG_LWIP_TCP_WND_DEFAULT=32768` | OK | **SDIO クラッシュ** |
| + `CONFIG_LWIP_TCP_WND_DEFAULT=16384` | OK | **SDIO クラッシュ** (同一症状) |
| フラッシュサイズのみ (フルクリーン再ビルド) | OK | OK (FPS 20, mbps=7.36) |
| + `TCP_WND=16384` + SRAM relief設定 | OK | **SDIO クラッシュ** (設定未反映、下記参照) |
| + `TCP_WND=8192` + `SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` | OK | **OK (FPS 30.3, mbps=8.73)** ✅ |

#### 結果: TCP_WND=8192 で 30fps 達成

```
Fetch Perf: fps=30.3 mbps=8.73 frame=35891b
Decode Perf: fps=30.5 decode=32490us errors=0
Render Perf: fps=30.6 ppa=27273us drops=0
MJPEG FPS: 30.3
```

理論どおり TCP ウィンドウサイズがボトルネックだった:
- TCP_WND=5760 → max ~9.2 Mbps → 22-25 fps
- TCP_WND=8192 → max ~13.1 Mbps → **30.3 fps** ✅

#### SDIO クラッシュの詳細

TCP_WND=32768 と TCP_WND=16384 の両方で同一のクラッシュが発生:
```
E (1715) os_wrapper_esp: Q create failed
assert failed: bus_init_internal sdio_drv.c:1381 (from_slave_queue[prio_q_idx])
```

FreeRTOS キュー作成が NULL を返す → ESP-Hosted SDIO ドライバ初期化で assert 失敗。
TCP_WND 増加 → LWIP 内部 SRAM 消費増 → WiFi/FreeRTOS キュー用 SRAM 不足。
TCP_WND=8192 (デフォルト 5760 の約 1.4 倍) では発生せず。

#### custom_sdkconfig "Add" vs "Replace" の注意

pioarduino の Hybrid Compile が `custom_sdkconfig` を sdkconfig.defaults にマージする際:
- **Replace**: 既存キーの値を置換 → 正しく反映される
- **Add**: 新規キーをファイル末尾に追加 → 旧名エイリアスや Kconfig choice で上書きされる場合あり

TCP_WND=16384 テスト時に以下の設定が反映されなかった:
- `CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=6` → 旧名 `CONFIG_ESP32_WIFI_*` が後方にデフォルト値で残存
- `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_400=y` → "Add" で末尾追加されたが Kconfig choice 解決で無視

#### 切り分け結果

- [x] **Step 1: Hybrid Compile 自体の影響を切り分け**
  - TCP_WND なし（フラッシュサイズのみ）でフルクリーンビルド → **動作OK**
  - **結論: Hybrid Compile 自体は問題なし、TCP_WND 変更が SDIO クラッシュの原因**

- [x] **Step 2: TCP_WND=8192 で SDIO クラッシュ回避 + FPS 改善**
  - TCP_WND=8192 + SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y → **30.3 fps 達成** ✅
  - 起動時に散発的な USB CDC ISR クラッシュあり（再起動で解消、TCP_WND とは無関係）

## 完了済み (DMA2D 並列パイプライン + PPA直接FB書込み)

- [x] DMA2D ISR デッドロックの根本原因を特定
  - `esp_lcd_panel_draw_bitmap` が DMA2D (`esp_async_fbcpy`) を使う3番目のコンシューマー
  - JPEG(RX0) + PPA(RX1) で全RXチャネル占有 → display DMA2D が PENDING → ISR デッドロック
- [x] PPA 書込み先を panel framebuffer に直接変更
  - `esp_lcd_dpi_panel_get_frame_buffer()` で FB ポインタ取得 → PPA DMA2D 出力先に設定
  - `draw_bitmap` 廃止（PPA DMA2D は PSRAM 直接書込み、overlay は独自 `esp_cache_msync`）
- [x] render_buf 廃止 → ~3.5MB SPIRAM 節約
  - `render_bufs_[2]`, `render_free_queue_`, `display_done_sema_`, `RenderedFrameData` 削除
- [x] `DMA2D_GATE_COUNT=2` でカウンティングセマフォ並列実行
  - JPEG(Core0) と PPA(Core1) が DMA2D 上で並列動作
- [x] DecodeTask perf logging 改善（error/truncated パスでもログ出力）

## 次のタスク

### decodedFrameQueue 簡素化の検討

`ctx->decodeBufferBytes(dfd.buf_idx)` を `ctx->getDecodedBuf()` のようなAPIに置き換え、
`decodedFrameQueue` (buf_idx を運ぶ Queue) を削除できるか検討。

**現状**: DecodeTask → `decodedFrameQueue(DecodedFrameData{buf_idx})` → RenderTask
**候補**: ダブルバッファの swap を atomic/セマフォで管理し、Queue を不要にする

考慮点:
- `DECODE_BUF_COUNT=2` でダブルバッファ: DecodeTask が書込み中の buf と RenderTask が読取中の buf
- Queue 深度=2 がバックプレッシャー制御を兼ねている → 代替メカニズムが必要
- Queue 削除による内部 SRAM 節約量は微小 (~100B)、主にコード簡素化が目的

### 内部 SRAM メモリ最適化

render_buf 廃止で ~3.5MB SPIRAM が空いた。内部 SRAM も最適化して以下を検討:

1. **TCP Window 増量**: `TCP_WND=8192` → より大きな値でスループット向上
   - TCP_WND=16384 は以前 SDIO クラッシュ（内部 SRAM 不足）→ SPIRAM 活用で回避可能か
   - `CONFIG_LWIP_TCP_WND_DEFAULT` + `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`
2. **SPIRAM → 内部 SRAM 移動**: 高速アクセスが必要なバッファを内部 SRAM に移動
   - Linear buffer (JPEG input): 内部 SRAM に 1 本置くとフェッチ速度向上の可能性
   - `LINEAR_INTERNAL_CACHE_COUNT=1` で既に対応可能（現在は 0 = 全て SPIRAM）
3. **FETCH_RX_BUF_SIZE 最適化**: 現在 32KB (内部 SRAM)
   - ソケット read バッファサイズと内部 SRAM 消費のトレードオフ
4. **ヒープ使用量の計測**: `heap_caps_get_free_size()` で起動後の空き領域を定期ログ

## クリーンアップ（動作確認後）

- [ ] デバッグ用 Tile 1 (renderDebugTile) を削除
- [ ] 充電検出が安定したら稲妻マーク表示を有効化
- [ ] バッテリーアイコンの表示微調整
- [ ] LiPo電圧カーブの微調整（実測データに基づく）

## 参考

### INA226 キャリブレーション計算（公式サンプル準拠）

```
shunt_res = 0.005 Ω
max_expected_current = 8.192 A
minimumLSB = 8.192 / 32767 = 0.000250
currentLSB = ceil(0.000250 / 0.0001) * 0.0001 = 0.0003 A
cal = 0.00512 / (0.0003 * 0.005) = 3413
```

### INA226 レジスタマップ（使用するもの）

| Reg | 名称 | 用途 |
|-----|------|------|
| 0x02 | BUS_V | バス電圧 (raw × 0.00125 = V) |
| 0x04 | CURRENT | 電流 (raw × currentLSB = A) |
| 0x05 | CALIBRATION | キャリブレーション (書き込み: 3413) |

### リンク

- 公式サンプル: `sample/tab5-userdemo/components/power_monitor_ina226/`
  - `calibrate(0.005, 8.192)` — `hal_esp32.cpp:80`
  - `readShuntCurrent()` — `hal_power.cpp:24`
- M5Unified INA226_Class: `.pio/libdeps/.../utility/power/INA226_Class.cpp`
- INA226 I2C アドレス: `0x41`、シャント抵抗: `5mΩ`
- シミュレータ: `cd research/ui-simulator && uv run app.py`
