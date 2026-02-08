# TODO: MJPEG FPS 改善

## 解決済み: MJPEG FPS 低下 (22-25fps → 30fps)

サーバーが 30fps で配信しているにもかかわらず、Tab5 側で 22-25fps しか出ない。

### 計測データ

**サーバー側 (PC で計測)**:
```
FPS: 30.5 | Frame: 43.8KB avg | BW: 11.0Mbps
Receive Time: 1.2-12.5ms (avg 5.2ms)
```

**Tab5 側 (変更前)**:
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
| + `TCP_WND=8192` + `SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` | OK | **OK (FPS 30.3, mbps=8.73)** |

#### 結果: TCP_WND=8192 で 30fps 達成

```
Fetch Perf: fps=30.3 mbps=8.73 frame=35891b
Decode Perf: fps=30.5 decode=32490us errors=0
Render Perf: fps=30.6 ppa=27273us drops=0
MJPEG FPS: 30.3
```

理論どおり TCP ウィンドウサイズがボトルネックだった:
- TCP_WND=5760 → max ~9.2 Mbps → 22-25 fps
- TCP_WND=8192 → max ~13.1 Mbps → **30.3 fps**

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

#### 切り分け結果

- [x] **Step 1: Hybrid Compile 自体の影響を切り分け**
  - TCP_WND なし（フラッシュサイズのみ）でフルクリーンビルド → **動作OK**
  - **結論: Hybrid Compile 自体は問題なし、TCP_WND 変更が SDIO クラッシュの原因**

- [x] **Step 2: TCP_WND=8192 で SDIO クラッシュ回避 + FPS 改善**
  - TCP_WND=8192 + SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y → **30.3 fps 達成**
  - 起動時に散発的な USB CDC ISR クラッシュあり（再起動で解消、TCP_WND とは無関係）
