# Tab5 Camera Viewer

M5Stack Tab5 (ESP32-P4) 向けの MJPEG ビデオストリーミングビューア。リアルタイム物体検出オーバーレイ対応。

## 必要なもの

- [PlatformIO](https://platformio.org/) (VSCode 拡張 or CLI)
- M5Stack Tab5
- MJPEG 配信サーバー

## ビルド環境セットアップ

### 1. 環境変数ファイル

プロジェクトルートに `.env` を作成:

```
WIFI_SSID="<ssid>"
WIFI_PASS="<password>"
MJPEG_URL="http://<server>/stream"
DETECTION_URL="http://<server>/api/detections/stream?format=json"
CONNECTIONS_URL="http://<server>/api/connections/stream"
RECORDING_BASE_URL="http://<server>/api/recording"
```

### 2. Hybrid Compile ワークアラウンド (Windows)

`platformio.ini` の `custom_sdkconfig` が有効な場合、pioarduino の Hybrid Compile が動作します。
Git URL でプラットフォームをインストールした場合、以下のワークアラウンドが必要です:

```powershell
# (a) build_lib ディレクトリをコピー
$src = "$env:USERPROFILE\.platformio\platforms\espressif32@src-*\builder\build_lib"
$dst = "$env:USERPROFILE\.platformio\platforms\espressif32\builder"
New-Item -ItemType Directory -Path $dst -Force | Out-Null
Copy-Item -Path (Resolve-Path $src) -Destination "$dst\build_lib" -Recurse -Force

# (b) toolchain ジャンクション作成
$tcSrc = (Get-ChildItem "$env:USERPROFILE\.platformio\packages\toolchain-riscv32-esp@*" -Directory).FullName
cmd /c mklink /J "$env:USERPROFILE\.platformio\packages\toolchain-riscv32-esp" $tcSrc
```

### 3. esptool.py / click 互換性

`click` 8.2+ でブートローダービルドが失敗する場合:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pip.exe" install "click<8.2"
```

## ビルド & フラッシュ

```bash
pio run                         # ビルド
pio run --target upload         # ビルド & フラッシュ
pio run --target monitor        # シリアルモニタ (115200 bps)
```

初回の Hybrid Compile ビルドは数分かかります。2回目以降はキャッシュが使われます。

### 4. sections.ld の orphan セクション修正

pioarduino v54.03.21 のプリビルト `sections.ld` にはバグがあり、`libhal.a` 等に含まれる
`static inline` 由来の関数セクション（`axi_dma_ll_*`, `ahb_dma_ll_*` 等）がリンカースクリプトの
個別列挙から漏れています。これにより ELF セグメント数が esptool の上限 16 を超え、
`Invalid segment count (max 16)` エラーでビルドが失敗します。

**修正:** `sections.ld` の `.flash.text` セクション末尾（`*(.irom0.text)` の直後）に
catch-all ルールを追加:

```
~/.platformio/packages/framework-arduinoespressif32-libs/esp32p4/ld/sections.ld
```

```ld
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */
    /* Catch-all: absorb orphan .text.* sections from prebuilt libs
       whose function names are missing from the explicit listings above.
       Without this, orphan sections create extra ELF segments (max 16). */
    *(.text .text.*)
```

> **注意:** この修正は PlatformIO パッケージ内のファイルを変更するため、
> `framework-arduinoespressif32-libs` パッケージの更新時に上書きされます。
> パッケージ更新後は再適用が必要です。

## Hybrid Compile トラブルシューティング

pioarduino の `custom_sdkconfig` による Hybrid Compile (ESP-IDF ライブラリの再ビルド) は
ESP32-P4 + Windows + Git URL インストールの組み合わせで複数の問題があります。

### 問題と解決の経緯

#### 1. パス解決バグ (`@src-` サフィックス)

pioarduino を Git URL でインストールすると、ディレクトリ名に `@src-<hash>` サフィックスが付きます。
ビルドスクリプト (`espidf.py`) はサフィックスなしのパスをハードコードしているため、
`build_lib` と `toolchain-riscv32-esp` の2箇所でパス解決に失敗します。

**解決:** 上記「Hybrid Compile ワークアラウンド」のコピーとジャンクション作成。

#### 2. ELF セグメント数超過 (`Invalid segment count 23, max 16`)

プリビルトの `sections.ld` が `.flash.text` セクション内でライブラリの関数名を個別列挙し、
`EXCLUDE_FILE` で catch-all ルールから除外しています。しかしプリビルト `.a` ライブラリには
リンカースクリプト生成時に inline 展開されていた関数が非 inline として存在しており、
これらの関数セクションが orphan になり個別セグメントとして配置されます。

```
sections.ld の構造:

*(EXCLUDE_FILE(*libhal.a:gdma_hal_axi.* ...) .text .text.*)  ← catch-all (除外あり)
*libhal.a:gdma_hal_axi.*(.text .text.gdma_axi_hal_append ...) ← 個別列挙 (漏れあり)
                                                                  axi_dma_ll_* が未列挙!
→ orphan → 別セグメント → 合計 23 > max 16
```

**解決:** 上記「sections.ld の orphan セクション修正」を適用。

#### 3. `--use-segments` + `--elf-sha256-offset` 非互換

セグメント数問題の別の解決策として esptool の `--use-segments` フラグを試行。
セグメント数は解決するが、`--elf-sha256-offset 0xb0` と非互換で、
イメージヘッダの eFuse リビジョンフィールドにゴミデータが入り
`Image requires efuse blk rev >= v568.36` でブートローダーがアプリを拒否。

**結論:** `--use-segments` は使用不可。`sections.ld` 修正がが正解。

#### 4. `.dummy` ディレクトリの状態不整合

Hybrid Compile は `.dummy/` ディレクトリを CMake ソーステンプレートとして使用。
ビルド失敗後に不整合な状態で残ることがあり、再ビルドで
`'.dummy': unknown name` エラーが発生。

**解決:** `.dummy/` を削除してからリビルド。

### クリーンビルド手順

Hybrid Compile で問題が発生した場合の完全クリーン手順:

```powershell
# 全キャッシュ削除
Remove-Item -Recurse -Force .pio/build, .dummy, managed_components -ErrorAction SilentlyContinue
Remove-Item -Force sdkconfig.defaults -ErrorAction SilentlyContinue

# リビルド
pio run
```

## ドキュメント

- [パイプラインアーキテクチャ](docs/pipeline-architecture.md)
- [タスク間通信](docs/task-interactions.md)
- [MJPEG ストリーム分析](research/mjpeg-stream-analysis/FINDINGS.md)
