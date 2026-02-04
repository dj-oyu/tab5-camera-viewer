# Detection Overlay UI Simulator

Tab5カメラビューワーのオーバーレイUIをブラウザでシミュレートするツール。

## 実行方法

```bash
cd research/ui-simulator
uv run app.py
```

ブラウザで http://localhost:5000 を開く。

## 機能

- 実際のDetection APIに接続してリアルタイム表示
- Mock Dataで任意の検出結果をテスト
- 実機と同じレイアウト（ランドスケープビュー）を再現

## 画面構成

```
+------+----------------+------+
| Left |    Video       | Right|
| Bar  |   960x720      | Bar  |
| 160px|                | 160px|
+------+----------------+------+
USB側                   カメラ側
(検出リスト)            (接続数)
```

## タイル構成

各バーは3つのタイル（各240px高）で構成:

```
Left Bar (検出結果)     Right Bar (ステータス)
┌─────────────┐         ┌─────────────┐
│ Tile 0      │         │ Tile 0      │
│ 検出 1-4    │         │ Conn: X     │
├─────────────┤         ├─────────────┤
│ Tile 1      │         │ Tile 1      │
│ 検出 5-8    │         │ (空)        │
├─────────────┤         ├─────────────┤
│ Tile 2      │         │ Tile 2      │
│ 検出 9-12   │         │ (空)        │
└─────────────┘         └─────────────┘
```

---

## 実機実装向けTips

### 1. タイルベース描画によるメモリ効率化

**原則**: 変更があったタイルのみを再描画する

```
タイル単位の更新判定:
- 検出数が変わった → 該当タイルのみ更新
- 検出数 0-4件  → Tile 0 のみ更新
- 検出数 5-8件  → Tile 0, 1 更新
- 検出数 9-12件 → 全タイル更新
```

**実装例** (擬似コード):
```cpp
// 前回の検出数を保持
static int prevDetectionCount = 0;

// 更新が必要なタイル範囲を計算
int prevTiles = (prevDetectionCount + 3) / 4;  // 切り上げ
int currTiles = (currentCount + 3) / 4;
int tilesToUpdate = max(prevTiles, currTiles);

// 必要なタイルのみ再描画
for (int i = 0; i < tilesToUpdate; i++) {
    renderTile(i, detections);
}
```

### 2. 信頼度のメーター表示

数値表示の代わりに背景バーで信頼度を視覚化:

```
┌────────────────────────┐
│██████████░░░░│ person  │  ← 緑バー = 信頼度
└────────────────────────┘
```

**利点**:
- 数値より直感的に把握可能
- フォント描画コスト削減（％表示不要）
- 小さい画面でも視認性良好

**実装**:
```cpp
// メーター幅 = タイル幅 × 信頼度
int meterWidth = (int)(TILE_WIDTH * confidence);
sprite.fillRect(0, y, meterWidth, ROW_HEIGHT, TFT_DARKGREEN);
sprite.drawString(label, PADDING, y + TEXT_OFFSET);
```

### 3. 均等間隔レイアウトの計算

タイル境界をまたいでも均等な間隔を維持する:

```
タイル高: 240px
行数: 4
行高: 48px
行間: 12px

計算式:
padding_top + (row_height × 4) + (gap × 3) + padding_bottom = 240px
6px + (48px × 4) + (12px × 3) + 6px = 240px ✓

タイル間の間隔:
Tile0_bottom_padding + Tile1_top_padding = 6px + 6px = 12px = 行間と同じ
```

### 4. 検出タイムアウト処理

検出データが一定時間来ない場合のリセット:

```cpp
#define DETECTION_TIMEOUT_MS 3000

// keepalive受信時にチェック
if (millis() - lastDetectionTime > DETECTION_TIMEOUT_MS) {
    clearDetections();
    // タイルをクリア（黒塗り）
}
```

**理由**:
- APIはkeepaliveを送り続けるが、検出がないとデータは来ない
- 古い検出結果を表示し続けるのは誤解を招く
- 3秒程度が「検出なし」と判断する合理的な待ち時間

### 5. 空タイルの最適化

コンテンツがないタイルは描画をスキップ:

```cpp
// 右バー: Tile 1, 2 は常に空
// → 初期化時に黒塗りして以降は触らない

// 左バー: 検出数に応じて
if (tileHasContent[i]) {
    renderTile(i);
} else if (tileWasCleared[i] == false) {
    clearTile(i);  // 一度だけクリア
    tileWasCleared[i] = true;
}
```

### 6. フォント・色の選択

**推奨設定**:
```cpp
// ラベル: 太字、白、影付き（視認性確保）
sprite.setTextColor(TFT_WHITE);
sprite.setTextSize(2);  // または適切なフォントサイズ

// メーター: 緑系グラデーション（不透明度70%程度）
uint16_t meterColor = sprite.color565(0x22, 0x88, 0x22);

// 背景: 暗いグレー（完全黒より視認性向上）
uint16_t bgColor = sprite.color565(0x11, 0x11, 0x11);
```

### 7. Sprite描画の最適化

```cpp
// 1. 内部RAMにSpriteを確保（高速）
sprite.createSprite(TILE_WIDTH, TILE_HEIGHT);
sprite.setPsram(false);  // PSRAMより内部RAM優先

// 2. 一括描画してからpushSprite
sprite.fillSprite(BG_COLOR);
// ... 描画処理 ...
sprite.pushSprite(x, y);  // 一度だけ転送

// 3. 不要なSprite削除
sprite.deleteSprite();  // メモリ解放
```

---

## API仕様

### Detection API
- エンドポイント: `/api/detections/stream?format=json`
- 形式: SSE (Server-Sent Events)
- データ: `{"detections": [{"class_name": "person", "confidence": 0.95}, ...]}`
- Keepalive: `: ping` (コメント行)

### Connections API
- エンドポイント: `/api/connections/stream`
- 形式: SSE
- データ: `{"webrtc": 1, "mjpeg": 2, "total": 3}`
