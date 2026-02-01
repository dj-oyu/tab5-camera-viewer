# MJPEG Stream Analysis Findings

## 調査日時
2026-02-01

## エンドポイント
`http://192.168.1.33:8082/stream`

## HTTP Response Headers

```
HTTP/1.1 200 OK
Cache-Control: no-cache
Content-Type: multipart/x-mixed-replace; boundary=frame
Transfer-Encoding: chunked
```

## MJPEG Part Structure

```
--frame\r\n
Content-Type: image/jpeg\r\n
Content-Length: NNNNN\r\n
\r\n
<JPEG binary data>
```

- Boundary: `--frame`
- Content-Length: **純粋なJPEGバイナリサイズ**（チャンク情報含まず）

## Chunked Transfer Encoding

### 発見: 1フレームは複数チャンクに分割される

Raw dump example:
```
Offset    Content
--------  -------
00000000  "800\r\n"                    ← Chunk 1 size (2048 bytes)
00000005  "--frame\r\n..."             ← Part header start
00000041  0xFFD8                       ← JPEG SOI marker
00000807  "ef82\r\n"                   ← Chunk 2 size (61314 bytes)
          <JPEG data continues>
```

### チャンク分割パターン

- Content-Length: 63302 bytes
- Chunk 1: 2048 bytes (ヘッダー + JPEG先頭)
- Chunk 2: ~61314 bytes (JPEG残り)

## Content-Length vs Actual Size

| Frame | Content-Length | Actual Size | Match |
|-------|----------------|-------------|-------|
| 0     | 63352          | 63352       | ✓     |
| 1     | 63288          | 63288       | ✓     |
| ...   | ...            | ...         | ✓     |

**結論**: Content-Length = 純粋なJPEGサイズ（SOI〜EOI）

## マイコン実装への示唆

### 必須要件
1. **ChunkedParser継続必要**: チャンク境界を処理してデチャンクする
2. **Content-Length活用可能**: JPEGサイズを事前把握してバッファ確保

### 推奨実装パターン

```
[Stream] ─recv→ [recv_buf 4KB] ─ChunkedParser→ [dechunked data]
                                                    │
                         MjpegParser: ヘッダー解析 → Content-Length取得
                                                    │
                         Content-Length分を → [linear_buf 256KB]
                                                    │
                                              フレーム送信
```

### 処理フロー

1. `recv_buf`にストリームから読み込み
2. `ChunkedParser`でデチャンク
   - チャンクサイズ行をスキップ
   - チャンク終端CRLFをスキップ
   - ペイロードバイトのみを次段へ
3. `MjpegParser`でパートヘッダーを解析
   - `--frame` boundary検出
   - `Content-Length: N` 抽出
   - 空行でヘッダー終了
4. Content-Length分のデータをリニアバッファにコピー
5. SOI/EOIマーカー検索は不要

### メモリ節約

- 旧: リングバッファ 1MB
- 新: recv_buf 4KB + linear_buf 256KB×2
- 節約: ~500KB

### 注意点

- チャンク境界がパートヘッダーの途中に来る可能性
- チャンク境界がJPEGデータの途中に来る（確認済み）
- → バイト単位でChunkedParserを通す必要あり

---

## JPEG Decode Test Results

### テスト条件
- フレーム数: 5
- デコーダー: Pillow (Python)

### 結果

| Frame | Content-Length | SOI | EOI | Decode | Size |
|-------|----------------|-----|-----|--------|------|
| 0 | 63626 | ✓ | ✓ | ✓ | 640x480 |
| 1 | 63586 | ✓ | ✓ | ✓ | 640x480 |
| 2 | 63598 | ✓ | ✓ | ✓ | 640x480 |
| 3 | 63618 | ✓ | ✓ | ✓ | 640x480 |
| 4 | 63489 | ✓ | ✓ | ✓ | 640x480 |

### 結論

- **Content-Lengthは信頼できる** - 純粋なJPEGサイズを正確に示す
- **SOI/EOIマーカーは正常** - 全フレームで正しい位置
- **デコード成功率: 100%** - Content-Length分読み取ればデコード可能

### マイコン実装への適用

1. Content-LengthでJPEGサイズを事前把握
2. リニアバッファにContent-Length分のデータをコピー
3. EOIマーカー検索は不要（Content-Lengthで終端が分かる）
4. HW JPEGデコーダーに渡す

---

## Buffer Size Analysis (100 frames)

### JPEG Frame Size

| 項目 | 値 |
|------|-----|
| Min | 57,038 bytes |
| Max | 65,831 bytes |
| Mean | 64,466 bytes |
| 99%ile | 65,831 bytes |

### MJPEG Part Header

- 固定 **60 bytes**
- `--frame\r\nContent-Type: image/jpeg\r\nContent-Length: NNNNN\r\n\r\n`

### HTTP Chunk Pattern

- 常に **2チャンク** per frame
- Chunk 1: 2,048 bytes (ヘッダー + JPEG先頭)
- Chunk 2: ~62KB (JPEG残り)

### 推奨バッファサイズ

```cpp
constexpr uint32_t LINEAR_BUF_SIZE = 83558;  // 82KB (max + 20%margin)
constexpr uint32_t HEADER_BUF_SIZE = 64;     // 60B actual + padding
```

### メモリ節約

| 構成 | サイズ |
|------|--------|
| 現在 | 1,536 KB (256KB×2 + 1MB ring) |
| 最適化後 | 163 KB (82KB×2) |
| **節約** | **1,373 KB (89%)** |

---

## Pipeline Efficiency Analysis

### テスト結果 (20 frames)

| Buffers | Memory | FPS | Fetch(ms) | Decode(ms) | Wait(ms) | Waits |
|---------|--------|-----|-----------|------------|----------|-------|
| 2 | 163 KB | 195 | 0.2 | 4.2 | 4.87 | 62 |
| 3 | 245 KB | 1428 | 0.2 | 0.1 | 0.43 | 6 |
| 4 | 326 KB | 1715 | 0.3 | 0.1 | 0.29 | 4 |

### 分析

- **2バッファ**: 競合多発（fetch待ちがdecodeをブロック）
- **3バッファ**: fetch/decode/render並行で劇的改善
- **4バッファ**: さらに改善するがメモリ増加

### 推奨構成

```
             ┌─────────┐
[Stream] ──→ │ Buf[0]  │ ──→ [Decode]
             │ Buf[1]  │ ──→ [Render]
             │ Buf[2]  │ ← free
             └─────────┘

- 3バッファ構成 (245 KB)
- fetch/decode/renderの3ステージが並行動作可能
- 待機時間を最小化
```

### マイコン実装への提案

```cpp
// 最適化された設定
constexpr uint32_t LINEAR_BUF_SIZE = 83558;  // 82KB
constexpr int LINEAR_BUF_COUNT = 3;          // トリプルバッファ
constexpr uint32_t HEADER_BUF_SIZE = 64;     // スタック上

// メモリ使用量: 82KB × 3 = 246KB
// (現在の1.5MBから84%削減)
```

---

## 参考実装

`reference_impl.py` - マイコン実装の参考用Pythonコード

### 状態マシン

```
CHUNK_SIZE → MJPEG_HEADER → JPEG_BODY → CHUNK_TRAILER
     ↑                          │              │
     └──────────────────────────┴──────────────┘
```

### 重要な状態遷移

1. **JPEG_BODY完了時** (jpeg_remaining == 0):
   - chunk_remaining > 0 → MJPEG_HEADER (次フレームのヘッダー開始)
   - chunk_remaining == 0 → CHUNK_TRAILER

2. **ゼロコピー原則**:
   - JPEG_BODY: linear_bufに直接write
   - その他: header_bufで処理後、上書き破棄
