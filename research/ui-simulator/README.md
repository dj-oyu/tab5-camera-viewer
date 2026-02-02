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
(検出リスト)            (ステータス)
```
