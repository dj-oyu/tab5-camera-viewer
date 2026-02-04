# Detection表示の安定性問題

## 問題の本質

**検出結果リストが「ある瞬間のフレーム」に強く依存している**

現在の実装では、APIから受信した最新の検出結果をそのまま表示している。
MLモデルの検出結果はフレームごとに変動するため、表示が不安定になる。

これは「視覚的なちらつき」の問題ではなく、**時間的な情報が失われている**という設計上の問題。

## 具体的な症状

### ケース1: 断続的な検出
```
time 0: [A]
time 1: []        ← Aが一時的に検出されない
time 2: [A]
time 3: [A, B]    ← Bが初めて検出
time 4: [A]       ← Bが一時的に検出されない
```

現在の実装: 表示が `A → 空 → A → A,B → A` と変化
期待する動作: Aは継続表示、Bも一度検出されたら一定時間表示を維持

### ケース2: 検出順序の変動
```
time 0: [A, B]
time 1: [B, A]    ← 順序が入れ替わる
time 2: [A, B]
```

表示リストの順序が毎フレーム変わると、ユーザーが追跡しづらい。

### ケース3: 同一ラベルの複数オブジェクト
```
time 0: [person, person]  ← 2人検出
time 1: [person]          ← 1人だけ検出
time 2: [person, person]
```

どの「person」が消えたのか、同じpersonが継続しているのか区別できない。

## 解決すべき課題

1. **時間的な安定化**: 一度検出されたオブジェクトを一定時間キープする
2. **散発的検出への対応**: 数フレーム検出されなくても表示を維持する
3. **同一ラベル複数対応**: 複数の同一ラベルオブジェクトを区別して追跡する（オプション）

## 解決アプローチ

### アプローチA: ラベルベースの永続化（シンプル）

各ラベルに「最終検出時刻」を付与し、タイムアウトまで表示を維持。

```
検出リスト = {}
タイムアウト = 500ms

新しい検出が来たら:
  for each detection in 新しい検出:
    検出リスト[detection.label].lastSeen = now
    検出リスト[detection.label].confidence = detection.confidence

表示時:
  for each entry in 検出リスト:
    if (now - entry.lastSeen) < タイムアウト:
      表示する
    else:
      削除する
```

**制限**: 同一ラベル複数オブジェクトを区別できない

### アプローチB: bbox位置ベースの追跡（正確）

bbox（境界ボックス）の位置で同一オブジェクトを識別。

```
追跡リスト = []

新しい検出が来たら:
  for each detection in 新しい検出:
    既存 = 追跡リストから近い位置の同ラベルを探す（IOU計算）
    if 既存が見つかった:
      既存.bbox = detection.bbox
      既存.confidence = detection.confidence
      既存.lastSeen = now
    else:
      追跡リストに追加
```

**必要な変更**: Detection構造体にbbox追加、DetectionTaskでbboxパース

### アプローチC: サーバー側トラッキング

検出APIサーバー側でオブジェクトトラッキングを実装し、
トラッキングIDを付与した安定した結果を返す。

**メリット**: クライアント側の実装がシンプルに
**デメリット**: サーバー側の変更が必要

## 現在の実装

```cpp
// OverlayRenderer.cpp
if (detectionData.tryRead(detections, &count, &timestamp)) {
  memcpy(cachedDetections, detections, sizeof(Detection) * count);
  cachedCount = count;  // ← 毎フレーム完全に置き換え
}
```

## 次のステップ

- [ ] アプローチを選択
- [ ] 実装
- [ ] UIシミュレータで検証
- [ ] 実機テスト
