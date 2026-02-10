# DERP Relay スループット分析

## 概要

Tailscale VPN (WireGuard over DERP relay) 経由でのMJPEGストリーミングにおいて、
TCP Window Scalingを有効化しても期待されたスループット向上が得られなかった原因を分析する。

## 測定環境

- **デバイス**: M5Stack Tab5 (ESP32-P4, WiFi RSSI=-33 dBm)
- **カメラサーバー**: 100.85.69.54:8082 (Tailscale VPN IP)
- **接続方式**: DERP relay経由 (STUNプローブ失敗、直接接続不可)
- **フレームサイズ**: ~37KB/frame (640x480 MJPEG)

## TCP Window Scaling 設定

```
CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y
CONFIG_LWIP_TCP_WND_DEFAULT=524280    # 512KB (65535 << 3)
CONFIG_LWIP_WND_SCALE=y
CONFIG_LWIP_TCP_RCV_SCALE=3           # shift factor = 3
CONFIG_LWIP_TCP_SND_BUF_DEFAULT=65534 # send側はu16_t制限
CONFIG_LWIP_TCP_RECVMBOX_SIZE=128
FETCH_TCP_RCVBUF_BYTES=512*1024       # SO_RCVBUF
```

ソケット確認ログ:
```
Fetch socket: rcvbuf_req=524288(ok) rcvbuf_eff=524288(ok) rcvto=4ms(ok) nodelay=on
```

## 測定結果

### 安定時 (TCP_WND=524280, Window Scaling有効)

| 指標 | 値 |
|------|------|
| MJPEG FPS | 4.0-5.0 fps (平均 ~4.5) |
| スループット | 1.34-1.44 Mbps (平均 ~1.40) |
| フレームサイズ | 36,500-38,200 bytes |
| read/frame | 1,600-2,500 bytes |
| idle時間 | 5,000-9,000 us |
| エラー | 0 |

### 定期的ストール

20-40秒間隔で大きなスループット低下が発生:

| fps | mbps | idle (us) | 推定原因 |
|-----|------|-----------|----------|
| 0.5 | 0.15 | 120,655 | DERP relay遅延 |
| 1.0 | 0.28 | 36,880 | WG再ハンドシェイク？ |
| 1.5 | 0.44 | 25,991 | TCP再送？ |
| 2.0 | 0.54 | 29,015 | DERP再接続？ |

### 比較: TCP_WND=65534 (Window Scaling無効)

| 指標 | WND=65534 | WND=524280 | 差分 |
|------|-----------|------------|------|
| MJPEG FPS | ~4.5 | ~4.5 | **変化なし** |
| スループット | ~1.4 Mbps | ~1.4 Mbps | **変化なし** |

## 分析

### BDP理論値 vs 実測値

```
理論: throughput = TCP_WND / RTT

TCP_WND=65,534  RTT=385ms → 170 KB/s (1.36 Mbps) → 4.5 fps  ← 実測と一致
TCP_WND=524,280 RTT=385ms → 1.3 MB/s (10.4 Mbps) → 35 fps  ← 実測では到達せず
```

TCP_WND=65534での理論値と実測値が一致していることから、WND=65534時点では
Inner TCP (HTTP over WG) のウィンドウがボトルネックだった。

しかしWND=524280に拡大しても実測値が変わらないのは、
**Inner TCP以外の場所に ~1.4 Mbps の上限が存在する** ことを示す。

### データフロー全体像

```
Camera Server
    │ HTTP response (MJPEG frames)
    ▼
Camera-side Tailscale daemon
    │ WireGuard encapsulation (UDP packets)
    ▼
Camera-side Tailscale → DERP relay    ← Outer TCP #1 (TLS/TCP)
    │
    ▼
DERP relay
    │
    ▼
DERP relay → ESP32 Tailscale          ← Outer TCP #2 (TLS/TCP)
    │ microlink_derp.c: DERP frame receive
    ▼
ESP32 WireGuard decryption
    │ wireguardif_network_rx() → ip_input()
    ▼
ESP32 lwIP TCP stack
    │ Inner TCP reassembly
    ▼
FetchTask: socket recv()
```

### ボトルネック候補

#### 1. Outer TCP (ESP32 ↔ DERP relay) のウィンドウ制限

ESP32 → DERP relay 間のTLS/TCP接続は、mbedTLS のソケットAPIを使用。
この接続のTCPウィンドウもlwIPのデフォルト設定が適用される。

ただし、`CONFIG_LWIP_TCP_WND_DEFAULT` はグローバル設定なので、
Outer TCPにも524280が適用されているはず。

Outer TCP の RTT は DERP relay までの距離に依存。
日本からのDERP relay (東京 or ソウル) なら RTT ~20-50ms。

```
TCP_WND=524280, RTT=50ms → 10.5 MB/s → ボトルネックではない
```

#### 2. DERP relay 自体の帯域制限

Tailscale の DERP relay は帯域制限を設けている可能性がある。
公式ドキュメントでは具体的な制限値は公開されていないが、
リレー経由の通信は "best effort" とされている。

1.4 Mbps (~175 KB/s) はDERPの帯域制限である可能性が高い。

#### 3. DERP プロトコルのオーバーヘッド

DERPフレームは 1パケットずつ処理される。
各パケットにDERPヘッダー (32-byte peer key + 4-byte length) が付加。
WireGuard MTU = 1420 bytes の場合、ペイロード効率:

```
(1420 - 40 IP/UDP - 32 WG) / (1420 + 36 DERP overhead) ≈ 92.5%
```

プロトコルオーバーヘッドは ~7.5% で、スループット制限の主因ではない。

#### 4. tcpip_try_callback() のスケジューリング遅延

WireGuardパケットは `tcpip_try_callback()` でtcpipスレッド内で処理される。
コールバックキューの深さやスケジューリング遅延がボトルネックになる可能性。

ただし、mbps=1.4 でのパケットレートは:
```
1.4 Mbps / (1420 * 8) ≈ 123 packets/sec
```

tcpip_try_callback + xSemaphoreTake のレイテンシが 1ms だとしても
1000 packets/sec は処理可能。123 packets/sec では余裕がある。

### 結論

**最も可能性の高いボトルネック: DERP relay の帯域制限 (~1.4 Mbps)**

Inner TCP Window Scaling は正しく機能しているが、
データがDERP relay を通過する時点で ~1.4 Mbps に制限されている。

## 直接接続 (DERP bypass) の必要性

### 現在のSTUN状態

```
W (19541) ml_coord: No public endpoint discovered via STUN
```

STUNプローブが失敗しているため、NATホールパンチング (DISCO) も機能しない。

### 直接接続時の理論性能

直接UDP接続 (同一LAN or NAT traversal成功時):
```
RTT = 2-5ms (LAN) or 20-50ms (Internet)
WG overhead ≈ 92 bytes/packet
実効帯域: WiFi 802.11n (40MHz) → 100+ Mbps

→ 30 fps (1.4 MB/s) は十分達成可能
```

### 改善アプローチ

| 手法 | 期待FPS | 難易度 | リスク |
|------|---------|--------|--------|
| STUN修復 → NAT traversal | 30+ | 中 | NATタイプ依存 |
| 同一LANバイパス | 30+ | 低 | 同一LAN限定 |
| DERP帯域上限の調査・緩和 | 10-15? | 高 | Tailscale側の制約 |
| VPNなし直接HTTP | 30+ | 低 | セキュリティ低下 |

## Window Scaling の不安定性と HW 限界

### 発生した問題

`TCP_WND=524280` + `CONFIG_LWIP_WND_SCALE=y` (shift=3) でデバイスが不安定化。
ストリーミング中にデバイスが再起動する事象が発生した。

### 原因の推定

| 要因 | 詳細 |
|------|------|
| **pbuf メモリ圧迫** | TCP_WND=512KB 分の受信バッファを lwIP が確保しようとする。SPIRAM上のpbufは遅延が大きく、SDIO DMA との競合が発生しうる |
| **RECVMBOX_SIZE=128** | tcpip スレッドの受信メールボックスを128に拡大。メールボックスあふれ時の挙動が不安定 |
| **SPIRAM_TRY_ALLOCATE_WIFI_LWIP** | WiFi/lwIP バッファを SPIRAM に配置。SDIO ドライバが内部SRAM DMA を前提としている場合に問題 |
| **TCP再送の爆発** | 512KB ウィンドウで大量パケットロス時、再送キューが巨大化 |

### 安定動作が確認済みの設定

```ini
# 現在の保守的設定 (安定動作確認済み)
CONFIG_LWIP_TCP_WND_DEFAULT=65534    # u16_t上限、Window Scaling不要
CONFIG_LWIP_TCP_SND_BUF_DEFAULT=65534
CONFIG_LWIP_TCP_RECVMBOX_SIZE=32     # ESP-IDF標準値
# CONFIG_LWIP_WND_SCALE is not set
# CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP is not set
FETCH_TCP_RCVBUF_BYTES=64*1024
```

### 次回チューニング時のガイドライン

Window Scaling を再度試す場合は、以下の段階的アプローチで:

#### Step 1: SPIRAM pbuf のみ有効化 (Window Scaling なし)

```ini
CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y
CONFIG_LWIP_TCP_WND_DEFAULT=65534    # 変更なし
# CONFIG_LWIP_WND_SCALE is not set
```

SPIRAM pbuf だけで安定するか確認。これが不安定なら Window Scaling 以前の問題。

#### Step 2: 控えめな Window Scaling (128KB)

```ini
CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y
CONFIG_LWIP_TCP_WND_DEFAULT=131064   # 65532 << 1 (shift=1)
CONFIG_LWIP_WND_SCALE=y
CONFIG_LWIP_TCP_RCV_SCALE=1          # 最小のshift
CONFIG_LWIP_TCP_RECVMBOX_SIZE=64     # 段階的に増加
FETCH_TCP_RCVBUF_BYTES=128*1024
```

#### Step 3: 中間 (256KB)

```ini
CONFIG_LWIP_TCP_WND_DEFAULT=262120   # 65530 << 2 (shift=2)
CONFIG_LWIP_TCP_RCV_SCALE=2
CONFIG_LWIP_TCP_RECVMBOX_SIZE=64
FETCH_TCP_RCVBUF_BYTES=256*1024
```

#### Step 4: 最大 (512KB) — 前回不安定だった設定

```ini
CONFIG_LWIP_TCP_WND_DEFAULT=524280   # 65535 << 3 (shift=3)
CONFIG_LWIP_TCP_RCV_SCALE=3
CONFIG_LWIP_TCP_RECVMBOX_SIZE=128
FETCH_TCP_RCVBUF_BYTES=512*1024
```

### 注意: DERP 経由では効果なし

上記のチューニングは **直接接続 (STUN/DISCO成功時) で効果を発揮する前提**。
DERP relay 経由では TCP_WND=65534 でも 524280 でもスループットは ~1.4 Mbps で同一。
直接接続が確立できない限り、Window Scaling の効果は得られない。

## 参考: メモリ使用状況

保守的設定 (TCP_WND=65534) でのヒープ状態:

```
INTERNAL: 366,000 B free (357 KB) — 十分な余裕
SPIRAM:   26,457,804 B free (25.8 MB)
```
