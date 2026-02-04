# Go統合ストリーミングサーバー設計書

**Version**: 1.0
**Date**: 2025-12-26
**Status**: 設計中

---

## 目次

1. [概要](#概要)
2. [背景と動機](#背景と動機)
3. [アーキテクチャ](#アーキテクチャ)
4. [技術スタック](#技術スタック)
5. [主要コンポーネント設計](#主要コンポーネント設計)
6. [実装詳細](#実装詳細)
7. [バックプレッシャー戦略](#バックプレッシャー戦略)
8. [モニタリング・メトリクス](#モニタリングメトリクス)
9. [プロファイリング](#プロファイリング)
10. [HTTP API仕様](#http-api仕様)
11. [パフォーマンス目標](#パフォーマンス目標)
12. [実装計画](#実装計画)
13. [移行戦略](#移行戦略)

---

## 概要

PythonベースのWebRTC配信 + 録画サーバーを、**単一のGoバイナリ**に統合し、効率性とメンテナンス性を向上させる。

### 主要機能

- **WebRTC H.264配信**: H.264 passthrough、ゼロコピー、低遅延
- **H.264録画**: SPS/PPS付き正しいファイル生成
- **HTTP API**: 録画制御、WebRTCシグナリング
- **リソース効率**: メモリ使用量 <20MB、CPU使用率 <10%

### 達成目標

| メトリクス | Python版 | Go版目標 | 改善率 |
|-----------|---------|---------|-------|
| メモリ使用量 | ~110MB (2プロセス) | **<20MB (1バイナリ)** | 82%削減 |
| CPU使用率 | ~25% (デコード/再エンコード) | **<10% (passthrough)** | 60%削減 |
| 起動時間 | ~3秒 | **<500ms** | 83%改善 |
| デプロイサイズ | ~300MB (Python + 依存) | **<15MB (静的バイナリ)** | 95%削減 |
| 遅延 | ~200ms | **<100ms** | 50%改善 |

---

## 背景と動機

### 現状の課題（Python実装）

#### 1. WebRTC配信の非効率性
```python
# h264_track.py:176-186
H.264 (共有メモリ) → PyAV decode → VideoFrame → aiortc re-encode → WebRTC
```
- **問題**: デコード + 再エンコードでCPU 2倍の負荷
- **原因**: aiortcのH.264 passthrough APIが未成熟

#### 2. SPS/PPS欠損問題
```bash
$ ffprobe recording.h264
[h264] non-existing PPS 0 referenced
[h264] decode_slice_header error
```
- **問題**: 途中から録画開始するとSPS/PPSヘッダーが欠ける
- **影響**: VLC/ffplayで再生不可、WebRTC接続失敗の可能性

#### 3. リソース効率の悪さ
```
python3 h264_recorder.py     50MB RAM
python3 webrtc_server.py     60MB RAM
python3 web_monitor.py       40MB RAM
------------------------------------------
合計                        150MB RAM, 3プロセス
```

#### 4. デプロイの複雑さ
```bash
# Python依存関係
aiortc>=1.9.0
av>=12.0.0
flask>=3.0.0
opencv-python>=4.8.0
numpy>=1.24.0
# → 300MB以上のディスク容量
```

### Goによる解決

```
┌─────────────────────────────────────────────┐
│  streaming-server (単一バイナリ 15MB)        │
├─────────────────────────────────────────────┤
│  ✅ H.264 passthrough (ゼロコピー)            │
│  ✅ SPS/PPS自動検出・キャッシュ              │
│  ✅ メモリ使用量 <20MB                       │
│  ✅ 依存関係なし (静的リンク)                │
└─────────────────────────────────────────────┘
```

---

## アーキテクチャ

### システム全体図

```
┌──────────────────────────────────────────────────────────┐
│                   Camera Daemon (C)                       │
│  - D-Robotics libspcdev                                  │
│  - H.264 hardware encoding (30fps @ 8Mbps)               │
└───────────────────┬──────────────────────────────────────┘
                    │ H.264 NAL units
                    ▼
    /pet_camera_stream (POSIX shared memory)
                    │
                    ▼
┌──────────────────────────────────────────────────────────┐
│          Go Streaming Server (streaming-server)          │
├──────────────────────────────────────────────────────────┤
│                                                           │
│  ┌────────────────────────────────────────────────────┐ │
│  │  Shared Memory Reader (cgo)                        │ │
│  │  - shm_open() / mmap()                             │ │
│  │  - Ring buffer read (atomic operations)            │ │
│  └────────────┬───────────────────────────────────────┘ │
│               │                                           │
│               ▼                                           │
│  ┌────────────────────────────────────────────────────┐ │
│  │  H264 Stream Processor                             │ │
│  │  - NAL unit parser                                 │ │
│  │  - SPS/PPS detector & cache                        │ │
│  │  - Keyframe detection                              │ │
│  └────────┬───────────────────┬───────────────────────┘ │
│           │                   │                           │
│           │                   │                           │
│  ┌────────▼────────┐  ┌──────▼──────────────────────┐   │
│  │  WebRTC Server  │  │  H264 Recorder              │   │
│  │  (pion/webrtc)  │  │  - File writer              │   │
│  │                 │  │  - SPS/PPS prepend          │   │
│  │  - H.264 Track  │  │  - .h264 / .mp4 output      │   │
│  │  - RTP pkt      │  └─────────────────────────────┘   │
│  │  - Signaling    │                                     │
│  └────────┬────────┘                                     │
│           │                                               │
│  ┌────────▼────────────────────────────────────────────┐ │
│  │  HTTP API Server (net/http)                        │ │
│  │  - POST /api/webrtc/offer                          │ │
│  │  - POST /api/recording/start                       │ │
│  │  - POST /api/recording/stop                        │ │
│  │  - GET  /api/recording/status                      │ │
│  └────────────────────────────────────────────────────┘ │
│                                                           │
└───────────────────────┬───────────────────────────────────┘
                        │ HTTP/WebRTC
                        ▼
            Browser (WebRTC Client + Canvas Overlay)
```

### データフロー

```
[Camera Daemon]
      │
      │ sp_encoder_get_stream()
      ▼
  H.264 NAL units (Annex-B format)
      │
      │ shm_frame_buffer_write()
      ▼
[POSIX Shared Memory: /pet_camera_stream]
      │
      │ mmap() + atomic read
      ▼
[Go: Shared Memory Reader (cgo)]
      │
      │ NAL unit parsing
      ▼
[Go: H264StreamProcessor]
      │
      ├─→ Detect SPS (type 7) → Cache
      ├─→ Detect PPS (type 8) → Cache
      └─→ Detect IDR (type 5) → Prepend SPS+PPS
      │
      ├─────────────────┬──────────────────┐
      │                 │                  │
      ▼                 ▼                  ▼
[WebRTC Track]   [H264 Recorder]   [Future: Analytics]
      │                 │
      │ RTP             │ File I/O
      ▼                 ▼
   Browser        recording.h264
```

---

## 技術スタック

### 1. Go標準ライブラリ

| パッケージ | 用途 |
|----------|------|
| `net/http` | HTTP API サーバー |
| `sync` | Goroutine同期 (Mutex, WaitGroup) |
| `context` | Goroutine lifetime管理 |
| `os` | ファイルI/O、シグナル処理 |
| `time` | タイマー、フレームレート制御 |
| `encoding/json` | JSON API |

### 2. 外部ライブラリ

| ライブラリ | バージョン | 用途 |
|----------|----------|------|
| [pion/webrtc](https://github.com/pion/webrtc) | v3.2.x | WebRTC実装 |
| [pion/rtp](https://github.com/pion/rtp) | v1.8.x | RTP packetization |
| [pion/interceptor](https://github.com/pion/interceptor) | v0.1.x | RTCP処理 |

### 3. cgoによるC統合

```go
/*
#cgo LDFLAGS: -lrt
#include <sys/mman.h>
#include <fcntl.h>
#include <stdint.h>
#include "../../capture/shared_memory.h"

// C関数のラッパー
extern SharedMemory* shm_frame_buffer_open(const char *name);
extern int shm_frame_buffer_read(SharedMemory *shm, Frame *frame);
extern void shm_frame_buffer_close(SharedMemory *shm);
*/
import "C"
```

### 4. ビルド設定

```go
// go.mod
module github.com/smart-pet-camera/streaming-server

go 1.21

require (
    github.com/pion/webrtc/v3 v3.2.24
    github.com/pion/rtp v1.8.3
    github.com/pion/interceptor v0.1.25
)

// CGO設定
// #cgo CFLAGS: -I${SRCDIR}/../../capture
// #cgo LDFLAGS: -L${SRCDIR}/../../build -lrt
```

---

## 主要コンポーネント設計

### 1. SharedMemoryReader (cgo)

**責務**: POSIX共有メモリからH.264フレームを読み取る

```go
package shm

/*
#cgo LDFLAGS: -lrt
#include "../../capture/shared_memory.h"
*/
import "C"
import (
    "errors"
    "unsafe"
)

type SharedMemoryReader struct {
    name    string
    cShm    *C.SharedMemory
    closed  bool
}

func NewSharedMemoryReader(name string) (*SharedMemoryReader, error) {
    cName := C.CString(name)
    defer C.free(unsafe.Pointer(cName))

    cShm := C.shm_frame_buffer_open(cName)
    if cShm == nil {
        return nil, errors.New("failed to open shared memory")
    }

    return &SharedMemoryReader{
        name:   name,
        cShm:   cShm,
        closed: false,
    }, nil
}

func (r *SharedMemoryReader) ReadFrame() (*Frame, error) {
    if r.closed {
        return nil, errors.New("shared memory closed")
    }

    var cFrame C.Frame
    ret := C.shm_frame_buffer_read(r.cShm, &cFrame)
    if ret != 0 {
        return nil, errors.New("failed to read frame")
    }

    // CのFrame構造体をGoのFrame構造体に変換
    frame := &Frame{
        FrameNumber: uint64(cFrame.frame_number),
        Timestamp:   timespecToTime(cFrame.timestamp),
        CameraID:    int(cFrame.camera_id),
        Width:       int(cFrame.width),
        Height:      int(cFrame.height),
        Format:      FrameFormat(cFrame.format),
        DataSize:    int(cFrame.data_size),
        Data:        C.GoBytes(unsafe.Pointer(&cFrame.data[0]), C.int(cFrame.data_size)),
    }

    return frame, nil
}

func (r *SharedMemoryReader) Close() error {
    if !r.closed {
        C.shm_frame_buffer_close(r.cShm)
        r.closed = true
    }
    return nil
}

type Frame struct {
    FrameNumber uint64
    Timestamp   time.Time
    CameraID    int
    Width       int
    Height      int
    Format      FrameFormat
    DataSize    int
    Data        []byte
}

type FrameFormat int

const (
    FormatJPEG FrameFormat = 0
    FormatNV12 FrameFormat = 1
    FormatRGB  FrameFormat = 2
    FormatH264 FrameFormat = 3
)
```

---

### 2. H264StreamProcessor

**責務**: NAL unit解析、SPS/PPS検出・キャッシュ

```go
package h264

import (
    "bytes"
    "errors"
)

// NAL unit types (H.264 spec)
const (
    NALTypeUnspecified    = 0
    NALTypeSlice          = 1  // Non-IDR slice
    NALTypeIDR            = 5  // IDR slice (keyframe)
    NALTypeSEI            = 6  // Supplemental Enhancement Information
    NALTypeSPS            = 7  // Sequence Parameter Set
    NALTypePPS            = 8  // Picture Parameter Set
    NALTypeAUD            = 9  // Access Unit Delimiter
)

type StreamProcessor struct {
    spsCache []byte
    ppsCache []byte
    hasHeader bool
}

func NewStreamProcessor() *StreamProcessor {
    return &StreamProcessor{
        hasHeader: false,
    }
}

// ProcessFrame processes a raw H.264 frame and returns NAL units with headers if needed
func (p *StreamProcessor) ProcessFrame(data []byte) ([]byte, error) {
    if len(data) < 5 {
        return nil, errors.New("frame too short")
    }

    nalUnits := p.parseNALUnits(data)
    if len(nalUnits) == 0 {
        return nil, errors.New("no NAL units found")
    }

    // Check first NAL unit type
    firstNALType := nalUnits[0][4] & 0x1F

    switch firstNALType {
    case NALTypeSPS:
        // Cache SPS
        p.spsCache = make([]byte, len(nalUnits[0]))
        copy(p.spsCache, nalUnits[0])
        return data, nil

    case NALTypePPS:
        // Cache PPS
        p.ppsCache = make([]byte, len(nalUnits[0]))
        copy(p.ppsCache, nalUnits[0])
        p.hasHeader = true
        return data, nil

    case NALTypeIDR:
        // IDR frame: prepend SPS+PPS if available
        if p.hasHeader {
            result := make([]byte, 0, len(p.spsCache)+len(p.ppsCache)+len(data))
            result = append(result, p.spsCache...)
            result = append(result, p.ppsCache...)
            result = append(result, data...)
            return result, nil
        }
        return data, nil

    default:
        // Non-IDR frame
        return data, nil
    }
}

// parseNALUnits splits Annex-B format into individual NAL units
// Annex-B format: 0x00 0x00 0x00 0x01 [NAL] 0x00 0x00 0x00 0x01 [NAL] ...
func (p *StreamProcessor) parseNALUnits(data []byte) [][]byte {
    var nalUnits [][]byte
    startCode := []byte{0x00, 0x00, 0x00, 0x01}

    start := 0
    for {
        idx := bytes.Index(data[start+4:], startCode)
        if idx == -1 {
            // Last NAL unit
            nalUnits = append(nalUnits, data[start:])
            break
        }

        idx += start + 4
        nalUnits = append(nalUnits, data[start:idx])
        start = idx
    }

    return nalUnits
}

func (p *StreamProcessor) HasHeader() bool {
    return p.hasHeader
}

func (p *StreamProcessor) GetSPS() []byte {
    return p.spsCache
}

func (p *StreamProcessor) GetPPS() []byte {
    return p.ppsCache
}
```

---

### 3. WebRTC Server (pion/webrtc)

**責務**: WebRTC peer connection管理、H.264トラック配信

```go
package webrtc

import (
    "context"
    "encoding/json"
    "fmt"
    "io"
    "log"
    "sync"
    "time"

    "github.com/pion/webrtc/v3"
    "github.com/pion/webrtc/v3/pkg/media"
    "github.com/smart-pet-camera/streaming-server/h264"
    "github.com/smart-pet-camera/streaming-server/shm"
)

type Server struct {
    shmReader     *shm.SharedMemoryReader
    h264Processor *h264.StreamProcessor

    // Peer connections
    peerConnections map[string]*webrtc.PeerConnection
    tracks          map[string]*webrtc.TrackLocalStaticSample
    mu              sync.RWMutex

    // WebRTC API
    api *webrtc.API
}

func NewServer(shmReader *shm.SharedMemoryReader) (*Server, error) {
    // Create WebRTC API with media engine
    mediaEngine := &webrtc.MediaEngine{}

    // Register H.264 codec
    if err := mediaEngine.RegisterCodec(webrtc.RTPCodecParameters{
        RTPCodecCapability: webrtc.RTPCodecCapability{
            MimeType:     webrtc.MimeTypeH264,
            ClockRate:    90000,
            Channels:     0,
            SDPFmtpLine:  "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f",
        },
        PayloadType: 96,
    }, webrtc.RTPCodecTypeVideo); err != nil {
        return nil, err
    }

    api := webrtc.NewAPI(webrtc.WithMediaEngine(mediaEngine))

    return &Server{
        shmReader:       shmReader,
        h264Processor:   h264.NewStreamProcessor(),
        peerConnections: make(map[string]*webrtc.PeerConnection),
        tracks:          make(map[string]*webrtc.TrackLocalStaticSample),
        api:             api,
    }, nil
}

// HandleOffer handles WebRTC offer from client
func (s *Server) HandleOffer(offerJSON []byte) ([]byte, error) {
    var offer webrtc.SessionDescription
    if err := json.Unmarshal(offerJSON, &offer); err != nil {
        return nil, fmt.Errorf("failed to parse offer: %w", err)
    }

    // Create peer connection
    config := webrtc.Configuration{
        ICEServers: []webrtc.ICEServer{
            {URLs: []string{"stun:stun.l.google.com:19302"}},
        },
    }

    pc, err := s.api.NewPeerConnection(config)
    if err != nil {
        return nil, fmt.Errorf("failed to create peer connection: %w", err)
    }

    // Create H.264 video track
    track, err := webrtc.NewTrackLocalStaticSample(
        webrtc.RTPCodecCapability{MimeType: webrtc.MimeTypeH264},
        "video",
        "pet-camera",
    )
    if err != nil {
        return nil, fmt.Errorf("failed to create track: %w", err)
    }

    // Add track to peer connection
    rtpSender, err := pc.AddTrack(track)
    if err != nil {
        return nil, fmt.Errorf("failed to add track: %w", err)
    }

    // Read RTCP packets (for feedback)
    go func() {
        rtcpBuf := make([]byte, 1500)
        for {
            if _, _, err := rtpSender.Read(rtcpBuf); err != nil {
                return
            }
        }
    }()

    // Set remote description (offer)
    if err := pc.SetRemoteDescription(offer); err != nil {
        return nil, fmt.Errorf("failed to set remote description: %w", err)
    }

    // Create answer
    answer, err := pc.CreateAnswer(nil)
    if err != nil {
        return nil, fmt.Errorf("failed to create answer: %w", err)
    }

    // Set local description (answer)
    if err := pc.SetLocalDescription(answer); err != nil {
        return nil, fmt.Errorf("failed to set local description: %w", err)
    }

    // Store peer connection and track
    pcID := fmt.Sprintf("pc-%d", time.Now().UnixNano())
    s.mu.Lock()
    s.peerConnections[pcID] = pc
    s.tracks[pcID] = track
    s.mu.Unlock()

    // Handle connection state changes
    pc.OnConnectionStateChange(func(state webrtc.PeerConnectionState) {
        log.Printf("[WebRTC] Connection %s state: %s", pcID, state.String())
        if state == webrtc.PeerConnectionStateFailed || state == webrtc.PeerConnectionStateClosed {
            s.removeConnection(pcID)
        }
    })

    // Return answer as JSON
    answerJSON, err := json.Marshal(answer)
    if err != nil {
        return nil, fmt.Errorf("failed to marshal answer: %w", err)
    }

    log.Printf("[WebRTC] Created peer connection %s", pcID)
    return answerJSON, nil
}

// Start starts the WebRTC streaming loop
func (s *Server) Start(ctx context.Context) error {
    ticker := time.NewTicker(33 * time.Millisecond) // ~30 fps
    defer ticker.Stop()

    for {
        select {
        case <-ctx.Done():
            return ctx.Err()
        case <-ticker.C:
            if err := s.processFrame(); err != nil {
                log.Printf("[WebRTC] Error processing frame: %v", err)
            }
        }
    }
}

func (s *Server) processFrame() error {
    // Read frame from shared memory
    frame, err := s.shmReader.ReadFrame()
    if err != nil {
        return err
    }

    // Check format
    if frame.Format != shm.FormatH264 {
        return fmt.Errorf("invalid frame format: %d", frame.Format)
    }

    // Process H.264 data (add SPS/PPS if needed)
    processedData, err := s.h264Processor.ProcessFrame(frame.Data)
    if err != nil {
        return err
    }

    // Broadcast to all tracks
    s.mu.RLock()
    defer s.mu.RUnlock()

    for pcID, track := range s.tracks {
        if err := track.WriteSample(media.Sample{
            Data:     processedData,
            Duration: 33 * time.Millisecond,
        }); err != nil && err != io.ErrClosedPipe {
            log.Printf("[WebRTC] Failed to write sample to %s: %v", pcID, err)
        }
    }

    return nil
}

func (s *Server) removeConnection(pcID string) {
    s.mu.Lock()
    defer s.mu.Unlock()

    if pc, ok := s.peerConnections[pcID]; ok {
        pc.Close()
        delete(s.peerConnections, pcID)
        delete(s.tracks, pcID)
        log.Printf("[WebRTC] Removed connection %s", pcID)
    }
}

func (s *Server) Close() error {
    s.mu.Lock()
    defer s.mu.Unlock()

    for pcID, pc := range s.peerConnections {
        pc.Close()
        log.Printf("[WebRTC] Closed connection %s", pcID)
    }

    return nil
}
```

---

### 4. H264Recorder

**責務**: H.264ファイルへの録画

```go
package recorder

import (
    "fmt"
    "os"
    "path/filepath"
    "sync"
    "time"

    "github.com/smart-pet-camera/streaming-server/h264"
    "github.com/smart-pet-camera/streaming-server/shm"
)

type Recorder struct {
    outputDir     string
    h264Processor *h264.StreamProcessor

    // Recording state
    recording     bool
    file          *os.File
    filename      string
    frameCount    int
    bytesWritten  int64
    startTime     time.Time
    mu            sync.Mutex
}

func NewRecorder(outputDir string, h264Processor *h264.StreamProcessor) (*Recorder, error) {
    if err := os.MkdirAll(outputDir, 0755); err != nil {
        return nil, fmt.Errorf("failed to create output directory: %w", err)
    }

    return &Recorder{
        outputDir:     outputDir,
        h264Processor: h264Processor,
        recording:     false,
    }, nil
}

func (r *Recorder) StartRecording(filename string) (string, error) {
    r.mu.Lock()
    defer r.mu.Unlock()

    if r.recording {
        return "", fmt.Errorf("already recording")
    }

    if filename == "" {
        filename = fmt.Sprintf("recording_%s.h264", time.Now().Format("20060102_150405"))
    }

    filepath := filepath.Join(r.outputDir, filename)
    file, err := os.Create(filepath)
    if err != nil {
        return "", fmt.Errorf("failed to create file: %w", err)
    }

    r.file = file
    r.filename = filepath
    r.recording = true
    r.frameCount = 0
    r.bytesWritten = 0
    r.startTime = time.Now()

    fmt.Printf("[Recorder] Started recording: %s\n", filepath)
    return filepath, nil
}

func (r *Recorder) StopRecording() (*RecordingStats, error) {
    r.mu.Lock()
    defer r.mu.Unlock()

    if !r.recording {
        return nil, fmt.Errorf("not recording")
    }

    if err := r.file.Close(); err != nil {
        return nil, fmt.Errorf("failed to close file: %w", err)
    }

    duration := time.Since(r.startTime)
    stats := &RecordingStats{
        Filename:     r.filename,
        FrameCount:   r.frameCount,
        BytesWritten: r.bytesWritten,
        Duration:     duration,
    }

    r.recording = false
    r.file = nil

    fmt.Printf("[Recorder] Stopped recording: %s (%d frames, %d bytes, %.1fs)\n",
        r.filename, r.frameCount, r.bytesWritten, duration.Seconds())

    return stats, nil
}

func (r *Recorder) WriteFrame(frame *shm.Frame) error {
    r.mu.Lock()
    defer r.mu.Unlock()

    if !r.recording {
        return nil // Silently ignore if not recording
    }

    // Process frame (add SPS/PPS to IDR frames)
    processedData, err := r.h264Processor.ProcessFrame(frame.Data)
    if err != nil {
        return fmt.Errorf("failed to process frame: %w", err)
    }

    // Write to file
    n, err := r.file.Write(processedData)
    if err != nil {
        return fmt.Errorf("failed to write frame: %w", err)
    }

    r.frameCount++
    r.bytesWritten += int64(n)

    return nil
}

func (r *Recorder) IsRecording() bool {
    r.mu.Lock()
    defer r.mu.Unlock()
    return r.recording
}

func (r *Recorder) GetStats() *RecordingStats {
    r.mu.Lock()
    defer r.mu.Unlock()

    if !r.recording {
        return nil
    }

    return &RecordingStats{
        Filename:     r.filename,
        FrameCount:   r.frameCount,
        BytesWritten: r.bytesWritten,
        Duration:     time.Since(r.startTime),
    }
}

type RecordingStats struct {
    Filename     string
    FrameCount   int
    BytesWritten int64
    Duration     time.Duration
}
```

---

### 5. HTTP API Server

**責務**: REST API エンドポイント提供

```go
package api

import (
    "encoding/json"
    "fmt"
    "io"
    "log"
    "net/http"

    "github.com/smart-pet-camera/streaming-server/recorder"
    "github.com/smart-pet-camera/streaming-server/webrtc"
)

type Server struct {
    webrtcServer *webrtc.Server
    recorder     *recorder.Recorder
}

func NewServer(webrtcServer *webrtc.Server, recorder *recorder.Recorder) *Server {
    return &Server{
        webrtcServer: webrtcServer,
        recorder:     recorder,
    }
}

func (s *Server) Start(addr string) error {
    http.HandleFunc("/api/webrtc/offer", s.handleWebRTCOffer)
    http.HandleFunc("/api/recording/start", s.handleRecordingStart)
    http.HandleFunc("/api/recording/stop", s.handleRecordingStop)
    http.HandleFunc("/api/recording/status", s.handleRecordingStatus)
    http.HandleFunc("/health", s.handleHealth)

    log.Printf("[API] Starting HTTP server on %s", addr)
    return http.ListenAndServe(addr, nil)
}

func (s *Server) handleWebRTCOffer(w http.ResponseWriter, r *http.Request) {
    if r.Method != http.MethodPost {
        http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
        return
    }

    offerJSON, err := io.ReadAll(r.Body)
    if err != nil {
        http.Error(w, "Failed to read body", http.StatusBadRequest)
        return
    }

    answerJSON, err := s.webrtcServer.HandleOffer(offerJSON)
    if err != nil {
        http.Error(w, fmt.Sprintf("Failed to handle offer: %v", err), http.StatusInternalServerError)
        return
    }

    w.Header().Set("Content-Type", "application/json")
    w.Write(answerJSON)
}

func (s *Server) handleRecordingStart(w http.ResponseWriter, r *http.Request) {
    if r.Method != http.MethodPost {
        http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
        return
    }

    var req struct {
        Filename string `json:"filename,omitempty"`
    }

    if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
        // Empty body is OK
        req.Filename = ""
    }

    filename, err := s.recorder.StartRecording(req.Filename)
    if err != nil {
        http.Error(w, fmt.Sprintf("Failed to start recording: %v", err), http.StatusInternalServerError)
        return
    }

    resp := map[string]interface{}{
        "status":   "recording",
        "filename": filename,
    }

    w.Header().Set("Content-Type", "application/json")
    json.NewEncoder(w).Encode(resp)
}

func (s *Server) handleRecordingStop(w http.ResponseWriter, r *http.Request) {
    if r.Method != http.MethodPost {
        http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
        return
    }

    stats, err := s.recorder.StopRecording()
    if err != nil {
        http.Error(w, fmt.Sprintf("Failed to stop recording: %v", err), http.StatusInternalServerError)
        return
    }

    resp := map[string]interface{}{
        "status":        "stopped",
        "filename":      stats.Filename,
        "frame_count":   stats.FrameCount,
        "bytes_written": stats.BytesWritten,
        "duration":      stats.Duration.Seconds(),
    }

    w.Header().Set("Content-Type", "application/json")
    json.NewEncoder(w).Encode(resp)
}

func (s *Server) handleRecordingStatus(w http.ResponseWriter, r *http.Request) {
    if r.Method != http.MethodGet {
        http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
        return
    }

    stats := s.recorder.GetStats()

    resp := map[string]interface{}{
        "recording": s.recorder.IsRecording(),
    }

    if stats != nil {
        resp["filename"] = stats.Filename
        resp["frame_count"] = stats.FrameCount
        resp["bytes_written"] = stats.BytesWritten
        resp["duration"] = stats.Duration.Seconds()
    }

    w.Header().Set("Content-Type", "application/json")
    json.NewEncoder(w).Encode(resp)
}

func (s *Server) handleHealth(w http.ResponseWriter, r *http.Request) {
    resp := map[string]string{
        "status": "ok",
    }

    w.Header().Set("Content-Type", "application/json")
    json.NewEncoder(w).Encode(resp)
}
```

---

### 6. メインプログラム

**ファイル**: `cmd/streaming-server/main.go`

```go
package main

import (
    "context"
    "fmt"
    "log"
    "os"
    "os/signal"
    "syscall"

    "github.com/smart-pet-camera/streaming-server/api"
    "github.com/smart-pet-camera/streaming-server/h264"
    "github.com/smart-pet-camera/streaming-server/recorder"
    "github.com/smart-pet-camera/streaming-server/shm"
    "github.com/smart-pet-camera/streaming-server/webrtc"
)

const (
    shmName       = "/pet_camera_stream"
    recordingsDir = "./recordings"
    httpAddr      = ":8080"
)

func main() {
    log.Println("Starting Smart Pet Camera Streaming Server...")

    // Open shared memory
    shmReader, err := shm.NewSharedMemoryReader(shmName)
    if err != nil {
        log.Fatalf("Failed to open shared memory: %v", err)
    }
    defer shmReader.Close()

    log.Printf("Opened shared memory: %s", shmName)

    // Create H.264 stream processor
    h264Processor := h264.NewStreamProcessor()

    // Create WebRTC server
    webrtcServer, err := webrtc.NewServer(shmReader)
    if err != nil {
        log.Fatalf("Failed to create WebRTC server: %v", err)
    }
    defer webrtcServer.Close()

    log.Println("WebRTC server created")

    // Create recorder
    rec, err := recorder.NewRecorder(recordingsDir, h264Processor)
    if err != nil {
        log.Fatalf("Failed to create recorder: %v", err)
    }

    log.Printf("Recorder initialized (output dir: %s)", recordingsDir)

    // Create HTTP API server
    apiServer := api.NewServer(webrtcServer, rec)

    // Start WebRTC streaming in background
    ctx, cancel := context.WithCancel(context.Background())
    defer cancel()

    go func() {
        if err := webrtcServer.Start(ctx); err != nil {
            log.Printf("WebRTC server error: %v", err)
        }
    }()

    // Start HTTP API server in background
    go func() {
        if err := apiServer.Start(httpAddr); err != nil {
            log.Fatalf("HTTP server error: %v", err)
        }
    }()

    log.Printf("HTTP API server listening on %s", httpAddr)
    log.Println("Server ready!")

    // Wait for interrupt signal
    sigCh := make(chan os.Signal, 1)
    signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)
    <-sigCh

    log.Println("Shutting down gracefully...")
    cancel()
}
```

---

## バックプレッシャー戦略

### 設計方針

**採用戦略**: ノンブロッキング送信 + フレームドロップ

リアルタイムストリーミングシステムでは、**最新のフレームが常に最も価値が高い**。遅いコンポーネントがシステム全体をブロックするよりも、古いフレームをドロップして最新フレームを優先する。

### 実装パターン

#### 1. Processor → WebRTC/Recorder配信

```go
func (s *Server) processLoop() {
    defer s.wg.Done()

    for {
        select {
        case <-s.ctx.Done():
            return
        case frame, ok := <-s.rawFrameChan:
            if !ok {
                return
            }

            processedData, err := s.h264Processor.ProcessFrame(frame.Data)
            if err != nil {
                s.metrics.ProcessErrors.Add(1)
                continue
            }

            processedFrame := &Frame{
                FrameNumber: frame.FrameNumber,
                Data:        processedData,
            }

            // ノンブロッキング送信: WebRTC
            select {
            case s.webrtcChan <- processedFrame:
                s.metrics.WebRTCFramesSent.Add(1)
            default:
                // WebRTCチャネルが溢れた場合はドロップ
                s.metrics.WebRTCFramesDropped.Add(1)
                log.Println("[Processor] Dropped frame for WebRTC (buffer full)")
            }

            // ノンブロッキング送信: Recorder
            select {
            case s.recorderChan <- processedFrame:
                s.metrics.RecorderFramesSent.Add(1)
            default:
                // Recorderが遅い場合はドロップ
                s.metrics.RecorderFramesDropped.Add(1)
                log.Println("[Processor] Dropped frame for Recorder (slow disk I/O)")
            }
        }
    }
}
```

#### 2. WebRTC Distributor → Clients

```go
func (s *Server) distributeWebRTC() {
    defer s.wg.Done()

    for {
        select {
        case <-s.ctx.Done():
            return
        case frame, ok := <-s.webrtcChan:
            if !ok {
                return
            }

            s.clientsMu.RLock()
            clients := make([]*WebRTCClient, 0, len(s.webrtcClients))
            for _, client := range s.webrtcClients {
                clients = append(clients, client)
            }
            s.clientsMu.RUnlock()

            // 各クライアントに並列送信（ノンブロッキング）
            for _, client := range clients {
                go func(c *WebRTCClient) {
                    select {
                    case c.frameChan <- frame:
                        c.metrics.FramesSent.Add(1)
                    default:
                        // 遅いクライアントはフレームスキップ
                        c.metrics.FramesDropped.Add(1)
                    }
                }(client)
            }
        }
    }
}
```

### メリット

| 項目 | 効果 |
|------|------|
| **システム全体の安定性** | 遅いコンポーネントが他に影響しない |
| **レイテンシ** | 常に最新フレームを配信（低遅延） |
| **リソース効率** | バッファ溢れによるメモリ消費を防ぐ |
| **ユーザー体験** | 30fps → 28fpsのドロップは人間には気づかれない |

### モニタリング

ドロップ率を監視し、閾値を超えた場合はアラート:

```go
// メトリクス収集
dropRate := float64(s.metrics.FramesDropped.Load()) / float64(s.metrics.FramesSent.Load())

if dropRate > 0.05 {  // 5%以上のドロップ
    log.Printf("WARNING: High frame drop rate: %.2f%%", dropRate*100)
    // アラート送信、バッファサイズ調整など
}
```

---

## モニタリング・メトリクス

### Prometheusメトリクス実装

#### メトリクス定義

```go
package metrics

import (
    "sync/atomic"
    "time"
)

type Metrics struct {
    // Frame counters
    FramesRead          atomic.Uint64
    FramesProcessed     atomic.Uint64
    FramesSent          atomic.Uint64
    FramesDropped       atomic.Uint64

    // Component-specific
    WebRTCFramesSent    atomic.Uint64
    WebRTCFramesDropped atomic.Uint64
    RecorderFramesSent  atomic.Uint64
    RecorderFramesDropped atomic.Uint64

    // Channel buffer usage (snapshot)
    RawFrameChanLen       atomic.Int32
    ProcessedFrameChanLen atomic.Int32
    WebRTCChanLen         atomic.Int32
    RecorderChanLen       atomic.Int32

    // Client metrics
    ActiveClients atomic.Int32
    TotalClients  atomic.Uint64

    // Error counters
    ReadErrors    atomic.Uint64
    ProcessErrors atomic.Uint64
    SendErrors    atomic.Uint64

    // Latency tracking
    ReadLatencyNs    atomic.Int64
    ProcessLatencyNs atomic.Int64
    SendLatencyNs    atomic.Int64

    // Recording
    RecordingActive  atomic.Bool
    RecordedBytes    atomic.Uint64
    RecordedFrames   atomic.Uint64

    // Uptime
    StartTime time.Time
}

func NewMetrics() *Metrics {
    return &Metrics{
        StartTime: time.Now(),
    }
}

// GetDropRate returns the overall frame drop rate
func (m *Metrics) GetDropRate() float64 {
    sent := m.FramesSent.Load()
    if sent == 0 {
        return 0
    }
    dropped := m.FramesDropped.Load()
    return float64(dropped) / float64(sent+dropped)
}

// GetUptime returns server uptime
func (m *Metrics) GetUptime() time.Duration {
    return time.Since(m.StartTime)
}
```

#### HTTP Prometheusエンドポイント

```go
func (s *Server) metricsHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set("Content-Type", "text/plain")

    fmt.Fprintf(w, "# HELP frames_read_total Total frames read from shared memory\n")
    fmt.Fprintf(w, "# TYPE frames_read_total counter\n")
    fmt.Fprintf(w, "frames_read_total %d\n", s.metrics.FramesRead.Load())

    fmt.Fprintf(w, "# HELP frames_processed_total Total frames processed\n")
    fmt.Fprintf(w, "# TYPE frames_processed_total counter\n")
    fmt.Fprintf(w, "frames_processed_total %d\n", s.metrics.FramesProcessed.Load())

    fmt.Fprintf(w, "# HELP frames_dropped_total Total frames dropped\n")
    fmt.Fprintf(w, "# TYPE frames_dropped_total counter\n")
    fmt.Fprintf(w, "frames_dropped_total{component=\"webrtc\"} %d\n", s.metrics.WebRTCFramesDropped.Load())
    fmt.Fprintf(w, "frames_dropped_total{component=\"recorder\"} %d\n", s.metrics.RecorderFramesDropped.Load())

    fmt.Fprintf(w, "# HELP channel_buffer_usage Current channel buffer usage\n")
    fmt.Fprintf(w, "# TYPE channel_buffer_usage gauge\n")
    fmt.Fprintf(w, "channel_buffer_usage{channel=\"raw\"} %d\n", s.metrics.RawFrameChanLen.Load())
    fmt.Fprintf(w, "channel_buffer_usage{channel=\"processed\"} %d\n", s.metrics.ProcessedFrameChanLen.Load())
    fmt.Fprintf(w, "channel_buffer_usage{channel=\"webrtc\"} %d\n", s.metrics.WebRTCChanLen.Load())
    fmt.Fprintf(w, "channel_buffer_usage{channel=\"recorder\"} %d\n", s.metrics.RecorderChanLen.Load())

    fmt.Fprintf(w, "# HELP active_clients Current number of WebRTC clients\n")
    fmt.Fprintf(w, "# TYPE active_clients gauge\n")
    fmt.Fprintf(w, "active_clients %d\n", s.metrics.ActiveClients.Load())

    fmt.Fprintf(w, "# HELP errors_total Total errors by type\n")
    fmt.Fprintf(w, "# TYPE errors_total counter\n")
    fmt.Fprintf(w, "errors_total{type=\"read\"} %d\n", s.metrics.ReadErrors.Load())
    fmt.Fprintf(w, "errors_total{type=\"process\"} %d\n", s.metrics.ProcessErrors.Load())
    fmt.Fprintf(w, "errors_total{type=\"send\"} %d\n", s.metrics.SendErrors.Load())

    fmt.Fprintf(w, "# HELP latency_microseconds Processing latency in microseconds\n")
    fmt.Fprintf(w, "# TYPE latency_microseconds gauge\n")
    fmt.Fprintf(w, "latency_microseconds{stage=\"read\"} %d\n", s.metrics.ReadLatencyNs.Load()/1000)
    fmt.Fprintf(w, "latency_microseconds{stage=\"process\"} %d\n", s.metrics.ProcessLatencyNs.Load()/1000)
    fmt.Fprintf(w, "latency_microseconds{stage=\"send\"} %d\n", s.metrics.SendLatencyNs.Load()/1000)

    fmt.Fprintf(w, "# HELP recording_active Recording status\n")
    fmt.Fprintf(w, "# TYPE recording_active gauge\n")
    if s.metrics.RecordingActive.Load() {
        fmt.Fprintf(w, "recording_active 1\n")
    } else {
        fmt.Fprintf(w, "recording_active 0\n")
    }

    fmt.Fprintf(w, "# HELP uptime_seconds Server uptime in seconds\n")
    fmt.Fprintf(w, "# TYPE uptime_seconds counter\n")
    fmt.Fprintf(w, "uptime_seconds %d\n", int64(s.metrics.GetUptime().Seconds()))
}
```

#### チャネルバッファ使用率の自動収集

```go
func (s *Server) monitorChannelBuffers(ctx context.Context) {
    ticker := time.NewTicker(1 * time.Second)
    defer ticker.Stop()

    for {
        select {
        case <-ctx.Done():
            return
        case <-ticker.C:
            // チャネルバッファの現在の長さを記録
            s.metrics.RawFrameChanLen.Store(int32(len(s.rawFrameChan)))
            s.metrics.ProcessedFrameChanLen.Store(int32(len(s.processedFrameChan)))
            s.metrics.WebRTCChanLen.Store(int32(len(s.webrtcChan)))
            s.metrics.RecorderChanLen.Store(int32(len(s.recorderChan)))
        }
    }
}

// Serverの起動時に追加
func (s *Server) Start() {
    // ... 他のgoroutine起動

    // チャネルバッファ監視goroutineを起動
    s.wg.Add(1)
    go func() {
        defer s.wg.Done()
        s.monitorChannelBuffers(s.ctx)
    }()
}
```

### Grafanaダッシュボード設定例

```yaml
# prometheus.yml
scrape_configs:
  - job_name: 'streaming-server'
    static_configs:
      - targets: ['localhost:8080']
    metrics_path: '/metrics'
    scrape_interval: 5s
```

**主要ダッシュボードパネル**:
1. Frame Throughput (frames/sec)
2. Drop Rate (%)
3. Channel Buffer Usage (%)
4. Active Clients
5. Latency (ms)
6. Error Rate

---

## プロファイリング

### pprof常時有効化

**設計方針**: 本番環境でも常時pprofを有効化し、パフォーマンス問題を即座に診断可能にする。

#### 実装

```go
package main

import (
    "log"
    "net/http"
    _ "net/http/pprof"  // pprof HTTPハンドラー自動登録
)

func (s *Server) enableProfiling(addr string) {
    // pprofエンドポイントを別ポートで起動（セキュリティのため）
    go func() {
        log.Printf("Starting pprof server on %s", addr)
        if err := http.ListenAndServe(addr, nil); err != nil {
            log.Printf("pprof server error: %v", err)
        }
    }()
}

func main() {
    server := NewServer("/pet_camera_stream")

    // pprofを localhost:6060 で起動
    server.enableProfiling("localhost:6060")

    // メインサーバーを起動
    server.Start()
}
```

#### 利用可能なプロファイル

| エンドポイント | 用途 |
|--------------|------|
| `/debug/pprof/profile` | CPUプロファイル（30秒間） |
| `/debug/pprof/heap` | メモリ割り当てプロファイル |
| `/debug/pprof/goroutine` | Goroutine情報 |
| `/debug/pprof/block` | ブロッキングプロファイル |
| `/debug/pprof/mutex` | Mutex競合プロファイル |
| `/debug/pprof/allocs` | メモリ割り当て統計 |

#### 使用例

```bash
# CPUプロファイル取得（30秒間）
go tool pprof http://localhost:6060/debug/pprof/profile

# メモリヒープダンプ
go tool pprof http://localhost:6060/debug/pprof/heap

# Goroutineリーク調査
curl http://localhost:6060/debug/pprof/goroutine?debug=2

# インタラクティブ分析（Web UI）
go tool pprof -http=:8081 http://localhost:6060/debug/pprof/profile
```

#### ブロッキング・Mutexプロファイリング有効化

```go
func init() {
    // ブロッキングプロファイルを有効化
    runtime.SetBlockProfileRate(1)

    // Mutexプロファイルを有効化
    runtime.SetMutexProfileFraction(1)
}
```

### パフォーマンスベンチマーク

```go
// benchmark_test.go
package main

import (
    "testing"
    "time"
)

func BenchmarkFrameProcessing(b *testing.B) {
    processor := NewH264Processor()
    frame := generateTestFrame()

    b.ResetTimer()
    for i := 0; i < b.N; i++ {
        processor.ProcessFrame(frame)
    }
}

func BenchmarkWebRTCDistribution(b *testing.B) {
    server := setupTestServer()
    frame := generateTestFrame()

    // 5クライアントを追加
    for i := 0; i < 5; i++ {
        server.AddWebRTCClient(fmt.Sprintf("client-%d", i), nil)
    }

    b.ResetTimer()
    for i := 0; i < b.N; i++ {
        server.webrtcChan <- frame
    }
}

func BenchmarkChannelThroughput(b *testing.B) {
    ch := make(chan *Frame, 10)
    go func() {
        for range ch {
            // 消費
        }
    }()

    b.ResetTimer()
    for i := 0; i < b.N; i++ {
        ch <- generateTestFrame()
    }
}
```

---

## HTTP API仕様

### 1. WebRTCシグナリング

#### POST /api/webrtc/offer

WebRTCのSDP offerを受け取り、answerを返す。

**Request**:
```json
{
    "type": "offer",
    "sdp": "v=0\r\no=- ... (SDP content)"
}
```

**Response**:
```json
{
    "type": "answer",
    "sdp": "v=0\r\no=- ... (SDP content)"
}
```

**Status Codes**:
- `200 OK`: 成功
- `400 Bad Request`: 不正なSDP
- `500 Internal Server Error`: サーバーエラー

---

### 2. 録画制御

#### POST /api/recording/start

録画を開始する。

**Request**:
```json
{
    "filename": "optional_custom_name.h264"
}
```

**Response**:
```json
{
    "status": "recording",
    "filename": "recordings/recording_20251226_143052.h264"
}
```

**Status Codes**:
- `200 OK`: 録画開始成功
- `500 Internal Server Error`: すでに録画中、またはファイル作成失敗

---

#### POST /api/recording/stop

録画を停止する。

**Request**: なし (空のPOST)

**Response**:
```json
{
    "status": "stopped",
    "filename": "recordings/recording_20251226_143052.h264",
    "frame_count": 1234,
    "bytes_written": 5678901,
    "duration": 41.1
}
```

**Status Codes**:
- `200 OK`: 録画停止成功
- `500 Internal Server Error`: 録画中でない、またはファイルクローズ失敗

---

#### GET /api/recording/status

現在の録画状態を取得する。

**Request**: なし

**Response (録画中)**:
```json
{
    "recording": true,
    "filename": "recordings/recording_20251226_143052.h264",
    "frame_count": 567,
    "bytes_written": 1234567,
    "duration": 18.9
}
```

**Response (非録画中)**:
```json
{
    "recording": false
}
```

**Status Codes**:
- `200 OK`: 常に成功

---

#### POST /api/recording/heartbeat

録画中のハートビートを送信する。3秒以内にハートビートがない場合、録画は自動停止される。

**Request**: なし (空のPOST)

**Response (成功)**:
```json
{
    "ok": true
}
```

**Response (録画中でない)**:
```json
{
    "ok": false
}
```

**Status Codes**:
- `200 OK`: 常に成功

---

### 3. 録画一覧管理

#### GET /api/recordings

保存された録画ファイルの一覧を取得する。

**Request**: なし

**Response**:
```json
{
    "recordings": [
        {
            "name": "recording_20260204_143052.mp4",
            "size_bytes": 47185920,
            "created_at": "2026-02-04T14:30:52+09:00"
        },
        {
            "name": "recording_20260204_150823.mp4",
            "size_bytes": 128450560,
            "created_at": "2026-02-04T15:08:23+09:00"
        }
    ]
}
```

**Status Codes**:
- `200 OK`: 常に成功

---

#### GET /api/recordings/{filename}

録画ファイルをダウンロードする。

**Request**: なし

**Response**: バイナリファイル (Content-Type: video/mp4 または application/octet-stream)

**Status Codes**:
- `200 OK`: ダウンロード成功
- `404 Not Found`: ファイルが存在しない
- `400 Bad Request`: 不正なファイル名

---

#### DELETE /api/recordings/{filename}

録画ファイルを削除する。

**Request**: なし

**Response**:
```json
{
    "deleted": "recording_20260204_143052.mp4"
}
```

**Status Codes**:
- `200 OK`: 削除成功
- `404 Not Found`: ファイルが存在しない
- `400 Bad Request`: 不正なファイル名

---

### 4. ヘルスチェック

#### GET /health

サーバーの稼働状態を確認する。

**Response**:
```json
{
    "status": "ok"
}
```

**Status Codes**:
- `200 OK`: サーバー稼働中

---

### 4. 接続数取得

#### GET /api/connections

現在のストリーム接続数を取得する。

**Request**: なし

**Response**:
```json
{
    "webrtc": 2,
    "mjpeg": 1,
    "detection_sse": 1,
    "status_sse": 2,
    "total": 6,
    "timestamp": 1738665600
}
```

| フィールド | 説明 |
|-----------|------|
| `webrtc` | WebRTC接続数 |
| `mjpeg` | MJPEGストリーム接続数 |
| `detection_sse` | 検出イベントSSE接続数 |
| `status_sse` | ステータスSSE接続数 |
| `total` | 合計接続数 |
| `timestamp` | Unix timestamp |

**Status Codes**:
- `200 OK`: 常に成功

---

#### GET /api/connections/stream

接続数の変化をSSE (Server-Sent Events) でリアルタイム配信する。
接続数が変化した時のみイベントが送信される（イベント駆動方式）。

**Request**: なし

**Response**: `text/event-stream`

```
event: connections
data: {"webrtc":2,"mjpeg":1,"detection_sse":1,"status_sse":2,"total":6,"timestamp":1738665600}

event: connections
data: {"webrtc":3,"mjpeg":1,"detection_sse":1,"status_sse":2,"total":7,"timestamp":1738665605}
```

**イベント形式**:
- `event`: イベント名 (`connections`)
- `data`: JSON形式の接続数データ

**特徴**:
- ポーリングではなくイベント駆動（Subscribe/Unsubscribe時に通知）
- WebRTC接続数のみ2秒間隔でポーリング（別サーバーのため）
- 30秒間隔でkeepaliveコメントを送信

**Status Codes**:
- `200 OK`: SSEストリーム開始

---

### 5. WebRTCクライアント数 (WebRTCサーバー)

#### GET /api/clients/count

WebRTCサーバーの現在の接続クライアント数を取得する。
（WebRTCサーバー :8081 で提供）

**Request**: なし

**Response**:
```json
{
    "count": 3
}
```

**Status Codes**:
- `200 OK`: 常に成功

---

## パフォーマンス目標

### リソース使用量

| メトリクス | 目標値 | 測定方法 |
|-----------|-------|---------|
| メモリ使用量 | <20MB | `top -p $(pgrep streaming-server)` |
| CPU使用率 | <10% (idle時 <3%) | `top -p $(pgrep streaming-server)` |
| 起動時間 | <500ms | `time ./streaming-server` |
| バイナリサイズ | <15MB | `ls -lh streaming-server` |

### ストリーミング性能

| メトリクス | 目標値 | 測定方法 |
|-----------|-------|---------|
| WebRTC遅延 | <100ms | ブラウザDevTools Network タブ |
| フレームレート | 30fps (安定) | ブラウザ `video.requestVideoFrameCallback()` |
| 同時接続クライアント | 10+ | 負荷テスト |
| パケットロス耐性 | <5%で再生可能 | ネットワークシミュレーション |

### 録画性能

| メトリクス | 目標値 | 測定方法 |
|-----------|-------|---------|
| ファイル書き込み遅延 | <10ms/frame | ベンチマーク |
| ディスク使用量 | ~30MB/分 @ 8Mbps | 録画ファイルサイズ確認 |
| SPS/PPS付与率 | 100% (すべてのIDRに付与) | ffprobe検証 |

---

## 実装計画

### Phase 1: 基盤実装 (2-3日)

#### Day 1: プロジェクト構造 + 共有メモリアクセス

- [ ] Goプロジェクト初期化 (`go mod init`)
- [ ] ディレクトリ構造作成
  ```
  streaming-server/
  ├── cmd/streaming-server/main.go
  ├── shm/                     # 共有メモリアクセス (cgo)
  ├── h264/                    # NAL unit処理
  ├── webrtc/                  # WebRTC server
  ├── recorder/                # 録画
  ├── api/                     # HTTP API
  └── go.mod
  ```
- [ ] cgo共有メモリアクセス実装
- [ ] 単体テスト (mock shared memory)

**成果物**: 共有メモリからH.264フレーム読み取り可能

---

#### Day 2: H.264処理 + WebRTC基礎

- [ ] NAL unit parser実装
- [ ] SPS/PPS detector & cache実装
- [ ] pion/webrtc統合
- [ ] H.264トラック実装 (passthrough)

**成果物**: WebRTCでH.264配信可能 (SPS/PPS対応)

---

#### Day 3: 録画 + HTTP API

- [ ] H264Recorder実装
- [ ] HTTP APIエンドポイント実装
- [ ] 統合テスト

**成果物**: 録画API動作、WebRTC配信と並行動作

---

### Phase 2: テスト・最適化 (1-2日)

#### Day 4: 統合テスト

- [ ] 実機でのエンドツーエンドテスト
- [ ] 複数クライアント接続テスト
- [ ] 長時間動作テスト (24時間)
- [ ] メモリリークチェック

---

#### Day 5: パフォーマンスチューニング

- [ ] プロファイリング (`pprof`)
- [ ] ボトルネック特定・最適化
- [ ] ベンチマーク計測

**目標**:
- メモリ: <20MB
- CPU: <10%
- 遅延: <100ms

---

### Phase 3: デプロイ・ドキュメント (1日)

#### Day 6: デプロイ準備

- [ ] systemdサービスファイル作成
- [ ] ビルドスクリプト作成
- [ ] クロスコンパイル設定 (ARM64)
- [ ] ユーザーマニュアル作成

**成果物**: 本番環境へのデプロイ可能

---

## 移行戦略

### フェーズド移行

#### Phase A: 並行稼働 (1週間)

```
┌─────────────────┐       ┌─────────────────┐
│ Python版        │       │ Go版            │
│ (port 8080)     │       │ (port 8081)     │
└─────────────────┘       └─────────────────┘
        │                         │
        └─────────┬───────────────┘
                  │
          共有メモリ読み取り
```

- Python版を8080で継続稼働
- Go版を8081で並行稼働
- 性能比較・検証

**判定基準**:
- ✅ Go版のメモリ使用量 < Python版の20%
- ✅ Go版のCPU使用率 < Python版の50%
- ✅ WebRTC接続成功率 > 95%
- ✅ 録画ファイルがVLC再生可能

---

#### Phase B: 部分切り替え (1週間)

- WebRTCのみGo版に切り替え
- 録画はPython版を継続
- 問題発生時はPython版にロールバック可能

---

#### Phase C: 完全移行 (1日)

- すべての機能をGo版に移行
- Python版を停止
- systemdサービス更新

---

### ロールバック計画

問題が発生した場合:

```bash
# Go版を停止
sudo systemctl stop streaming-server

# Python版を再起動
sudo systemctl start smart-pet-camera-webrtc
sudo systemctl start smart-pet-camera-recorder
```

---

## 依存関係とビルド

### go.mod

```go
module github.com/smart-pet-camera/streaming-server

go 1.21

require (
    github.com/pion/webrtc/v3 v3.2.24
    github.com/pion/rtp v1.8.3
    github.com/pion/interceptor v0.1.25
)

require (
    github.com/google/uuid v1.5.0 // indirect
    github.com/pion/datachannel v1.5.5 // indirect
    github.com/pion/dtls/v2 v2.2.8 // indirect
    github.com/pion/ice/v2 v2.3.13 // indirect
    github.com/pion/logging v0.2.2 // indirect
    github.com/pion/mdns v0.0.9 // indirect
    github.com/pion/randutil v0.1.0 // indirect
    github.com/pion/rtcp v1.2.13 // indirect
    github.com/pion/sctp v1.8.12 // indirect
    github.com/pion/sdp/v3 v3.0.6 // indirect
    github.com/pion/srtp/v2 v2.0.18 // indirect
    github.com/pion/stun v0.6.1 // indirect
    github.com/pion/transport/v2 v2.2.4 // indirect
    github.com/pion/turn/v2 v2.1.5 // indirect
    golang.org/x/crypto v0.17.0 // indirect
    golang.org/x/net v0.19.0 // indirect
    golang.org/x/sys v0.15.0 // indirect
)
```

### Makefile

```makefile
# Makefile for streaming-server

BINARY_NAME=streaming-server
BUILD_DIR=../../build
CGO_ENABLED=1

# Compiler flags
CFLAGS=-I../../capture
LDFLAGS=-L$(BUILD_DIR) -lrt

.PHONY: all build clean test run

all: build

build:
	@echo "Building $(BINARY_NAME)..."
	CGO_ENABLED=$(CGO_ENABLED) \
	CGO_CFLAGS="$(CFLAGS)" \
	CGO_LDFLAGS="$(LDFLAGS)" \
	go build -o $(BUILD_DIR)/$(BINARY_NAME) ./cmd/streaming-server
	@echo "Build complete: $(BUILD_DIR)/$(BINARY_NAME)"

clean:
	@echo "Cleaning..."
	rm -f $(BUILD_DIR)/$(BINARY_NAME)

test:
	@echo "Running tests..."
	go test -v ./...

run: build
	@echo "Running $(BINARY_NAME)..."
	$(BUILD_DIR)/$(BINARY_NAME)

# Cross-compile for ARM64
build-arm64:
	@echo "Cross-compiling for ARM64..."
	CGO_ENABLED=$(CGO_ENABLED) \
	GOOS=linux \
	GOARCH=arm64 \
	CGO_CFLAGS="$(CFLAGS)" \
	CGO_LDFLAGS="$(LDFLAGS)" \
	go build -o $(BUILD_DIR)/$(BINARY_NAME)-arm64 ./cmd/streaming-server
```

---

## テスト計画

### 単体テスト

```go
// shm/reader_test.go
func TestSharedMemoryReader(t *testing.T) {
    // Mock shared memory test
}

// h264/processor_test.go
func TestNALUnitParser(t *testing.T) {
    // NAL unit parsing test
}

func TestSPSPPSDetection(t *testing.T) {
    // SPS/PPS detection test
}
```

### 統合テスト

```go
// integration_test.go
func TestWebRTCStreaming(t *testing.T) {
    // End-to-end WebRTC test
}

func TestRecording(t *testing.T) {
    // Recording test
}
```

### ベンチマーク

```go
// benchmark_test.go
func BenchmarkFrameProcessing(b *testing.B) {
    // Frame processing benchmark
}

func BenchmarkWebRTCWriteSample(b *testing.B) {
    // WebRTC write sample benchmark
}
```

---

## systemdサービス

**ファイル**: `/etc/systemd/system/streaming-server.service`

```ini
[Unit]
Description=Smart Pet Camera Streaming Server
After=network.target

[Service]
Type=simple
User=sunrise
WorkingDirectory=/app/smart-pet-camera
ExecStart=/app/smart-pet-camera/build/streaming-server
Restart=always
RestartSec=5

# Resource limits
MemoryLimit=50M
CPUQuota=30%

# Logging
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

---

## まとめ

### 主要な技術的決定

1. **pion/webrtc**: 成熟したGo WebRTC実装、H.264 passthrough対応
2. **cgo**: 既存のC共有メモリコードを再利用
3. **シングルバイナリ**: デプロイ・保守が容易
4. **SPS/PPS自動付与**: NAL unit解析で録画・WebRTC両方に対応

### 期待される成果

- **効率性**: メモリ82%削減、CPU60%削減
- **信頼性**: SPS/PPS問題の完全解決
- **保守性**: 単一言語、単一コードベース
- **拡張性**: Goroutineで容易にスケール

### 次のステップ

1. ✅ 設計ドキュメント作成 (本ドキュメント)
2. ⏳ PoC実装開始 (Day 1-3)
3. ⏳ 性能比較テスト (Phase A)
4. ⏳ 本番移行 (Phase B-C)

---

**Last Updated**: 2025-12-26
**Author**: Claude Sonnet 4.5
**Status**: 設計完了、実装準備完了
