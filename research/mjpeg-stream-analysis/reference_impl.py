#!/usr/bin/env python3
"""
MJPEG Parser Reference Implementation
マイコン実装の参考用Pythonコード

Usage:
  python reference_impl.py [--frames N]

Features:
  - ゼロコピー状態マシン (案B)
  - 最適化されたバッファサイズ
  - トリプルバッファ パイプライン
"""

import socket
import re
import time
import threading
import queue
import argparse
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional
import io

# PIL is optional (for decode verification)
try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False


# =============================================================================
# Configuration (from buffer analysis)
# =============================================================================

MJPEG_URL = "http://192.168.1.33:8082/stream"

# 最適化されたバッファサイズ (実測値ベース)
LINEAR_BUF_SIZE = 83558   # 82KB (max 66KB + 20% margin)
HEADER_BUF_SIZE = 64      # 60B actual + padding
LINEAR_BUF_COUNT = 3      # トリプルバッファ推奨


# =============================================================================
# State Machine
# =============================================================================

class State(Enum):
    """パーサー状態"""
    CHUNK_SIZE = auto()      # "xxxx\r\n" チャンクサイズ行
    MJPEG_HEADER = auto()    # "--frame\r\n...Content-Length:N\r\n\r\n"
    JPEG_BODY = auto()       # JPEGバイナリデータ
    CHUNK_TRAILER = auto()   # "\r\n" チャンク終端


@dataclass
class StreamParser:
    """
    ゼロコピー MJPEG パーサー

    案B: write_ptr位置に直接読み込み + 上書き読み捨て

    C++実装時の対応:
      header_buf → char header_buf[HEADER_BUF_SIZE] (スタック)
      linear_buf → 外部から渡されるバッファポインタ
    """
    state: State = State.CHUNK_SIZE
    chunk_remaining: int = 0    # 現チャンクの残りバイト
    jpeg_remaining: int = 0     # JPEGの残りバイト
    content_length: int = 0     # パース済みContent-Length

    header_buf: bytearray = field(default_factory=lambda: bytearray(HEADER_BUF_SIZE))
    header_idx: int = 0

    def reset(self):
        """新フレーム開始時にリセット"""
        self.state = State.CHUNK_SIZE
        self.chunk_remaining = 0
        self.content_length = 0
        self.jpeg_remaining = 0
        self.header_idx = 0


# =============================================================================
# Buffer Management
# =============================================================================

class BufferState(Enum):
    FREE = auto()
    FETCHING = auto()
    READY = auto()


@dataclass
class FrameBuffer:
    """フレームバッファ"""
    id: int
    data: bytearray = field(default_factory=lambda: bytearray(LINEAR_BUF_SIZE))
    size: int = 0
    state: BufferState = BufferState.FREE


class BufferPool:
    """
    バッファプール

    C++実装時: FreeRTOS Queue で管理
      linear_free_queue_ = xQueueCreate(LINEAR_BUF_COUNT, sizeof(uint8_t*))
    """
    def __init__(self, count: int = LINEAR_BUF_COUNT):
        self.buffers = [FrameBuffer(id=i) for i in range(count)]
        self.lock = threading.Lock()

    def acquire(self) -> Optional[FrameBuffer]:
        with self.lock:
            for buf in self.buffers:
                if buf.state == BufferState.FREE:
                    buf.state = BufferState.FETCHING
                    buf.size = 0
                    return buf
        return None

    def release(self, buf: FrameBuffer):
        with self.lock:
            buf.state = BufferState.FREE


# =============================================================================
# Fetch Task
# =============================================================================

class FetchTask:
    """
    Fetchタスク

    C++実装時: void fetchTask(void* pvParameters)
    """
    def __init__(self, pool: BufferPool, out_queue: queue.Queue):
        self.pool = pool
        self.out_queue = out_queue
        self.parser = StreamParser()
        self.running = True

    def fetch_frame(self, sock: socket.socket, buf: FrameBuffer) -> bool:
        """
        1フレームをバッファに読み込み

        Returns: 成功時True
        """
        # 最初のフレームのみリセット、以降は前フレームの状態を継続
        if self.parser.content_length == 0:
            self.parser.state = State.CHUNK_SIZE
            self.parser.header_idx = 0

        self.parser.content_length = 0
        self.parser.jpeg_remaining = 0
        write_ptr = 0

        while self.running:
            if self.parser.state == State.CHUNK_SIZE:
                # チャンクサイズ行をheader_bufに読む
                while self.parser.header_idx < HEADER_BUF_SIZE:
                    b = sock.recv(1)
                    if not b:
                        return False

                    self.parser.header_buf[self.parser.header_idx] = b[0]
                    self.parser.header_idx += 1

                    # \r\n検出
                    if self.parser.header_idx >= 2:
                        if (self.parser.header_buf[self.parser.header_idx - 2] == 0x0d and
                            self.parser.header_buf[self.parser.header_idx - 1] == 0x0a):
                            line = bytes(self.parser.header_buf[:self.parser.header_idx - 2]).strip()
                            if line:
                                try:
                                    self.parser.chunk_remaining = int(line, 16)
                                    self.parser.header_idx = 0  # 上書き読み捨て
                                    self.parser.state = (State.JPEG_BODY if self.parser.jpeg_remaining > 0
                                                        else State.MJPEG_HEADER)
                                    break
                                except ValueError:
                                    pass
                            self.parser.header_idx = 0

            elif self.parser.state == State.MJPEG_HEADER:
                # MJPEGヘッダーをheader_bufに読む
                while self.parser.chunk_remaining > 0 and self.parser.header_idx < HEADER_BUF_SIZE:
                    b = sock.recv(1)
                    if not b:
                        return False

                    self.parser.header_buf[self.parser.header_idx] = b[0]
                    self.parser.header_idx += 1
                    self.parser.chunk_remaining -= 1

                    # \r\n\r\n検出 (ヘッダー終了)
                    if self.parser.header_idx >= 4:
                        if bytes(self.parser.header_buf[self.parser.header_idx-4:self.parser.header_idx]) == b'\r\n\r\n':
                            header = bytes(self.parser.header_buf[:self.parser.header_idx])
                            match = re.search(rb'Content-Length:\s*(\d+)', header)
                            if match:
                                self.parser.content_length = int(match.group(1))
                                self.parser.jpeg_remaining = self.parser.content_length
                            self.parser.header_idx = 0  # 上書き読み捨て
                            self.parser.state = State.JPEG_BODY
                            break

                if self.parser.chunk_remaining == 0 and self.parser.state == State.MJPEG_HEADER:
                    self.parser.state = State.CHUNK_TRAILER

            elif self.parser.state == State.JPEG_BODY:
                # JPEGデータをlinear_bufに直接読み込み（ゼロコピー）
                to_read = min(self.parser.chunk_remaining, self.parser.jpeg_remaining)
                to_read = min(to_read, LINEAR_BUF_SIZE - write_ptr, 4096)

                if to_read > 0:
                    data = sock.recv(to_read)
                    if not data:
                        return False

                    # 直接書き込み
                    buf.data[write_ptr:write_ptr + len(data)] = data
                    write_ptr += len(data)
                    self.parser.chunk_remaining -= len(data)
                    self.parser.jpeg_remaining -= len(data)

                if self.parser.jpeg_remaining == 0:
                    buf.size = write_ptr
                    # 次のフレームへの状態遷移
                    if self.parser.chunk_remaining > 0:
                        self.parser.state = State.MJPEG_HEADER
                        self.parser.header_idx = 0
                    else:
                        self.parser.state = State.CHUNK_TRAILER
                    return True  # フレーム完了

                if self.parser.chunk_remaining == 0:
                    self.parser.state = State.CHUNK_TRAILER

            elif self.parser.state == State.CHUNK_TRAILER:
                sock.recv(2)  # \r\n を読み飛ばす
                self.parser.state = State.CHUNK_SIZE
                self.parser.header_idx = 0

        return False


# =============================================================================
# Main
# =============================================================================

def parse_url(url: str) -> tuple[str, int, str]:
    match = re.match(r'http://([^:/]+):?(\d+)?(/.*)?', url)
    if not match:
        raise ValueError(f"Invalid URL: {url}")
    return match.group(1), int(match.group(2) or 80), match.group(3) or '/'


def main():
    parser = argparse.ArgumentParser(description='MJPEG Parser Reference Implementation')
    parser.add_argument('--frames', type=int, default=10, help='Number of frames to capture')
    parser.add_argument('--url', type=str, default=MJPEG_URL, help='MJPEG stream URL')
    args = parser.parse_args()

    host, port, path = parse_url(args.url)
    print(f"MJPEG Parser Reference Implementation")
    print(f"=====================================")
    print(f"URL: {args.url}")
    print(f"Buffer: {LINEAR_BUF_SIZE/1024:.0f}KB x {LINEAR_BUF_COUNT}")
    print(f"Frames: {args.frames}\n")

    # Setup
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(30)
    sock.connect((host, port))

    request = f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nConnection: keep-alive\r\n\r\n"
    sock.sendall(request.encode())

    # Skip HTTP headers
    buf = b''
    while b'\r\n\r\n' not in buf:
        buf += sock.recv(1)

    pool = BufferPool()
    out_queue = queue.Queue()
    fetch = FetchTask(pool, out_queue)

    # Capture frames
    print(f"{'#':<4} {'Size':>8} {'SOI':<4} {'EOI':<4} {'Decode':<7}")
    print("-" * 40)

    results = []
    start = time.perf_counter()

    for i in range(args.frames):
        buf = pool.acquire()
        while buf is None:
            time.sleep(0.001)
            buf = pool.acquire()

        if fetch.fetch_frame(sock, buf):
            # Verify
            soi = buf.data[0:2] == b'\xff\xd8'
            eoi = buf.data[buf.size-2:buf.size] == b'\xff\xd9'
            decode = False

            if HAS_PIL:
                try:
                    img = Image.open(io.BytesIO(bytes(buf.data[:buf.size])))
                    img.load()
                    decode = True
                except:
                    pass

            print(f"{i:<4} {buf.size:>8} {'✓' if soi else '✗':<4} {'✓' if eoi else '✗':<4} {'✓' if decode else '-':<7}")
            results.append({'size': buf.size, 'soi': soi, 'eoi': eoi, 'decode': decode})

        pool.release(buf)

    elapsed = time.perf_counter() - start
    sock.close()

    # Summary
    print("-" * 40)
    print(f"Frames:  {len(results)}")
    print(f"Time:    {elapsed:.2f}s")
    print(f"FPS:     {len(results)/elapsed:.1f}")
    print(f"SOI OK:  {sum(r['soi'] for r in results)}/{len(results)}")
    print(f"EOI OK:  {sum(r['eoi'] for r in results)}/{len(results)}")
    if HAS_PIL:
        print(f"Decode:  {sum(r['decode'] for r in results)}/{len(results)}")


if __name__ == "__main__":
    main()
