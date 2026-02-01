#!/usr/bin/env python3
"""
MJPEG Stream Statistics Analyzer
カメラストリームのFPS・フレームサイズ・遅延などを計測

Usage:
  python stream_stats.py [--duration SECONDS] [--url URL]
"""

import socket
import re
import time
import argparse
import statistics
from dataclasses import dataclass, field
from typing import List

MJPEG_URL = "http://192.168.1.33:8082/stream"
BUFFER_SIZE = 100 * 1024  # 100KB


@dataclass
class FrameStats:
    timestamp: float
    size: int
    recv_duration: float  # フレーム受信にかかった時間


@dataclass
class StreamAnalyzer:
    frames: List[FrameStats] = field(default_factory=list)
    start_time: float = 0

    def add_frame(self, size: int, recv_duration: float):
        self.frames.append(FrameStats(
            timestamp=time.perf_counter(),
            size=size,
            recv_duration=recv_duration
        ))

    def print_realtime(self, interval: int = 10):
        """直近N フレームの統計を表示"""
        if len(self.frames) < 2:
            return

        recent = self.frames[-interval:] if len(self.frames) >= interval else self.frames

        # FPS計算
        if len(recent) >= 2:
            duration = recent[-1].timestamp - recent[0].timestamp
            fps = (len(recent) - 1) / duration if duration > 0 else 0
        else:
            fps = 0

        # フレームサイズ
        sizes = [f.size for f in recent]
        avg_size = statistics.mean(sizes)

        # 帯域幅 (Mbps)
        total_bytes = sum(sizes)
        duration = recent[-1].timestamp - recent[0].timestamp if len(recent) >= 2 else 1
        bandwidth_mbps = (total_bytes * 8) / (duration * 1_000_000) if duration > 0 else 0

        print(f"\r[{len(self.frames):4d}] FPS: {fps:5.1f} | Size: {avg_size/1024:5.1f}KB | BW: {bandwidth_mbps:5.2f}Mbps", end="", flush=True)

    def print_summary(self):
        """最終統計サマリー"""
        if len(self.frames) < 2:
            print("Not enough frames for statistics")
            return

        total_duration = self.frames[-1].timestamp - self.frames[0].timestamp

        # FPS
        avg_fps = (len(self.frames) - 1) / total_duration

        # フレーム間隔
        intervals = []
        for i in range(1, len(self.frames)):
            intervals.append(self.frames[i].timestamp - self.frames[i-1].timestamp)

        # フレームサイズ
        sizes = [f.size for f in self.frames]

        # 受信時間
        recv_times = [f.recv_duration * 1000 for f in self.frames]  # ms

        # 帯域幅
        total_bytes = sum(sizes)
        bandwidth_mbps = (total_bytes * 8) / (total_duration * 1_000_000)

        print("\n")
        print("=" * 60)
        print("MJPEG Stream Statistics Summary")
        print("=" * 60)

        print(f"\n{'Duration:':<20} {total_duration:.2f} seconds")
        print(f"{'Total Frames:':<20} {len(self.frames)}")
        print(f"{'Total Data:':<20} {total_bytes / 1024 / 1024:.2f} MB")

        print(f"\n--- Frame Rate ---")
        print(f"{'Average FPS:':<20} {avg_fps:.2f}")
        print(f"{'Frame Interval:':<20}")
        print(f"  {'Min:':<16} {min(intervals)*1000:.2f} ms")
        print(f"  {'Max:':<16} {max(intervals)*1000:.2f} ms")
        print(f"  {'Avg:':<16} {statistics.mean(intervals)*1000:.2f} ms")
        print(f"  {'StdDev:':<16} {statistics.stdev(intervals)*1000:.2f} ms")

        print(f"\n--- Frame Size ---")
        print(f"  {'Min:':<16} {min(sizes)/1024:.2f} KB")
        print(f"  {'Max:':<16} {max(sizes)/1024:.2f} KB")
        print(f"  {'Avg:':<16} {statistics.mean(sizes)/1024:.2f} KB")
        print(f"  {'StdDev:':<16} {statistics.stdev(sizes)/1024:.2f} KB")

        print(f"\n--- Receive Time (per frame) ---")
        print(f"  {'Min:':<16} {min(recv_times):.2f} ms")
        print(f"  {'Max:':<16} {max(recv_times):.2f} ms")
        print(f"  {'Avg:':<16} {statistics.mean(recv_times):.2f} ms")

        print(f"\n--- Bandwidth ---")
        print(f"{'Average:':<20} {bandwidth_mbps:.2f} Mbps")

        # ジッター (フレーム間隔の標準偏差)
        jitter = statistics.stdev(intervals) * 1000
        print(f"\n--- Jitter ---")
        print(f"{'Frame Interval σ:':<20} {jitter:.2f} ms")

        print("=" * 60)


def parse_url(url: str) -> tuple:
    match = re.match(r'http://([^:/]+):?(\d+)?(/.*)?', url)
    if not match:
        raise ValueError(f"Invalid URL: {url}")
    return match.group(1), int(match.group(2) or 80), match.group(3) or '/'


def fetch_frames(sock: socket.socket, analyzer: StreamAnalyzer, duration: float):
    """MJPEGストリームからフレームを取得し統計収集"""
    buffer = bytearray(BUFFER_SIZE)
    write_pos = 0
    content_length = 0
    in_jpeg = False
    frame_start_time = 0

    start_time = time.perf_counter()

    while time.perf_counter() - start_time < duration:
        try:
            data = sock.recv(4096)
            if not data:
                break

            # バッファに追加
            buffer[write_pos:write_pos + len(data)] = data
            write_pos += len(data)

            # 解析
            while True:
                buf_view = bytes(buffer[:write_pos])

                if not in_jpeg:
                    # Content-Length を探す
                    match = re.search(rb'Content-Length:\s*(\d+)\r\n\r\n', buf_view)
                    if match:
                        content_length = int(match.group(1))
                        header_end = match.end()

                        # ヘッダー部分を削除
                        remaining = write_pos - header_end
                        buffer[:remaining] = buffer[header_end:write_pos]
                        write_pos = remaining

                        in_jpeg = True
                        frame_start_time = time.perf_counter()
                    else:
                        break

                if in_jpeg:
                    if write_pos >= content_length:
                        # フレーム完了
                        recv_duration = time.perf_counter() - frame_start_time
                        analyzer.add_frame(content_length, recv_duration)
                        analyzer.print_realtime()

                        # 次のフレームへ
                        remaining = write_pos - content_length
                        buffer[:remaining] = buffer[content_length:write_pos]
                        write_pos = remaining

                        in_jpeg = False
                        content_length = 0
                    else:
                        break

        except socket.timeout:
            continue
        except Exception as e:
            print(f"\nError: {e}")
            break


def main():
    parser = argparse.ArgumentParser(description='MJPEG Stream Statistics Analyzer')
    parser.add_argument('--duration', type=float, default=10.0, help='Measurement duration in seconds')
    parser.add_argument('--url', type=str, default=MJPEG_URL, help='MJPEG stream URL')
    args = parser.parse_args()

    host, port, path = parse_url(args.url)

    print(f"MJPEG Stream Statistics Analyzer")
    print(f"URL: {args.url}")
    print(f"Duration: {args.duration} seconds")
    print("-" * 60)
    print("Connecting...")

    # 接続
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect((host, port))

    # HTTPリクエスト
    request = f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nConnection: keep-alive\r\n\r\n"
    sock.sendall(request.encode())

    # HTTPヘッダーをスキップ
    header_buf = b''
    while b'\r\n\r\n' not in header_buf:
        header_buf += sock.recv(1)

    print("Connected. Measuring...\n")

    # 計測
    analyzer = StreamAnalyzer()
    analyzer.start_time = time.perf_counter()

    try:
        fetch_frames(sock, analyzer, args.duration)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        sock.close()

    # 結果表示
    analyzer.print_summary()


if __name__ == "__main__":
    main()
