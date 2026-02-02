#!/usr/bin/env python3
"""
Detection Overlay UI Simulator

Simulates the Tab5 display overlay to test UI designs before deploying.
- Portrait framebuffer: 720x1280
- Landscape viewing: Device rotated 90° CCW
- Top bar (camera side): rows 0-159 in framebuffer → right side in landscape
- Bottom bar (USB side): rows 1120-1279 in framebuffer → left side in landscape
"""

import json
import time
import threading
import requests
import urllib3
from flask import Flask, render_template, jsonify, Response
from dataclasses import dataclass, field, asdict
from typing import List, Optional
import queue

# Disable SSL warnings for self-signed certs
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

app = Flask(__name__)

# Configuration
DETECTION_URL = "https://192.168.1.33:8080/api/detections/stream?format=json"
MJPEG_URL = "http://192.168.1.33:8082/stream"

# Display dimensions (matching PipelineConfig.h)
PANEL_WIDTH = 720
PANEL_HEIGHT = 1280
OVERLAY_BAR_SIZE = 160
VIDEO_HEIGHT = 960

# Shared state
@dataclass
class Detection:
    label: str = "unknown"
    confidence: float = 0.0

@dataclass
class AppState:
    detections: List[Detection] = field(default_factory=list)
    timestamp: int = 0
    connected: bool = False
    error: Optional[str] = None
    last_update: float = 0

state = AppState()
state_lock = threading.Lock()
sse_clients: List[queue.Queue] = []
log_clients: List[queue.Queue] = []  # For raw API event log


def broadcast_log(event_type: str, content: str):
    """Send raw API event to log clients"""
    log_entry = {
        "type": event_type,
        "content": content,
        "time": time.strftime("%H:%M:%S")
    }
    message = f"data: {json.dumps(log_entry)}\n\n"

    dead_clients = []
    for q in log_clients:
        try:
            q.put_nowait(message)
        except queue.Full:
            dead_clients.append(q)

    for q in dead_clients:
        log_clients.remove(q)


def sse_listener():
    """Background thread to listen to detection SSE stream"""
    global state

    while True:
        try:
            with state_lock:
                state.connected = False
                state.error = None

            print(f"Connecting to: {DETECTION_URL}")
            broadcast_log("connect", f"Connecting to {DETECTION_URL}...")
            response = requests.get(DETECTION_URL, stream=True, verify=False, timeout=30)
            response.raise_for_status()

            with state_lock:
                state.connected = True
                state.error = None

            print("SSE stream connected")
            broadcast_log("connect", "Connected successfully")
            broadcast_state()

            for line in response.iter_lines(decode_unicode=True):
                if not line:
                    continue

                # SSE data line
                if line.startswith("data:"):
                    json_str = line[5:].strip()
                    if json_str:
                        try:
                            data = json.loads(json_str)

                            # Debug: Log raw JSON structure
                            raw_dets = data.get("detections", [])
                            print(f"=== RAW JSON ===")
                            print(f"Full data: {data}")
                            if raw_dets:
                                # Show actual field names for first detection
                                first = raw_dets[0]
                                fields = list(first.keys())
                                print(f"Fields in detection[0]: {fields}")
                                print(f"Raw detection[0]: {first}")
                                # Check each possible field
                                print(f"  class_name: {first.get('class_name', 'NOT FOUND')}")
                                print(f"  label: {first.get('label', 'NOT FOUND')}")
                                print(f"  name: {first.get('name', 'NOT FOUND')}")
                                print(f"  className: {first.get('className', 'NOT FOUND')}")
                            print(f"================")

                            detections = []
                            for det in raw_dets:
                                # Try multiple field names
                                label = det.get("class_name") or det.get("label") or det.get("name") or "unknown"
                                detections.append(Detection(
                                    label=label,
                                    confidence=det.get("confidence", 0.0)
                                ))

                            with state_lock:
                                state.detections = detections
                                state.timestamp = int(time.time() * 1000)
                                state.last_update = time.time()

                            # Log the detection data with raw field info
                            if raw_dets:
                                first = raw_dets[0]
                                fields_info = f"[fields: {list(first.keys())}] "
                            else:
                                fields_info = ""
                            det_summary = ", ".join([f"{d.label}({d.confidence:.0%})" for d in detections])
                            broadcast_log("data", f"{fields_info}[{len(detections)}] {det_summary}" if detections else "[0] No detections")
                            broadcast_state()

                        except json.JSONDecodeError as e:
                            print(f"JSON parse error: {e}")
                            broadcast_log("error", f"JSON parse error: {e}")

                # Keepalive (comment line starting with :)
                elif line.startswith(":"):
                    print(f"Keepalive: {line}")
                    broadcast_log("keepalive", line[1:].strip() or "ping")

        except requests.exceptions.RequestException as e:
            print(f"Connection error: {e}")
            broadcast_log("error", f"Connection error: {e}")
            with state_lock:
                state.connected = False
                state.error = str(e)
            broadcast_state()
            time.sleep(3)
        except Exception as e:
            print(f"Unexpected error: {e}")
            broadcast_log("error", f"Unexpected error: {e}")
            with state_lock:
                state.connected = False
                state.error = str(e)
            broadcast_state()
            time.sleep(3)


def broadcast_state():
    """Send state update to all connected SSE clients"""
    with state_lock:
        data = {
            "detections": [asdict(d) for d in state.detections],
            "timestamp": state.timestamp,
            "connected": state.connected,
            "error": state.error,
            "count": len(state.detections)
        }

    message = f"data: {json.dumps(data)}\n\n"

    # Clean up disconnected clients
    dead_clients = []
    for q in sse_clients:
        try:
            q.put_nowait(message)
        except queue.Full:
            dead_clients.append(q)

    for q in dead_clients:
        sse_clients.remove(q)


@app.route("/")
def index():
    return render_template("index.html",
                           panel_width=PANEL_WIDTH,
                           panel_height=PANEL_HEIGHT,
                           overlay_bar_size=OVERLAY_BAR_SIZE,
                           video_height=VIDEO_HEIGHT)


@app.route("/api/state")
def get_state():
    with state_lock:
        return jsonify({
            "detections": [asdict(d) for d in state.detections],
            "timestamp": state.timestamp,
            "connected": state.connected,
            "error": state.error,
            "count": len(state.detections)
        })


@app.route("/api/mock", methods=["POST"])
def set_mock():
    """Set mock detection data for testing"""
    from flask import request
    data = request.json

    with state_lock:
        state.detections = [
            Detection(
                label=d.get("class_name") or d.get("label") or d.get("name") or "unknown",
                confidence=d.get("confidence", 0.0)
            )
            for d in data.get("detections", [])
        ]
        state.timestamp = int(time.time() * 1000)
        state.last_update = time.time()

    broadcast_state()
    return jsonify({"status": "ok"})


@app.route("/api/mjpeg")
def mjpeg_proxy():
    """Proxy MJPEG stream to avoid CORS issues"""
    def generate():
        try:
            response = requests.get(MJPEG_URL, stream=True, timeout=10)
            response.raise_for_status()
            for chunk in response.iter_content(chunk_size=4096):
                if chunk:
                    yield chunk
        except Exception as e:
            print(f"MJPEG proxy error: {e}")

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/stream")
def stream():
    """SSE endpoint for real-time updates to browser"""
    def generate():
        q = queue.Queue(maxsize=10)
        sse_clients.append(q)

        # Send initial state
        with state_lock:
            data = {
                "detections": [asdict(d) for d in state.detections],
                "timestamp": state.timestamp,
                "connected": state.connected,
                "error": state.error,
                "count": len(state.detections)
            }
        yield f"data: {json.dumps(data)}\n\n"

        try:
            while True:
                try:
                    message = q.get(timeout=30)
                    yield message
                except queue.Empty:
                    yield ": keepalive\n\n"
        finally:
            if q in sse_clients:
                sse_clients.remove(q)

    return Response(generate(), mimetype="text/event-stream")


@app.route("/api/log_stream")
def log_stream():
    """SSE endpoint for raw API event log"""
    def generate():
        q = queue.Queue(maxsize=50)
        log_clients.append(q)

        try:
            while True:
                try:
                    message = q.get(timeout=30)
                    yield message
                except queue.Empty:
                    yield ": keepalive\n\n"
        finally:
            if q in log_clients:
                log_clients.remove(q)

    return Response(generate(), mimetype="text/event-stream")


def main():
    """Entry point for uv run"""
    # Start SSE listener thread
    listener_thread = threading.Thread(target=sse_listener, daemon=True)
    listener_thread.start()

    print("=" * 60)
    print("Detection Overlay UI Simulator")
    print("=" * 60)
    print(f"MJPEG Stream:   {MJPEG_URL}")
    print(f"Detection API:  {DETECTION_URL}")
    print(f"Display:        {PANEL_WIDTH}x{PANEL_HEIGHT} (landscape view)")
    print(f"Overlay bars:   {OVERLAY_BAR_SIZE}px each")
    print("=" * 60)
    print("Open http://localhost:5000 in your browser")
    print("=" * 60)

    app.run(debug=True, host="0.0.0.0", port=5000, threaded=True)


if __name__ == "__main__":
    main()
