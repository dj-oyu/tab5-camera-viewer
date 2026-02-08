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
DEVICE_BASE_URL = "https://192.168.1.33:8080"
DETECTION_URL = f"{DEVICE_BASE_URL}/api/detections/stream?format=json"
MJPEG_URL = "http://192.168.1.33:8082/stream"
CONNECTIONS_URL = f"{DEVICE_BASE_URL}/api/connections/stream"
RECORDING_BASE_URL = f"{DEVICE_BASE_URL}/api/recording"

# Display dimensions (matching PipelineConfig.h)
PANEL_WIDTH = 720
PANEL_HEIGHT = 1280
OVERLAY_BAR_SIZE = 160
VIDEO_HEIGHT = 960

# Detection timeout - clear detections if no data for this many seconds
DETECTION_TIMEOUT_SEC = 3.0

# Shared state
@dataclass
class Detection:
    label: str = "unknown"
    confidence: float = 0.0

@dataclass
class ConnectionStats:
    webrtc: int = 0
    mjpeg: int = 0
    total: int = 0
    timestamp: int = 0
    connected: bool = False

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

# Connection stats
conn_stats = ConnectionStats()
conn_stats_lock = threading.Lock()
conn_clients: List[queue.Queue] = []  # SSE clients for connection updates

# Recording state
@dataclass
class RecordingState:
    recording: bool = False
    filename: str = ""
    duration: float = 0.0
    error: Optional[str] = None

recording_state = RecordingState()
recording_state_lock = threading.Lock()
recording_clients: List[queue.Queue] = []  # SSE clients for recording updates

# Battery state
@dataclass
class BatteryState:
    percent: int = 75
    charging: bool = False

battery_state = BatteryState()
battery_state_lock = threading.Lock()
battery_clients: List[queue.Queue] = []  # SSE clients for battery updates


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

            broadcast_log("connect", f"Connecting to {DETECTION_URL}...")
            response = requests.get(DETECTION_URL, stream=True, verify=False, timeout=30)
            response.raise_for_status()

            with state_lock:
                state.connected = True
                state.error = None

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

                            raw_dets = data.get("detections", [])

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
                            broadcast_log("error", f"JSON parse error: {e}")

                # Keepalive (comment line starting with :)
                elif line.startswith(":"):
                    broadcast_log("keepalive", line[1:].strip() or "ping")

                    # Check if detections should be cleared due to timeout
                    with state_lock:
                        if state.detections and state.last_update > 0:
                            elapsed = time.time() - state.last_update
                            if elapsed > DETECTION_TIMEOUT_SEC:
                                state.detections = []
                                state.timestamp = int(time.time() * 1000)
                                broadcast_log("data", "[timeout] No detections")
                    broadcast_state()

        except requests.exceptions.RequestException as e:
            broadcast_log("error", f"Connection error: {e}")
            with state_lock:
                state.connected = False
                state.error = str(e)
            broadcast_state()
            time.sleep(3)
        except Exception as e:
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


def broadcast_conn_stats():
    """Send connection stats update to all connected SSE clients"""
    with conn_stats_lock:
        data = {
            "webrtc": conn_stats.webrtc,
            "mjpeg": conn_stats.mjpeg,
            "total": conn_stats.total,
            "timestamp": conn_stats.timestamp,
            "connected": conn_stats.connected
        }

    message = f"data: {json.dumps(data)}\n\n"

    dead_clients = []
    for q in conn_clients:
        try:
            q.put_nowait(message)
        except queue.Full:
            dead_clients.append(q)

    for q in dead_clients:
        conn_clients.remove(q)


def conn_sse_listener():
    """Background thread to listen to connection stats SSE stream"""
    global conn_stats

    while True:
        try:
            with conn_stats_lock:
                conn_stats.connected = False

            broadcast_log("connect", f"Connecting to {CONNECTIONS_URL}...")
            response = requests.get(CONNECTIONS_URL, stream=True, verify=False, timeout=30)
            response.raise_for_status()

            with conn_stats_lock:
                conn_stats.connected = True

            broadcast_log("connect", "Connections API connected")
            broadcast_conn_stats()

            for line in response.iter_lines(decode_unicode=True):
                if not line:
                    continue

                if line.startswith("data:"):
                    json_str = line[5:].strip()
                    if json_str:
                        try:
                            data = json.loads(json_str)

                            with conn_stats_lock:
                                conn_stats.webrtc = data.get("webrtc", 0)
                                conn_stats.mjpeg = data.get("mjpeg", 0)
                                conn_stats.total = conn_stats.webrtc + conn_stats.mjpeg
                                conn_stats.timestamp = int(time.time() * 1000)

                            broadcast_conn_stats()

                        except json.JSONDecodeError:
                            pass  # Ignore parse errors

                elif line.startswith(":"):
                    pass  # Keepalive

        except requests.exceptions.RequestException:
            with conn_stats_lock:
                conn_stats.connected = False
            broadcast_conn_stats()
            time.sleep(3)
        except Exception:
            with conn_stats_lock:
                conn_stats.connected = False
            broadcast_conn_stats()
            time.sleep(3)


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


@app.route("/api/connections")
def get_connections():
    """Get current connection stats"""
    with conn_stats_lock:
        return jsonify({
            "webrtc": conn_stats.webrtc,
            "mjpeg": conn_stats.mjpeg,
            "total": conn_stats.total,
            "timestamp": conn_stats.timestamp,
            "connected": conn_stats.connected
        })


@app.route("/api/connections/stream")
def connections_stream():
    """SSE endpoint for real-time connection updates"""
    def generate():
        q = queue.Queue(maxsize=10)
        conn_clients.append(q)

        # Send initial state
        with conn_stats_lock:
            data = {
                "webrtc": conn_stats.webrtc,
                "mjpeg": conn_stats.mjpeg,
                "total": conn_stats.total,
                "timestamp": conn_stats.timestamp,
                "connected": conn_stats.connected
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
            if q in conn_clients:
                conn_clients.remove(q)

    return Response(generate(), mimetype="text/event-stream")


@app.route("/api/connections/mock", methods=["POST"])
def set_mock_connections():
    """Set mock connection data for testing"""
    from flask import request
    data = request.json

    with conn_stats_lock:
        conn_stats.webrtc = data.get("webrtc", conn_stats.webrtc)
        conn_stats.mjpeg = data.get("mjpeg", conn_stats.mjpeg)
        conn_stats.total = conn_stats.webrtc + conn_stats.mjpeg
        conn_stats.timestamp = int(time.time() * 1000)

    broadcast_conn_stats()
    return jsonify({"status": "ok"})


# Recording API - Proxy to device
recording_start_time: Optional[float] = None  # Local fallback for duration tracking


def broadcast_recording_state():
    """Send recording state update to all connected SSE clients"""
    with recording_state_lock:
        data = {
            "recording": recording_state.recording,
            "filename": recording_state.filename,
            "duration": recording_state.duration,
            "error": recording_state.error
        }

    message = f"data: {json.dumps(data)}\n\n"

    dead_clients = []
    for q in recording_clients:
        try:
            q.put_nowait(message)
        except queue.Full:
            dead_clients.append(q)

    for q in dead_clients:
        recording_clients.remove(q)


def broadcast_battery_state():
    """Send battery state update to all connected SSE clients"""
    with battery_state_lock:
        data = {
            "percent": battery_state.percent,
            "charging": battery_state.charging
        }

    message = f"data: {json.dumps(data)}\n\n"

    dead_clients = []
    for q in battery_clients:
        try:
            q.put_nowait(message)
        except queue.Full:
            dead_clients.append(q)

    for q in dead_clients:
        battery_clients.remove(q)


@app.route("/api/recording/start", methods=["POST"])
def recording_start():
    """Start recording - proxy to device"""
    global recording_start_time
    print(f"[Recording] Start requested -> {RECORDING_BASE_URL}/start")

    try:
        resp = requests.post(f"{RECORDING_BASE_URL}/start", timeout=5, verify=False)
        data = resp.json()

        if resp.status_code == 200:
            recording_start_time = time.time()  # Local fallback tracking
            with recording_state_lock:
                recording_state.recording = True
                recording_state.filename = data.get("filename", "")
                recording_state.duration = 0.0
                recording_state.error = None

            broadcast_recording_state()
            broadcast_log("recording", f"Started: {recording_state.filename}")
            print(f"[Recording] Started: {recording_state.filename}")

        return jsonify(data), resp.status_code

    except requests.exceptions.RequestException as e:
        error_msg = f"Device connection failed: {e}"
        print(f"[Recording] Error: {error_msg}")
        with recording_state_lock:
            recording_state.error = error_msg
        broadcast_recording_state()
        return jsonify({"error": error_msg}), 503


@app.route("/api/recording/stop", methods=["POST"])
def recording_stop():
    """Stop recording - proxy to device"""
    global recording_start_time
    print(f"[Recording] Stop requested -> {RECORDING_BASE_URL}/stop")

    try:
        resp = requests.post(f"{RECORDING_BASE_URL}/stop", timeout=5, verify=False)
        data = resp.json()

        # Log raw response from device
        print(f"[Recording] Device response: {json.dumps(data, indent=2)}")

        if resp.status_code == 200:
            # Get duration from device API (duration_ms in stats)
            device_duration_ms = data.get("stats", {}).get("duration_ms", 0)
            device_duration = device_duration_ms / 1000.0  # Convert ms to seconds

            # Fallback to local tracking if device returns 0
            local_duration = time.time() - recording_start_time if recording_start_time else 0.0
            final_duration = device_duration if device_duration > 0 else local_duration

            recording_start_time = None
            with recording_state_lock:
                recording_state.recording = False
                recording_state.duration = final_duration
                recording_state.error = None

            broadcast_recording_state()
            source = "device API" if device_duration > 0 else "local tracking"
            broadcast_log("recording", f"Stopped: {final_duration:.1f}s ({source})")
            print(f"[Recording] Stopped: {final_duration:.1f}s ({source})")

        return jsonify(data), resp.status_code

    except requests.exceptions.RequestException as e:
        error_msg = f"Device connection failed: {e}"
        print(f"[Recording] Error: {error_msg}")
        with recording_state_lock:
            recording_state.error = error_msg
        broadcast_recording_state()
        return jsonify({"error": error_msg}), 503


@app.route("/api/recording/heartbeat", methods=["POST"])
def recording_heartbeat():
    """Send heartbeat - proxy to device"""
    try:
        resp = requests.post(f"{RECORDING_BASE_URL}/heartbeat", timeout=2, verify=False)
        return jsonify(resp.json()), resp.status_code
    except requests.exceptions.RequestException as e:
        return jsonify({"error": str(e)}), 503


@app.route("/api/recording/status")
def recording_status():
    """Get recording status - proxy to device"""
    try:
        resp = requests.get(f"{RECORDING_BASE_URL}/status", timeout=2, verify=False)
        data = resp.json()

        # Update local state
        with recording_state_lock:
            recording_state.recording = data.get("recording", False)
            recording_state.filename = data.get("filename", "")
            recording_state.duration = data.get("duration", 0.0)

        return jsonify(data), resp.status_code

    except requests.exceptions.RequestException as e:
        return jsonify({"error": str(e)}), 503


@app.route("/api/recording/stream")
def recording_stream():
    """SSE endpoint for real-time recording status updates - polls device"""
    def generate():
        q = queue.Queue(maxsize=10)
        recording_clients.append(q)

        # Send initial state
        with recording_state_lock:
            data = {
                "recording": recording_state.recording,
                "filename": recording_state.filename,
                "duration": recording_state.duration,
                "error": recording_state.error
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
            if q in recording_clients:
                recording_clients.remove(q)

    return Response(generate(), mimetype="text/event-stream")


@app.route("/api/battery")
def get_battery():
    """Get current battery state"""
    with battery_state_lock:
        return jsonify({
            "percent": battery_state.percent,
            "charging": battery_state.charging
        })


@app.route("/api/battery/stream")
def battery_stream():
    """SSE endpoint for real-time battery updates"""
    def generate():
        q = queue.Queue(maxsize=10)
        battery_clients.append(q)

        # Send initial state
        with battery_state_lock:
            data = {
                "percent": battery_state.percent,
                "charging": battery_state.charging
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
            if q in battery_clients:
                battery_clients.remove(q)

    return Response(generate(), mimetype="text/event-stream")


@app.route("/api/battery/mock", methods=["POST"])
def set_mock_battery():
    """Set mock battery data for testing"""
    from flask import request
    data = request.json

    with battery_state_lock:
        battery_state.percent = max(0, min(100, data.get("percent", battery_state.percent)))
        battery_state.charging = data.get("charging", battery_state.charging)

    broadcast_battery_state()
    return jsonify({"status": "ok"})


def recording_status_poller():
    """Update recording duration locally (device doesn't provide duration during recording)"""
    global recording_start_time

    while True:
        time.sleep(1)

        with recording_state_lock:
            is_recording = recording_state.recording

        if is_recording and recording_start_time:
            # Update duration using local time tracking
            local_duration = time.time() - recording_start_time

            with recording_state_lock:
                recording_state.duration = local_duration

            broadcast_recording_state()


def main():
    """Entry point for uv run"""
    # Start SSE listener threads
    detection_thread = threading.Thread(target=sse_listener, daemon=True)
    detection_thread.start()

    connections_thread = threading.Thread(target=conn_sse_listener, daemon=True)
    connections_thread.start()

    # Start recording status polling thread
    recording_thread = threading.Thread(target=recording_status_poller, daemon=True)
    recording_thread.start()

    print("=" * 60)
    print("Detection Overlay UI Simulator")
    print("=" * 60)
    print(f"MJPEG Stream:     {MJPEG_URL}")
    print(f"Detection API:    {DETECTION_URL}")
    print(f"Connections API:  {CONNECTIONS_URL}")
    print(f"Recording API:    {RECORDING_BASE_URL}/{{start,stop,status,heartbeat}} (proxied)")
    print(f"Display:          {PANEL_WIDTH}x{PANEL_HEIGHT} (landscape view)")
    print(f"Overlay bars:     {OVERLAY_BAR_SIZE}px each")
    print("=" * 60)
    print("Open http://localhost:5000 in your browser")
    print("=" * 60)

    # use_reloader=False to avoid dual-process issues with global state
    app.run(debug=True, host="0.0.0.0", port=5000, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
