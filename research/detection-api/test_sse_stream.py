#!/usr/bin/env python3
"""
Detection API SSE Stream Tester

Tests the SSE stream endpoint and shows the JSON structure
to verify parsing logic in DetectionTask.cpp
"""

import requests
import json
import sys
import urllib3

# Disable SSL warnings for self-signed certs
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# Default URL from .env
DEFAULT_URL = "https://192.168.1.33:8080/api/detections/stream?format=json"

def test_sse_stream(url: str, max_events: int = 10):
    """Connect to SSE stream and print received events"""

    print(f"Connecting to: {url}")
    print("-" * 60)

    try:
        response = requests.get(url, stream=True, verify=False, timeout=30)
        response.raise_for_status()

        print(f"Status: {response.status_code}")
        print(f"Content-Type: {response.headers.get('Content-Type', 'N/A')}")
        print("-" * 60)

        event_count = 0
        for line in response.iter_lines(decode_unicode=True):
            if not line:
                continue

            print(f"RAW: {repr(line)}")

            # SSE format: "data: {...}"
            if line.startswith("data:"):
                json_str = line[5:].strip()  # Remove "data:" prefix
                print(f"JSON string: {json_str}")

                try:
                    data = json.loads(json_str)
                    print(f"Parsed JSON:")
                    print(json.dumps(data, indent=2, ensure_ascii=False))

                    # Check structure
                    if "detections" in data:
                        print(f"\n=== Detection Analysis ===")
                        print(f"Number of detections: {len(data['detections'])}")
                        for i, det in enumerate(data["detections"]):
                            print(f"  [{i}] keys: {list(det.keys())}")
                            print(f"      label: {det.get('label', 'MISSING')}")
                            print(f"      confidence: {det.get('confidence', 'MISSING')}")
                            # Check for alternative key names
                            if 'class' in det:
                                print(f"      class: {det['class']}")
                            if 'name' in det:
                                print(f"      name: {det['name']}")
                            if 'score' in det:
                                print(f"      score: {det['score']}")
                    else:
                        print(f"Keys in root: {list(data.keys())}")

                except json.JSONDecodeError as e:
                    print(f"JSON parse error: {e}")

                print("-" * 60)
                event_count += 1

                if event_count >= max_events:
                    print(f"\nReached {max_events} events, stopping.")
                    break

    except requests.exceptions.RequestException as e:
        print(f"Request error: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 0

    return 0

if __name__ == "__main__":
    url = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_URL
    max_events = int(sys.argv[2]) if len(sys.argv) > 2 else 10

    print(f"Detection API SSE Stream Tester")
    print(f"URL: {url}")
    print(f"Max events: {max_events}")
    print()

    sys.exit(test_sse_stream(url, max_events))
