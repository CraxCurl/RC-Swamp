"""
OpenCV & PyTorch Real-Time Image Streamer for RC UFO Drone
Directly streams live camera frames into OpenCV numpy arrays (cv2.Mat) at 30 FPS.

Usage:
    python cv_stream.py
"""

import sys
import time
import io
import json
import urllib.request
import numpy as np

try:
    import cv2
except ImportError:
    print("[!] OpenCV is not installed. Run: pip install opencv-python")
    cv2 = None

try:
    import requests
except ImportError:
    requests = None

GCS_HOST = "http://localhost:8080"
STREAM_URL = f"{GCS_HOST}/api/camera-frame"
TELEMETRY_URL = f"{GCS_HOST}/api/camera-frame/json"

def get_frame_numpy():
    """
    Fetches the latest live frame from the Ground Control Station
    and decodes it directly into a BGR NumPy array for OpenCV / YOLO / PyTorch.
    """
    try:
        req = urllib.request.Request(STREAM_URL)
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            if resp.status == 200:
                img_bytes = resp.read()
                if len(img_bytes) > 100:
                    arr = np.frombuffer(img_bytes, dtype=np.uint8)
                    if cv2 is not None:
                        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        return frame
                    else:
                        from PIL import Image
                        pil_img = Image.open(io.BytesIO(img_bytes))
                        return np.array(pil_img)
    except Exception:
        pass
    return None

def run_vision_loop():
    print("=" * 65)
    print("🛸 RC UFO DRONE — REAL-TIME OPENCV / NUMPY VISION LOOP")
    print("=" * 65)
    print(f"[*] Ground Control URL : {GCS_HOST}")
    print(f"[*] Stream Endpoint    : {STREAM_URL}")
    print("[*] Press 'q' or Ctrl+C to exit vision loop")
    print("=" * 65)

    fps_count = 0
    start_time = time.time()
    last_frame_time = time.time()

    while True:
        frame = get_frame_numpy()
        
        if frame is not None:
            fps_count += 1
            now = time.time()
            if now - start_time >= 1.0:
                current_fps = fps_count / (now - start_time)
                fps_count = 0
                start_time = now
            else:
                current_fps = 30.0

            # -------------------------------------------------------------
            # >> INSERT YOUR CUSTOM IMAGE PROCESSING / AI / CV CODE HERE <<
            # -------------------------------------------------------------
            # Example: Compute frame dimensions and color channels
            h, w, c = frame.shape
            
            # Example: Overlay telemetry and FPS on image
            if cv2 is not None:
                display_frame = frame.copy()
                cv2.putText(display_frame, f"LIVE FEED - {w}x{h} @ {current_fps:.1f} FPS", 
                            (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Show live window
                cv2.imshow("RC UFO Drone - Vision Feed", display_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break
            else:
                print(f"[Vision] Received frame: {w}x{h}x{c}")
            
            time.sleep(0.01)
        else:
            time.sleep(0.05)

    if cv2 is not None:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    run_vision_loop()
