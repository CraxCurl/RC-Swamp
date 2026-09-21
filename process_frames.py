"""
RC UFO Drone — Live Camera Image Processing & Vision Pipeline
This script pulls real-time camera frames from the Ground Control Station (GCS)
and demonstrates how to run image processing, computer vision, and AI models on the drone's feed.
"""

import time
import io
import os
import sys
import base64
import json

try:
    import requests
    from PIL import Image, ImageFilter, ImageStat
    import numpy as np
except ImportError as e:
    print(f"[Error] Missing dependency: {e}")
    print("[*] Install required packages via: pip install requests pillow numpy")
    sys.exit(1)

GCS_HOST = "http://localhost:8080"
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "processed_output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

def fetch_latest_frame():
    """
    Fetches the latest live camera frame as a PIL Image along with synchronized telemetry.
    """
    try:
        res = requests.get(f"{GCS_HOST}/api/camera-frame/json", timeout=2.0)
        if res.status_code == 200:
            data = res.json()
            img_bytes = base64.b64decode(data['image_base64'])
            img = Image.open(io.BytesIO(img_bytes))
            return img, data['telemetry'], data['frame_number']
    except Exception as e:
        pass
    return None, None, None

def process_frame(img: Image.Image, frame_num: int, telemetry: dict):
    """
    Example image processing function:
    1. Converts image to NumPy array for fast computation
    2. Calculates scene brightness & edge detection
    3. Saves processed visual output to disk
    """
    # 1. Convert to NumPy array (RGB)
    img_np = np.array(img)
    height, width, channels = img_np.shape

    # 2. Basic Image Statistics (Brightness, Color Dominance)
    stat = ImageStat.Stat(img)
    avg_brightness = sum(stat.mean[:3]) / 3.0

    # 3. Edge Detection filter
    edges = img.filter(ImageFilter.FIND_EDGES)

    # 4. Color Mask Example: Detect Bright / Red Objects
    r, g, b = img_np[:, :, 0], img_np[:, :, 1], img_np[:, :, 2]
    # Simple red mask condition
    red_mask = (r > 150) & (g < 100) & (b < 100)
    red_pixel_count = int(np.sum(red_mask))

    print(f"[Frame #{frame_num:05d}] Size: {width}x{height} | Brightness: {avg_brightness:.1f} | Red Pixels: {red_pixel_count} | Roll: {telemetry.get('roll')} Pitch: {telemetry.get('pitch')}")

    # 5. Save sample processed frame every 10 frames
    if frame_num % 10 == 0:
        out_path = os.path.join(OUTPUT_DIR, f"edge_frame_{frame_num:05d}.jpg")
        edges.save(out_path)

def main():
    print("=" * 65)
    print("🛸 RC UFO DRONE LIVE IMAGE PROCESSING PIPELINE")
    print("=" * 65)
    print(f"[*] Target GCS Server: {GCS_HOST}")
    print(f"[*] Output Directory : {OUTPUT_DIR}")
    print("=" * 65)
    print("[*] Listening for live camera frames... (Press Ctrl+C to stop)")

    last_processed_frame = -1

    while True:
        img, telemetry, frame_num = fetch_latest_frame()
        if img and frame_num != last_processed_frame:
            last_processed_frame = frame_num
            process_frame(img, frame_num, telemetry)
            time.sleep(0.03) # ~30 FPS
        else:
            time.sleep(0.1)

if __name__ == "__main__":
    main()
