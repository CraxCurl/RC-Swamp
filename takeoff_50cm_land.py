"""
RC UFO Drone — Autonomous Mission: Takeoff to 50cm, Turn 180° & Land (Zero Delay)

Usage:
    python takeoff_50cm_land.py
"""

import time
import json
import urllib.request
import sys

GCS_HOST = "http://localhost:8080"
API_CONTROL = f"{GCS_HOST}/api/control"

def send_command(roll=128, pitch=128, throttle=128, yaw=128, take_off=False, land=False, emergency=False, gear=3):
    """Sends a flight command JSON payload to the Ground Control Station API."""
    payload = {
        "roll": int(roll),
        "pitch": int(pitch),
        "throttle": int(throttle),
        "yaw": int(yaw),
        "take_off": bool(take_off),
        "land": bool(land),
        "emergency": bool(emergency),
        "gear": int(gear)
    }
    try:
        data = json.dumps(payload).encode('utf-8')
        req = urllib.request.Request(API_CONTROL, data=data, headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            return resp.status == 200
    except Exception:
        print(f"[!] Could not connect to GCS API at {GCS_HOST}. Ensure server is running (`npm start` or `python server.py`).")
        return False

def run_takeoff_180_land_mission():
    print("=" * 65)
    print("🛸 RC UFO DRONE — TAKEOFF TO 50CM, TURN 180° & LAND")
    print("=" * 65)
    print(f"[*] Target Endpoint    : {GCS_HOST}")
    print("[*] Target Height      : ~50 cm")
    print("[*] Rotation           : 180° Turn")
    print("[*] Stabilization Delay: 0.0s (Instant transitions)")
    print("=" * 65)
    print("[*] Starting mission in 2 seconds... (Press Ctrl+C to abort)")
    time.sleep(2)

    try:
        # STEP 1: Auto-Takeoff
        print("\n[1/3] 🚀 Triggering Auto-Takeoff...")
        if not send_command(take_off=True, gear=3):
            print("[!] Takeoff signal failed. Aborting.")
            return
        time.sleep(2.0)  # Fast 2s takeoff climb

        # STEP 2: Turn 180 Degrees
        print("\n[2/3] 🔄 Turning 180 degrees...")
        yaw_start = time.time()
        # Full right yaw deflection (yaw=213) for ~1.2s to turn 180°
        while time.time() - yaw_start < 1.2:
            send_command(roll=128, pitch=128, throttle=128, yaw=213, gear=3)
            time.sleep(0.04)

        # Stop rotation
        send_command(128, 128, 128, 128, gear=3)

        # STEP 3: Land Immediately
        print("\n[3/3] 🛬 Landing straightaway...")
        send_command(land=True, gear=3)
        time.sleep(2.5)

        print("\n" + "=" * 65)
        print("✅ MISSION COMPLETED SUCCESSFULLY!")
        print("=" * 65)

    except KeyboardInterrupt:
        print("\n⚠️ Emergency abort! Landing drone...")
        send_command(emergency=True)
        send_command(land=True)
        sys.exit(1)

if __name__ == "__main__":
    run_takeoff_180_land_mission()
