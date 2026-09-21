"""
RC UFO Drone — Autonomous Mission: Takeoff to 1m, 360° Flip & Land

Usage:
    python takeoff_1m_flip_land.py
"""

import time
import json
import urllib.request
import sys

GCS_HOST = "http://localhost:8080"
API_CONTROL = f"{GCS_HOST}/api/control"

def send_command(roll=128, pitch=128, throttle=128, yaw=128, take_off=False, land=False, emergency=False, action=None, direction=None, gear=3):
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
    if action:
        payload["action"] = action
    if direction:
        payload["direction"] = direction

    try:
        data = json.dumps(payload).encode('utf-8')
        req = urllib.request.Request(API_CONTROL, data=data, headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            return resp.status == 200
    except Exception:
        print(f"[!] Could not connect to GCS API at {GCS_HOST}. Ensure server is running (`npm start` or `python server.py`).")
        return False

def run_takeoff_1m_flip_land_mission():
    print("=" * 65)
    print("🛸 RC UFO DRONE — TAKEOFF TO 1M, 360° STUNT FLIP & LAND")
    print("=" * 65)
    print(f"[*] Target Endpoint    : {GCS_HOST}")
    print("[*] Target Height      : ~1.0 Metre (100 cm)")
    print("[*] Maneuver           : 360° Forward Stunt Flip")
    print("[*] Speed Rate Mode    : 100% (Gear 3 Active)")
    print("=" * 65)
    print("[*] Starting mission in 2 seconds... (Press Ctrl+C to abort)")
    time.sleep(2)

    try:
        # STEP 1: Auto-Takeoff (Reaches ~50cm default hover altitude)
        print("\n[1/4] 🚀 Step 1: Triggering Auto-Takeoff (~50cm base altitude)...")
        if not send_command(take_off=True, gear=3):
            print("[!] Takeoff signal failed. Aborting.")
            return
        time.sleep(2.5)  # Allow takeoff climb & optical flow lock

        # STEP 2: Ascend 50cm to reach 1 Metre
        print("\n[2/4] ⬆️ Step 2: Ascending +50cm to reach 1 metre altitude...")
        climb_start = time.time()
        # Throttle up deflection (throttle=213) for ~1.0 second to climb +50cm
        while time.time() - climb_start < 1.0:
            send_command(roll=128, pitch=128, throttle=213, yaw=128, gear=3)
            time.sleep(0.04)

        # Settle back to neutral hover at 1m height
        send_command(128, 128, 128, 128, gear=3)
        time.sleep(1.0)  # Settle stabilization window

        # STEP 3: Perform 360° Stunt Flip
        print("\n[3/4] 🔄 Step 3: Triggering 360° Forward Stunt Flip...")
        if not send_command(action="flip_360", direction="forward", gear=3):
            print("[!] Flip command failed.")
        time.sleep(1.8)  # Wait for flip execution and hover stabilization

        # STEP 4: Land Smoothly
        print("\n[4/4] 🛬 Step 4: Landing drone smoothly...")
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
    run_takeoff_1m_flip_land_mission()
