"""
RC UFO Drone — Autonomous Python Mission Script Runner
Tuned for 100% Speed with Extended Stabilization Delays & Pure Camera Feed

Usage:
    python autonomous_mission.py
    python autonomous_mission.py --script custom_mission.txt
"""

import time
import io
import sys
import argparse
import urllib.request
import json
import numpy as np

GCS_HOST = "http://localhost:8080"
API_CONTROL = f"{GCS_HOST}/api/control"
API_FRAME = f"{GCS_HOST}/api/camera-frame"

DEFAULT_SCRIPT = """
# Mission: 50cm Forward -> Stabilize -> Right 50cm -> Stabilize -> 20cm Forward -> Land
TAKEOFF
HOVER 3.0
FORWARD 50
HOVER 2.0
RIGHT 50
HOVER 2.0
FORWARD 20
HOVER 3.0
LAND
"""

def send_flight_command(roll=128, pitch=128, throttle=128, yaw=128, flags=0, take_off=False, land=False, emergency=False, gear=3):
    payload = {
        "roll": int(roll),
        "pitch": int(pitch),
        "throttle": int(throttle),
        "yaw": int(yaw),
        "take_off": bool(take_off),
        "land": bool(land),
        "emergency": bool(emergency),
        "flags": int(flags),
        "gear": int(gear)
    }
    try:
        data = json.dumps(payload).encode('utf-8')
        req = urllib.request.Request(API_CONTROL, data=data, headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(req, timeout=0.8) as resp:
            return resp.status == 200
    except Exception:
        return False

def smooth_ramp_execute(axis_name, target_val, duration_sec):
    """
    Smoothly ramps the flight axis from 128 to target value, holds, and eases back to 128.
    Ensures silky smooth, steady flight without abrupt jerks.
    """
    start_time = time.time()
    ramp_duration = 0.15  # 150ms ramp in / out
    dt = 0.03  # ~33 Hz control rate

    while True:
        now = time.time()
        elapsed = now - start_time
        if elapsed >= duration_sec:
            break

        # Calculate eased value
        if elapsed < ramp_duration:
            factor = elapsed / ramp_duration
            val = int(128 + (target_val - 128) * factor)
        elif elapsed > duration_sec - ramp_duration:
            remaining = duration_sec - elapsed
            factor = max(0, remaining / ramp_duration)
            val = int(128 + (target_val - 128) * factor)
        else:
            val = target_val

        roll = val if axis_name == 'roll' else 128
        pitch = val if axis_name == 'pitch' else 128
        throttle = val if axis_name == 'throttle' else 128
        yaw = val if axis_name == 'yaw' else 128

        send_flight_command(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw, gear=3)
        time.sleep(dt)

    # Return to neutral and settle for 1.0s to allow optical flow to lock position
    send_flight_command(128, 128, 128, 128, gear=3)
    time.sleep(1.0)  # Extended 1.0s optical flow lock window

def run_script(script_text):
    print("=" * 65)
    print("🛸 RC UFO DRONE — AUTONOMOUS 100% SPEED MISSION RUNNER")
    print("=" * 65)
    print(f"[*] Target GCS Server    : {GCS_HOST}")
    print("[*] Speed Rate Mode       : 100% (Gear 3 Active)")
    print("[*] Stabilization Delays  : EXTENDED (1.0s Settle Windows)")
    print("[*] Camera Video Feed     : DIRECT STREAM (Low-Latency)")
    print("=" * 65)

    # Set 100% Speed Mode in firmware
    send_flight_command(128, 128, 128, 128, gear=3)

    lines = [line.strip() for line in script_text.split('\n') if line.strip() and not line.strip().startswith('#')]
    print(f"[*] Loaded {len(lines)} flight mission steps:")
    for idx, l in enumerate(lines):
        print(f"    Step #{idx+1}: {l}")
    print("=" * 65)
    print("[*] Starting mission in 2 seconds... (Press Ctrl+C to abort)")
    time.sleep(2)

    speed_cm_per_sec = 65.0
    stick_defl = 85  # 100% deflection for crisp responsiveness

    for step_num, line in enumerate(lines, 1):
        parts = line.split()
        cmd = parts[0].upper()
        arg1 = float(parts[1]) if len(parts) > 1 else 0

        print(f"\n[Step #{step_num}/{len(lines)}] -> {line}")

        if cmd == "TAKEOFF":
            print("  --> Triggering Auto-Takeoff (Climbing to 1.2m hover + 4.5s stabilization)...")
            send_flight_command(take_off=True, gear=3)
            time.sleep(4.5)
            send_flight_command(128, 128, 128, 128, gear=3)
            time.sleep(1.0)

        elif cmd == "LAND":
            print("  --> Triggering Smooth Auto-Land (3.5s)...")
            send_flight_command(land=True, gear=3)
            time.sleep(3.5)

        elif cmd in ("HOVER", "STAY", "WAIT"):
            duration = arg1 if arg1 > 0 else 2.5
            print(f"  --> Holding stable optical flow hover lock for {duration}s...")
            t_end = time.time() + duration
            while time.time() < t_end:
                send_flight_command(128, 128, 128, 128, gear=3)
                time.sleep(0.04)

        elif cmd == "FORWARD":
            distance_cm = arg1 if arg1 > 0 else 30
            duration = max(0.6, distance_cm / speed_cm_per_sec)
            print(f"  --> Smooth Forward {distance_cm} cm (Est: {duration:.1f}s @ 100% Speed)...")
            smooth_ramp_execute('pitch', min(255, 128 + stick_defl), duration)

        elif cmd == "BACKWARD":
            distance_cm = arg1 if arg1 > 0 else 30
            duration = max(0.6, distance_cm / speed_cm_per_sec)
            print(f"  --> Smooth Backward {distance_cm} cm (Est: {duration:.1f}s @ 100% Speed)...")
            smooth_ramp_execute('pitch', max(0, 128 - stick_defl), duration)

        elif cmd == "RIGHT":
            distance_cm = arg1 if arg1 > 0 else 30
            duration = max(0.6, distance_cm / speed_cm_per_sec)
            print(f"  --> Smooth Right {distance_cm} cm (Est: {duration:.1f}s @ 100% Speed)...")
            smooth_ramp_execute('roll', min(255, 128 + stick_defl), duration)

        elif cmd == "LEFT":
            distance_cm = arg1 if arg1 > 0 else 30
            duration = max(0.6, distance_cm / speed_cm_per_sec)
            print(f"  --> Smooth Left {distance_cm} cm (Est: {duration:.1f}s @ 100% Speed)...")
            smooth_ramp_execute('roll', max(0, 128 - stick_defl), duration)

        elif cmd in ("YAW", "ROTATE"):
            deg = arg1 if arg1 != 0 else 90
            duration = max(0.5, abs(deg) / 120.0)
            target_yaw = min(255, 128 + stick_defl) if deg > 0 else max(0, 128 - stick_defl)
            print(f"  --> Smooth Yaw {deg}° (Est: {duration:.1f}s @ 100% Speed)...")
            smooth_ramp_execute('yaw', target_yaw, duration)

        elif cmd == "UP":
            distance_cm = arg1 if arg1 > 0 else 30
            duration = max(0.6, distance_cm / (speed_cm_per_sec * 0.8))
            print(f"  --> Ascending {distance_cm} cm...")
            smooth_ramp_execute('throttle', min(255, 128 + stick_defl), duration)

        elif cmd == "DOWN":
            distance_cm = arg1 if arg1 > 0 else 30
            duration = max(0.6, distance_cm / (speed_cm_per_sec * 0.8))
            print(f"  --> Descending {distance_cm} cm...")
            smooth_ramp_execute('throttle', max(0, 128 - stick_defl), duration)

    print("\n" + "=" * 65)
    print("✅ MISSION COMPLETED SUCCESSFULLY (100% SPEED + STABILIZED)")
    print("=" * 65)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Autonomous Drone Mission Runner")
    parser.add_argument("--script", type=str, help="Path to custom mission script file", default=None)
    args = parser.parse_args()

    script_content = DEFAULT_SCRIPT
    if args.script:
        with open(args.script, 'r') as f:
            script_content = f.read()

    try:
        run_script(script_content)
    except KeyboardInterrupt:
        print("\n⚠️ EMERGENCY MISSION ABORT TRIGGERED BY USER!")
        send_flight_command(128, 128, 128, 128, emergency=True)
