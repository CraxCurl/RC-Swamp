import asyncio
import io
import math
import os
import socket
import sys
import time
import threading
from typing import Set

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from PIL import Image, ImageDraw, ImageFont
import av

# Configuration Constants
DRONE_IP = "192.168.1.1"
UDP_CONTROL_PORT = 7099
RTSP_URL = "rtsp://192.168.1.1:7070/webcam"
WEB_PORT = 8080

app = FastAPI(title="RC UFO Drone Web Controller")

# Global State
class DroneState:
    def __init__(self):
        self.device_type = 2  # 2 = GL Protocol (21 Bytes), 10 = Legacy (9 Bytes)
        self.connected_to_drone = False
        self.drone_ip = DRONE_IP
        self.udp_port = UDP_CONTROL_PORT
        self.rtsp_url = RTSP_URL
        
        # Flight channels (1..255, 128 = center)
        self.roll = 128
        self.pitch = 128
        self.throttle = 128
        self.yaw = 128
        
        # Flags
        self.is_fast_fly = False       # Takeoff (1-Key)
        self.is_fast_drop = False      # Landing (1-Key)
        self.is_emergency_stop = False # Motor Cutoff
        self.is_gyro_correction = False# Calibrate
        self.is_circle_turn_end = False# 360 Flip
        self.is_no_head_mode = False   # Headless
        self.is_fixed_height = True    # Altitude Hold
        self.is_gesture_mode = False
        
        # Trims
        self.roll_trim = 0
        self.pitch_trim = 0
        self.yaw_trim = 0
        self.gear = 2  # 1: 30%, 2: 60%, 3: 100%
        
        # Telemetry & Stats
        self.packets_sent = 0
        self.packets_received = 0
        self.last_drone_telemetry_time = 0
        self.simulated_mode = False
        self.fps = 0
        self.camera_id = 1
        
        # Socket
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setblocking(False)

drone_state = DroneState()
connected_websockets: Set[WebSocket] = set()
latest_video_frame_bytes = None
frame_lock = threading.Lock()

# ----------------- UDP Control Transmitter ----------------- #

def build_control_packet() -> bytes:
    r = max(1, min(255, int(drone_state.roll)))
    p = max(1, min(255, int(drone_state.pitch)))
    t = max(0, min(255, int(drone_state.throttle)))
    y = max(1, min(255, int(drone_state.yaw)))

    if drone_state.device_type != 10:  # GL Protocol (Type 2 - 21 Bytes)
        flags1 = 0
        if drone_state.is_fast_fly or drone_state.is_fast_drop:
            flags1 |= 0x01
        if drone_state.is_emergency_stop:
            flags1 |= 0x02
        if drone_state.is_gyro_correction:
            flags1 |= 0x04
        if drone_state.is_circle_turn_end:
            flags1 |= 0x08
        if drone_state.is_gesture_mode:
            flags1 |= 0x40

        flags2 = 0
        if drone_state.is_no_head_mode:
            flags2 |= 0x01
        if drone_state.is_fixed_height:
            flags2 |= 0x02

        cs = (flags1 ^ (((p ^ r) ^ t) ^ y)) ^ (flags2 & 0xFF)

        inner = bytearray(20)
        inner[0] = 0x66  # Header magic
        inner[1] = 0x14  # Length (20)
        inner[2] = r     # Roll
        inner[3] = p     # Pitch
        inner[4] = t     # Throttle
        inner[5] = y     # Yaw
        inner[6] = flags1
        inner[7] = flags2
        # bytes 8..17 are 0x00 reserved
        inner[18] = cs & 0xFF
        inner[19] = 0x99 # Tail byte

        packet = bytes([0x03]) + bytes(inner)
        return packet
    else:  # Legacy Protocol (Type 10 - 9 Bytes)
        flags = 0
        if drone_state.is_fast_fly: flags += 1
        if drone_state.is_fast_drop: flags += 2
        if drone_state.is_emergency_stop: flags += 4
        if drone_state.is_circle_turn_end: flags += 8
        if drone_state.is_no_head_mode: flags += 16
        if drone_state.is_gyro_correction: flags += 128

        cs = (((r ^ p) ^ t) ^ y) ^ (flags & 0xFF)
        inner = bytes([0x66, r, p, t, y, flags & 0xFF, cs & 0xFF, 0x99])
        return bytes([0x03]) + inner

def send_drone_udp(data: bytes):
    try:
        drone_state.udp_sock.sendto(data, (drone_state.drone_ip, drone_state.udp_port))
        drone_state.packets_sent += 1
    except Exception as e:
        pass

def udp_worker_loop():
    """Background thread sending 25Hz flight packets and 1Hz heartbeat."""
    last_hb_time = 0
    while True:
        now = time.time()
        # 1 Hz Heartbeat
        if now - last_hb_time >= 1.0:
            send_drone_udp(bytes([0x01, 0x01]))
            last_hb_time = now

        # 25 Hz Control Packet (40 ms)
        pkt = build_control_packet()
        send_drone_udp(pkt)

        # Non-blocking receive for drone telemetry
        try:
            while True:
                data, addr = drone_state.udp_sock.recvfrom(512)
                if data:
                    drone_state.packets_received += 1
                    drone_state.last_drone_telemetry_time = time.time()
                    drone_state.connected_to_drone = True
                    dev_id = data[0]
                    # Update device type
                    is_gl = (90 <= dev_id <= 101) or dev_id in [103, 82, 85, 88]
                    drone_state.device_type = 2 if is_gl else 10

                    # Shutter feedback ACK
                    if len(data) > 4:
                        if data[2] == 0x4D:  # Photo trigger 'M'
                            send_drone_udp(bytes([0x09, 0x01]))
                        elif data[2] == 0x58:  # Video trigger 'X'
                            send_drone_udp(bytes([0x09, 0x02]))
        except (BlockingIOError, socket.error):
            pass

        # Check if drone telemetry timed out
        if now - drone_state.last_drone_telemetry_time > 3.0:
            drone_state.connected_to_drone = False

        time.sleep(0.04)

# ----------------- RTSP & Synthetic Video Streamer ----------------- #

def generate_simulated_frame(tick: float) -> bytes:
    """Generates a high-quality simulated FPV camera feed with attitude horizon and HUD telemetry."""
    width, height = 640, 480
    img = Image.new("RGB", (width, height), color=(15, 23, 42))
    draw = ImageDraw.Draw(img)

    # Dynamic horizon lines based on roll and pitch
    pitch_offset = (drone_state.pitch - 128) * 1.5
    roll_angle = (drone_state.roll - 128) * 0.35
    
    # Ground color & sky color gradient
    sky_y = int(height / 2 + pitch_offset)
    draw.rectangle([(0, 0), (width, max(0, min(height, sky_y)))], fill=(12, 35, 64))
    draw.rectangle([(0, max(0, min(height, sky_y))), (width, height)], fill=(18, 48, 38))

    # Grid horizon line
    cx, cy = width // 2, height // 2 + pitch_offset
    rad = math.radians(roll_angle)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    
    x1 = cx - 250 * cos_a
    y1 = cy - 250 * sin_a
    x2 = cx + 250 * cos_a
    y2 = cy + 250 * sin_a
    draw.line([(x1, y1), (x2, y2)], fill=(0, 255, 204), width=2)
    
    # Pitch ladder steps
    for step in [-40, -20, 20, 40]:
        step_cy = cy + step
        sx1 = cx - 40 * cos_a - step * sin_a
        sy1 = cy + step * cos_a - 40 * sin_a
        sx2 = cx + 40 * cos_a - step * sin_a
        sy2 = cy + step * cos_a + 40 * sin_a
        draw.line([(sx1, sy1), (sx2, sy2)], fill=(0, 220, 180, 150), width=1)

    # Crosshair
    draw.line([(cx - 15, cy), (cx - 5, cy)], fill=(255, 204, 0), width=2)
    draw.line([(cx + 5, cy), (cx + 15, cy)], fill=(255, 204, 0), width=2)
    draw.line([(cx, cy - 15), (cx, cy - 5)], fill=(255, 204, 0), width=2)
    draw.line([(cx, cy + 5), (cx, cy + 15)], fill=(255, 204, 0), width=2)

    # Simulated motion particle grid
    speed_factor = (drone_state.throttle - 128) * 0.1
    for i in range(12):
        px = int((i * 65 + tick * 40 * (drone_state.yaw - 128) * 0.05) % width)
        py = int((height / 2 + 30 + (i * 25 + tick * 60) % (height / 2 - 30)))
        draw.ellipse([(px - 2, py - 2), (px + 2, py + 2)], fill=(0, 255, 180))

    # Watermark text
    status_text = "LIVE RTSP: 192.168.1.1:7070" if drone_state.connected_to_drone else "NO DRONE LINK - SIMULATED HUD CAMERA"
    draw.text((20, 20), status_text, fill=(0, 255, 204))
    draw.text((20, 40), f"R:{drone_state.roll} P:{drone_state.pitch} T:{drone_state.throttle} Y:{drone_state.yaw} | FPS: {drone_state.fps}", fill=(200, 220, 240))

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=75)
    return buf.getvalue()


def video_stream_worker():
    """Background worker ingesting low-latency RTSP frames via PyAV or generating synthetic feed."""
    global latest_video_frame_bytes
    frame_count = 0
    fps_timer = time.time()
    tick = 0.0

    while True:
        container = None
        try:
            # Low-latency options for RTSP
            options = {
                'rtsp_transport': 'udp',
                'fflags': 'nobuffer',
                'flags': 'low_delay',
                'analyzeduration': '100000',
                'probesize': '100000',
                'max_delay': '50000'
            }
            container = av.open(drone_state.rtsp_url, options=options, timeout=2.0)
            stream = container.streams.video[0]
            stream.codec_context.thread_type = "AUTO"

            for frame in container.decode(stream):
                # Convert frame to JPEG
                pil_img = frame.to_image()
                buf = io.BytesIO()
                pil_img.save(buf, format="JPEG", quality=70)
                frame_bytes = buf.getvalue()

                with frame_lock:
                    latest_video_frame_bytes = frame_bytes

                drone_state.connected_to_drone = True
                frame_count += 1
                if time.time() - fps_timer >= 1.0:
                    drone_state.fps = frame_count
                    frame_count = 0
                    fps_timer = time.time()

        except Exception as e:
            # Fallback to simulated HUD generator
            drone_state.fps = 30
            tick += 0.033
            frame_bytes = generate_simulated_frame(tick)
            with frame_lock:
                latest_video_frame_bytes = frame_bytes
            time.sleep(0.033)
        finally:
            if container:
                try:
                    container.close()
                except:
                    pass

# ----------------- WebSocket Video & Telemetry Bridge ----------------- #

@app.websocket("/ws/telemetry")
async def websocket_telemetry_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_websockets.add(websocket)
    try:
        while True:
            # Receive commands from client
            msg = await websocket.receive_json()
            action = msg.get("action")
            
            if action == "stick":
                drone_state.roll = msg.get("roll", 128)
                drone_state.pitch = msg.get("pitch", 128)
                drone_state.throttle = msg.get("throttle", 128)
                drone_state.yaw = msg.get("yaw", 128)
            elif action == "takeoff":
                drone_state.is_fast_fly = True
                asyncio.create_task(_reset_flag_delayed("fast_fly", 1.0))
            elif action == "land":
                drone_state.is_fast_drop = True
                asyncio.create_task(_reset_flag_delayed("fast_drop", 1.0))
            elif action == "emergency_stop":
                drone_state.is_emergency_stop = True
                asyncio.create_task(_reset_flag_delayed("emergency_stop", 1.0))
            elif action == "calibrate_gyro":
                drone_state.is_gyro_correction = True
                asyncio.create_task(_reset_flag_delayed("gyro", 2.0))
            elif action == "flip_360":
                direction = msg.get("direction", "forward")
                if direction == "left": drone_state.roll = 1
                elif direction == "right": drone_state.roll = 255
                elif direction == "forward": drone_state.pitch = 255
                elif direction == "backward": drone_state.pitch = 1
                else: drone_state.pitch = 255
                drone_state.is_circle_turn_end = True
                send_drone_udp(bytes([0x07, 0x01]))
                send_drone_udp(bytes([0x08, 0x01]))
                asyncio.create_task(_reset_flip_delayed(0.8))
            elif action == "toggle_headless":
                drone_state.is_no_head_mode = not drone_state.is_no_head_mode
            elif action == "toggle_altitude_hold":
                drone_state.is_fixed_height = not drone_state.is_fixed_height
            elif action == "set_gear":
                drone_state.gear = msg.get("gear", 2)
            elif action == "switch_camera":
                cam_id = msg.get("camera_id", 1)
                drone_state.camera_id = cam_id
                send_drone_udp(bytes([0x06, cam_id]))
                send_drone_udp(bytes([0x06, cam_id]))
                send_drone_udp(bytes([0x06, cam_id]))
            elif action == "set_trims":
                drone_state.roll_trim = msg.get("roll_trim", 0)
                drone_state.pitch_trim = msg.get("pitch_trim", 0)
                drone_state.yaw_trim = msg.get("yaw_trim", 0)
            elif action == "disarm":
                send_drone_udp(bytes([0x08, 0x01]))

    except WebSocketDisconnect:
        connected_websockets.remove(websocket)
    except Exception:
        if websocket in connected_websockets:
            connected_websockets.remove(websocket)

async def _reset_flag_delayed(flag_name: str, delay: float):
    await asyncio.sleep(delay)
    if flag_name == "fast_fly":
        drone_state.is_fast_fly = False
    elif flag_name == "fast_drop":
        drone_state.is_fast_drop = False
    elif flag_name == "emergency_stop":
        drone_state.is_emergency_stop = False
    elif flag_name == "gyro":
        drone_state.is_gyro_correction = False

async def _reset_flip_delayed(delay: float):
    await asyncio.sleep(delay)
    drone_state.is_circle_turn_end = False
    drone_state.roll = 128
    drone_state.pitch = 128

@app.websocket("/ws/video")
async def websocket_video_endpoint(websocket: WebSocket):
    """Streams live JPEG binary frames directly to HTML5 Canvas."""
    await websocket.accept()
    last_sent_frame = None
    try:
        while True:
            with frame_lock:
                current_frame = latest_video_frame_bytes

            if current_frame is not None and current_frame is not last_sent_frame:
                await websocket.send_bytes(current_frame)
                last_sent_frame = current_frame

            await asyncio.sleep(0.02)  # ~50 FPS max
    except (WebSocketDisconnect, Exception):
        pass

# Telemetry broadcast task (10 Hz to UI)
async def telemetry_broadcast_loop():
    while True:
        if connected_websockets:
            telemetry_data = {
                "type": "telemetry",
                "connected": drone_state.connected_to_drone,
                "device_type": "GL (21-Byte)" if drone_state.device_type == 2 else "Legacy (9-Byte)",
                "roll": drone_state.roll,
                "pitch": drone_state.pitch,
                "throttle": drone_state.throttle,
                "yaw": drone_state.yaw,
                "is_fast_fly": drone_state.is_fast_fly,
                "is_fast_drop": drone_state.is_fast_drop,
                "is_emergency_stop": drone_state.is_emergency_stop,
                "is_gyro_correction": drone_state.is_gyro_correction,
                "is_no_head_mode": drone_state.is_no_head_mode,
                "is_fixed_height": drone_state.is_fixed_height,
                "is_circle_turn_end": drone_state.is_circle_turn_end,
                "gear": drone_state.gear,
                "camera_id": drone_state.camera_id,
                "roll_trim": drone_state.roll_trim,
                "pitch_trim": drone_state.pitch_trim,
                "yaw_trim": drone_state.yaw_trim,
                "fps": drone_state.fps,
                "packets_sent": drone_state.packets_sent,
                "packets_received": drone_state.packets_received
            }
            # Broadcast to all connected clients
            dead_sockets = []
            for ws in list(connected_websockets):
                try:
                    await ws.send_json(telemetry_data)
                except Exception:
                    dead_sockets.append(ws)
            for ws in dead_sockets:
                if ws in connected_websockets:
                    connected_websockets.remove(ws)

        await asyncio.sleep(0.1)

# ----------------- App Startup & Static Files ----------------- #

@app.on_event("startup")
async def on_startup():
    # Start UDP control loop thread
    t_udp = threading.Thread(target=udp_worker_loop, daemon=True)
    t_udp.start()
    
    # Start Video Stream worker thread
    t_video = threading.Thread(target=video_stream_worker, daemon=True)
    t_video.start()
    
    # Start telemetry broadcaster
    asyncio.create_task(telemetry_broadcast_loop())

@app.get("/api/status")
async def get_status():
    return JSONResponse({
        "drone_ip": drone_state.drone_ip,
        "udp_port": drone_state.udp_port,
        "rtsp_url": drone_state.rtsp_url,
        "connected": drone_state.connected_to_drone,
        "device_type": drone_state.device_type,
        "packets_sent": drone_state.packets_sent,
        "packets_received": drone_state.packets_received
    })

# Mount static web directory
public_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "public")
os.makedirs(public_dir, exist_ok=True)
app.mount("/", StaticFiles(directory=public_dir, html=True), name="static")

if __name__ == "__main__":
    print("=" * 65)
    print("🛸 RC UFO DRONE COCKPIT & GROUND CONTROL STATION")
    print("=" * 65)
    print(f"[*] Target Drone IP: {DRONE_IP}:{UDP_CONTROL_PORT}")
    print(f"[*] RTSP Live Feed : {RTSP_URL}")
    print(f"[*] Web Cockpit HUD : http://localhost:{WEB_PORT}")
    print("=" * 65)
    uvicorn.run(app, host="0.0.0.0", port=WEB_PORT, log_level="warning")
