# RC UFO Drone Communication & Real-Time Control Protocol Specification

This document provides a comprehensive, reverse-engineered specification of the communication architecture, UDP telemetry/control protocol, video streaming pipeline, and device management protocols used by the **RC UFO** (`com.cooingdv.rcufo`) Android application.

Any custom application (built in Python, Kotlin/Java, Swift, Flutter, React Native, Node.js, or C++) can use this specification to connect to the drone, receive live video, send real-time joystick flight commands, trigger stunts, and manage on-board camera recordings.

---

## Table of Contents
1. [Network Architecture & Endpoints](#1-network-architecture--endpoints)
2. [Connection Lifecycle & Handshake](#2-connection-lifecycle--handshake)
3. [UDP Control Protocol (Port 7099)](#3-udp-control-protocol-port-7099)
   - [Protocol Overview & Wrapper](#protocol-overview--wrapper)
   - [Type 2: GL Protocol (21-Byte Packet)](#type-2-gl-protocol-21-byte-packet)
   - [Type 10: Legacy Protocol (9-Byte Packet)](#type-10-legacy-protocol-9-byte-packet)
   - [Checksum Calculation](#checksum-calculation)
4. [Flight Control Channels & Value Mapping](#4-flight-control-channels--value-mapping)
   - [Stick Ranges & Neutral Centers](#stick-ranges--neutral-centers)
   - [Speed Rates & Dynamic Scaling](#speed-rates--dynamic-scaling)
   - [Trim Adjustment Formula](#trim-adjustment-formula)
5. [Special Commands & Maneuvers](#5-special-commands--maneuvers)
   - [Heartbeat Keepalive](#heartbeat-keepalive)
   - [One-Key Takeoff & Landing](#one-key-takeoff--landing)
   - [Emergency Stop](#emergency-stop)
   - [Gyroscope Calibration](#gyroscope-calibration)
   - [360° Stunt Flip](#360-stunt-flip)
   - [Dual-Camera Switching](#dual-camera-switching)
   - [Wi-Fi Password Configuration](#wi-fi-password-configuration)
   - [Photo / Video Hardware Shutter Triggers & ACK](#photo--video-hardware-shutter-triggers--ack)
6. [Telemetry & Incoming Feedback Parsing](#6-telemetry--incoming-feedback-parsing)
   - [Device Type & Resolution Identification](#device-type--resolution-identification)
   - [Wi-Fi Password Decoding](#wi-fi-password-decoding)
7. [Live Video Stream & Camera Pipeline](#7-live-video-stream--camera-pipeline)
   - [RTSP Video Stream (Port 7070)](#rtsp-video-stream-port-7070)
   - [HTTP Media Endpoints (Port 80)](#http-media-endpoints-port-80)
   - [SD Card Access via FTP (Port 21)](#sd-card-access-via-ftp-port-21)
8. [Complete Working Implementation Examples](#8-complete-working-implementation-examples)
   - [Python Controller & Video Player](#python-controller--video-player)
   - [Kotlin / Android Controller Core](#kotlin--android-controller-core)
   - [Node.js UDP Controller](#nodejs-udp-controller)
9. [Step-by-Step Implementation Guide for a New App](#9-step-by-step-implementation-guide-for-a-new-app)

---

## 1. Network Architecture & Endpoints

When powered on, the drone initializes a Wi-Fi Access Point (AP). The controlling device must connect to this network.

| Property | Value | Description |
| :--- | :--- | :--- |
| **Drone IP Address** | `192.168.1.1` | Static gateway IP assigned by drone AP |
| **Drone Subnet** | `255.255.255.0` | Client receives `192.168.1.X` via DHCP |
| **SSID Patterns** | `KY-WiFi-xxxx`, `WiFi-xxxx`, `4K-WiFi-xxxx`, `RC-UFO-xxxx` | Open network (or WPA-PSK with 8-digit password) |
| **UDP Control Port** | `7099` | Bidirectional real-time flight control & telemetry |
| **RTSP Stream Port** | `7070` | Live MJPEG/H.264 camera stream |
| **RTSP Live URL** | `rtsp://192.168.1.1:7070/webcam` | Primary camera feed |
| **HTTP Web Server** | `http://192.168.1.1:80` | Thumbnails and full photo/video downloads |
| **FTP Server** | `ftp://192.168.1.1:21` | SD Card file system access (`user: ftp`, `pass: ftp`, root: `/0/`) |
| **Secondary TCP Port**| `5000` | Optional auxiliary TCP keepalive connection |

```
                       +-------------------------------------------------+
                       |             Drone (192.168.1.1)                 |
                       +-------------------------------------------------+
                         ^             ^             ^             ^
                         |             |             |             |
                UDP 7099 |   RTSP 7070 |     HTTP 80 |      FTP 21 |
           (Control/Data)|(Live Stream)| (Media/Thumb| (SD Storage |
                         |             |  Downloads) |   Transfer) |
                         v             v             v             v
                       +-------------------------------------------------+
                       |             Custom Controller App               |
                       +-------------------------------------------------+
```

---

## 2. Connection Lifecycle & Handshake

To establish and maintain a reliable link with the drone:

1. **Wi-Fi Association**: Connect client Wi-Fi to the drone AP (`192.168.1.1`).
2. **Socket Initialization**:
   - Open a UDP Datagram Socket bound to an ephemeral local port.
   - Set remote target endpoint to `192.168.1.1:7099`.
3. **Heartbeat Loop**:
   - Start a 1000 ms recurring timer sending the 2-byte heartbeat packet: `[0x01, 0x01]`.
4. **Device Type Negotiation**:
   - Listen for incoming UDP telemetry packets on port `7099`.
   - Read byte 0 (`b2` / Resolution Code):
     - If `b2` matches GL profiles (`(b2 >= 90 && b2 <= 101) || b2 == 103 || b2 == 82 || b2 == 85 || b2 == 88`), set `DeviceType = 2` (GL Protocol).
     - Otherwise, set `DeviceType = 10` (Legacy Protocol).
5. **Flight Control Loop**:
   - Start a recurring timer at **20 Hz to 25 Hz (every 40 ms – 50 ms)**.
   - Periodically transmit 21-byte (GL) or 9-byte (Legacy) flight control packets.
6. **Video Stream Initialization**:
   - Open the RTSP URL `rtsp://192.168.1.1:7070/webcam` using FFmpeg, OpenCV, GStreamer, or IJKPlayer.

---

## 3. UDP Control Protocol (Port 7099)

### Protocol Overview & Wrapper
Every command sent to port 7099 is prefixed by a single-byte **Command ID**. The payload format depends on whether the drone runs the **GL Protocol (`DeviceType 2`)** or the **Legacy Protocol (`DeviceType 10`)**.

| Command ID | Hex | Name | Payload Description | Length |
| :--- | :--- | :--- | :--- | :--- |
| `1` | `0x01` | Heartbeat | `[0x01]` (Total packet: `0x01 0x01`) | 2 bytes |
| `3` | `0x03` | Fly Control | 20-byte GL packet or 8-byte Legacy packet | 21 or 9 bytes |
| `6` | `0x06` | Switch Camera | `[0x01]` (Front) or `[0x02]` (Bottom) | 2 bytes |
| `8` | `0x08` | Stop Flight | `[0x01]` (Disarm / disable flight controls) | 2 bytes |
| `9` | `0x09` | Media ACK | `[0x01]` (Photo ACK) or `[0x02]` (Video ACK) | 2 bytes |
| `10` | `0x0A` | Set Wi-Fi Pass | `[d0, d1, d2, d3, d4, d5, d6, d7]` (8 ASCII digits) | 9 bytes |

---

### Type 2: GL Protocol (21-Byte Packet)
Used by newer HD/4K/8K optical-flow models.

#### Packet Layout
```
Offset   0      1      2      3      4      5      6      7      8    ...   17     18     19     20
       +------+------+------+------+------+------+------+------+------+-----+------+------+------+------+
       | 0x03 | 0x66 | 0x14 | Roll |Pitch | Throt| Yaw  |Flag1 |Flag2 | 0x00 ...0x00| Check| 0x99 | (21B)
       +------+------+------+------+------+------+------+------+------+-----+------+------+------+------+
       |Cmd ID|Header|Length|      Axis Control Values   |     Flags    |  Reserved   |  CS  | Tail |
```

#### Field Details

| Offset | Byte Name | Value Range | Description |
| :--- | :--- | :--- | :--- |
| **0** | `Command ID` | `0x03` (3) | Fixed outer routing ID for flight control |
| **1** | `Header` | `0x66` (102) | Protocol synchronization header |
| **2** | `Payload Len`| `0x14` (20) | Length of inner payload |
| **3** | `Roll (Aileron)` | `1` – `255` | Left (1) to Right (255). Neutral center = `128` |
| **4** | `Pitch (Elevator)`| `1` – `255` | Backward (1) to Forward (255). Neutral center = `128` |
| **5** | `Throttle` | `0` – `255` | Down/Idle (0) to Max Up (255). Neutral hover = `128` |
| **6** | `Yaw (Rudder)` | `1` – `255` | Rotate Left (1) to Rotate Right (255). Center = `128` |
| **7** | `Flags 1` | Bitfield | Action triggers (see below) |
| **8** | `Flags 2` | Bitfield | Flight modes (see below) |
| **9–18** | `Reserved` | `0x00` | 10 zero padding bytes (`[0x00]*10`) |
| **19** | `Checksum` | `0` – `255` | XOR Checksum (see formula) |
| **20** | `End Byte` | `0x99` (153) | Protocol packet termination |

#### Flags 1 Bitfield (Byte 7)
* `Bit 0` (`0x01`): **Take-off / Land** (`isFastFly || isFastDrop`)
* `Bit 1` (`0x02`): **Emergency Stop** (`isEmergencyStop`)
* `Bit 2` (`0x04`): **Gyroscope Calibration** (`isGyroCorrection`)
* `Bit 3` (`0x08`): **360° Flip Execution** (`isCircleTurnEnd`)
* `Bit 6` (`0x40`): **Gesture Control Active** (`isGestureMode`)

#### Flags 2 Bitfield (Byte 8)
* `Bit 0` (`0x01`): **Headless Mode** (`isNoHeadMode`)
* `Bit 1` (`0x02`): **Altitude Hold / Fixed Height** (`isFixedHeightMode`)

---

### Type 10: Legacy Protocol (9-Byte Packet)
Used by standard/classic mini-drones.

#### Packet Layout
```
Offset   0      1      2      3      4      5      6      7      8
       +------+------+------+------+------+------+------+------+------+
       | 0x03 | 0x66 | Roll |Pitch | Throt| Yaw  |Flags | Check| 0x99 | (9B)
       +------+------+------+------+------+------+------+------+------+
       |Cmd ID|Header|      Axis Control Values   | Mode |  CS  | Tail |
```

#### Field Details

| Offset | Byte Name | Value Range | Description |
| :--- | :--- | :--- | :--- |
| **0** | `Command ID` | `0x03` | Outer routing ID |
| **1** | `Header` | `0x66` (102) | Header start byte |
| **2** | `Roll` | `1` – `255` | Left/Right tilt. Center = `128` |
| **3** | `Pitch` | `1` – `255` | Forward/Backward tilt. Center = `128` |
| **4** | `Throttle` | `0` – `255` | Up/Down. Center = `128` |
| **5** | `Yaw` | `1` – `255` | Turn Left/Right. Center = `128` |
| **6** | `Flags` | Integer (0–255)| Additive bitmask (see below) |
| **7** | `Checksum` | `0` – `255` | XOR Checksum |
| **8** | `End Byte` | `0x99` (153) | Termination byte |

#### Flags Additive Values (Byte 6)
* `+1`: Take-off (`isFastFly`)
* `+2`: Landing (`isFastDrop`)
* `+4`: Emergency Stop (`isEmergencyStop`)
* `+8`: 360° Flip End (`isCircleTurnEnd`)
* `+16`: Headless Mode (`isNoHeadMode`)
* `+32`: Return to Home (`isFastReturn`)
* `+128`: Gyroscope Calibration (`isGyroCorrection`)

---

### Checksum Calculation

#### GL Protocol (Type 2) Checksum
```java
int checksum = (flags1 ^ (((pitch ^ roll) ^ throttle) ^ yaw)) ^ (flags2 & 0xFF);
```

#### Legacy Protocol (Type 10) Checksum
```java
int checksum = (((roll ^ pitch) ^ throttle) ^ yaw) ^ (flags & 0xFF);
```

---

## 4. Flight Control Channels & Value Mapping

### Stick Ranges & Neutral Centers

Each control axis operates with an 8-bit unsigned integer range:
* **Minimum Value**: `1` (Full Negative)
* **Neutral Center**: `128` (Stick Centered / Idle)
* **Maximum Value**: `255` (Full Positive)

> **Throttle Special Case**: When the drone is on the ground / disarmed, `Throttle = 0`. If `controlAccelerator == 1`, it is automatically clamped to `0`.

```
          Pitch Forward (255)                  Throttle Up (255)
                   ^                                   ^
                   |                                   |
 Roll Left (1) <---+---> Roll Right (255)  Yaw Left (1)<---+---> Yaw Right (255)
                   |                                   |
                   v                                   v
          Pitch Backward (1)                  Throttle Down (0/1)
           [Right Stick]                         [Left Stick]
```

---

### Speed Rates & Dynamic Scaling

The drone firmware supports 3 sensitivity levels (gear/speed modes: 30%, 60%, 100%). The app scales the displacement from center using the `halfLen` multiplier:

| Speed Rate | Gear Level | `currPower` ID | `halfLen` Multiplier | Deflection Range | Stick Range Sent |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **30% (Slow)** | 1 | `4` | `40` | `±40` | `88` – `168` |
| **60% (Medium)**| 2 | `9` | `60` | `±60` | `68` – `188` |
| **100% (Fast)** | 3 | `14`| `127` | `±127` | `1` – `255` |

#### Axis Mapping Formula
Given joystick normalized inputs $x \in [-1.0, 1.0]$ and $y \in [-1.0, 1.0]$:

```python
# Right Stick (Roll & Pitch)
roll = int(128 + (x_right * halfLen) + (roll_trim * trim_mult))
pitch = int(128 + (y_right * halfLen) + (pitch_trim * trim_mult))

# Left Stick (Yaw & Throttle)
yaw = int(128 + (x_left * 127.0) + (yaw_trim * 4))
throttle = int(128 + (y_left * 127.0))

# Clamping
roll = max(1, min(255, roll))
pitch = max(1, min(255, pitch))
yaw = max(1, min(255, yaw))
throttle = max(0, min(255, throttle))
```

---

### Trim Adjustment Formula
Hardware trims apply fine-grained offsets stored in settings:
* **Roll Trim**: Offset multiplied by `2`, `3`, or `4` (depending on speed gear).
* **Pitch Trim**: Offset multiplied by `2`, `3`, or `4`.
* **Yaw Trim**: Offset multiplied by `4`.

---

## 5. Special Commands & Maneuvers

### Heartbeat Keepalive
* **Interval**: Every 1000 ms.
* **Payload**: `[0x01, 0x01]` (UDP port 7099).

---

### One-Key Takeoff & Landing
1. **Takeoff**: Send control packet with `FastFly` bit set (`Flags1 |= 0x01` in GL, `Flags |= 1` in Legacy) continuously for **1000 ms**.
2. **Landing**: Send control packet with `FastDrop` bit set (`Flags1 |= 0x01` in GL, `Flags |= 2` in Legacy) continuously for **1000 ms**.
3. After 1000 ms, clear the flag back to `0`.

---

### Emergency Stop
* Set `EmergencyStop` bit (`Flags1 |= 0x02` in GL, `Flags |= 4` in Legacy) continuously for **1000 ms**.
* The flight controller immediately cuts power to all 4 motors.

---

### Gyroscope Calibration
1. Place drone on a flat, horizontal surface.
2. Send control packet with `GyroCorrection` bit set (`Flags1 |= 0x04` in GL, `Flags |= 128` in Legacy) for **2000 ms**.
3. Keep sticks at neutral center (`Roll=128, Pitch=128, Throttle=128, Yaw=128`).
4. Drone LEDs will flash rapidly during calibration and turn solid when complete.

---

### 360° Stunt Flip
1. Trigger Flip Mode by flicking the right stick in the desired flip direction when the stick displacement $> 70\%$:
   - **Flip Left**: Set `Roll = 1`
   - **Flip Right**: Set `Roll = 255`
   - **Flip Forward**: Set `Pitch = 255`
   - **Flip Backward**: Set `Pitch = 1`
2. Set the `CircleTurnEnd` bit (`Flags1 |= 0x08` in GL, `Flags |= 8` in Legacy).
3. Send this state for **600 ms**.
4. After 600 ms, clear the flip bit and reset `Roll = 128, Pitch = 128`.

---

### Dual-Camera Switching
* **Front Camera**: Send UDP `[0x06, 0x01]` to `192.168.1.1:7099`.
* **Bottom Camera**: Send UDP `[0x06, 0x02]` to `192.168.1.1:7099`.
* Reconnect or restart the RTSP stream client after switching.

---

### Wi-Fi Password Configuration
To update the drone's AP password:
* Send UDP packet: `[0x0A, d0, d1, d2, d3, d4, d5, d6, d7]`
* Where `d0..d7` are 8 numeric ASCII byte digits (e.g., `'1', '2', '3', '4', '5', '6', '7', '8'` -> `0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38`).

---

### Photo / Video Hardware Shutter Triggers & ACK
When the user presses a physical shutter button on a remote transmitter paired with the drone, the drone transmits a UDP telemetry packet on port 7099:
* **Photo Trigger Received**: Packet `byte[2] == 0x4D` (77, ASCII `'M'`) -> App replies with UDP `[0x09, 0x01]`.
* **Video Trigger Received**: Packet `byte[2] == 0x58` (88, ASCII `'X'`) -> App replies with UDP `[0x09, 0x02]`.

---

## 6. Telemetry & Incoming Feedback Parsing

The drone broadcasts periodic status datagrams to the client's UDP port:

```
Telemetry Packet Layout:
+--------+--------+--------+--------+--------+--------+--------+--------+--------+-----+
| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 | Byte 8 | ... |
+--------+--------+--------+--------+--------+--------+--------+--------+--------+-----+
| Dev ID | CamRes | Status | Photo# | Video# | Data A | Data B | Pass 0 | Pass 1 | ... |
+--------+--------+--------+--------+--------+--------+--------+--------+--------+-----+
```

### Device Type & Resolution Identification
* `Byte 0` (`Dev ID`):
  - Check `isGL(byte0)`:
    - If `(byte0 >= 90 && byte0 <= 101) || byte0 == 103 || byte0 == 82 || byte0 == 85 || byte0 == 88`, device is **Type 2 (GL Protocol)**.
    - Otherwise, device is **Type 10 (Legacy Protocol)**.
  - Video stream resolution capabilities:
    - `480p`: `5, 26, 63, 68, 85`
    - `720p`: `9, 21, 27, 41, 64, 69, 88, 90`
    - `1080p`: `12, 23, 29, 43, 65, 70, 91`
    - `4K`: `30, 44, 71, 92, 94`
    - `8K`: `19, 20, 24, 31, 45, 51, 66, 67, 72, 80, 81, 82, 83, 84, 86, 87, 93, 99, 100, 101, 103, 105`

---

### Wi-Fi Password Decoding
If the drone AP is password-protected, the drone reports the current password in the telemetry payload as hex nibbles:
* **For GL Devices (`Type 2`, length $\ge$ 19)**:
  Password is read from `byte[11]` through `byte[18]` (taking low-order nibbles/second character of hex string).
* **For Legacy Devices (`Type 10`, length $\ge$ 15)**:
  Password is read from `byte[7]` through `byte[14]`.

---

## 7. Live Video Stream & Camera Pipeline

### RTSP Video Stream (Port 7070)
* **Stream URL**: `rtsp://192.168.1.1:7070/webcam`
* **Transport**: RTSP over UDP / RTP.
* **Payload Encoding**: Motion JPEG (MJPEG) frames (JPEG start delimiter `0xFF 0xD8`, end delimiter `0xFF 0xD9`) or H.264 video.
* **FFmpeg Demuxing Flags**:
  - `-rtsp_transport udp`
  - `-fflags nobuffer`
  - `-flags low_delay`
  - `-strict experimental`

---

### HTTP Media Endpoints (Port 80)
The drone runs an embedded HTTP server for browsing captured media:
* **Photo Thumbnail**: `http://192.168.1.1/PHOTO/T/<filename>.jpg`
* **Full-Resolution Photo**: `http://192.168.1.1/PHOTO/O/<filename>.jpg`
* **Video Thumbnail / Stream**: `http://192.168.1.1/DCIM/<filename>.avi`
* **Recorded Video RTSP Stream**: `rtsp://192.168.1.1:7070/file/DCIM/<filename>.avi`

---

### SD Card Access via FTP (Port 21)
* **Host**: `192.168.1.1`
* **Port**: `21`
* **Username**: `ftp`
* **Password**: `ftp`
* **Root Directory**: `/0/` (contains `PHOTO/` and `DCIM/`)

---

## 8. Complete Working Implementation Examples

### Python Controller & Video Player

A complete, standalone Python controller implementing the full 21-byte GL & 9-byte Legacy protocols, heartbeat loop, joystick controls, and OpenCV live video player:

```python
import socket
import threading
import time
import cv2

class RCUFODroneController:
    DRONE_IP = "192.168.1.1"
    UDP_PORT = 7099
    RTSP_URL = "rtsp://192.168.1.1:7070/webcam"
    
    def __init__(self, device_type=2):
        self.device_type = device_type  # 2 = GL Protocol, 10 = Legacy
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = False
        
        # Flight channels (1..255, 128 = center)
        self.roll = 128
        self.pitch = 128
        self.throttle = 128
        self.yaw = 128
        
        # Flags
        self.is_fast_fly = False       # Takeoff
        self.is_fast_drop = False      # Land
        self.is_emergency_stop = False # Kill switch
        self.is_gyro_correction = False# Calibrate
        self.is_circle_turn_end = False# 360 Flip
        self.is_no_head_mode = False   # Headless
        self.is_fixed_height = True    # Altitude hold
        self.is_gesture_mode = False
        
    def start(self):
        self.running = True
        # Start heartbeat (1 Hz)
        self.hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.hb_thread.start()
        # Start control command loop (25 Hz / 40 ms)
        self.ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.ctrl_thread.start()
        # Start telemetry receiver
        self.recv_thread = threading.Thread(target=self._receive_telemetry_loop, daemon=True)
        self.recv_thread.start()

    def stop(self):
        self.running = False
        # Send stop control command
        self.send_raw(bytes([0x08, 0x01]))
        self.sock.close()

    def send_raw(self, data: bytes):
        try:
            self.sock.sendto(data, (self.DRONE_IP, self.UDP_PORT))
        except Exception as e:
            print(f"[Error] Socket send: {e}")

    def _heartbeat_loop(self):
        while self.running:
            self.send_raw(bytes([0x01, 0x01]))
            time.sleep(1.0)

    def _control_loop(self):
        while self.running:
            packet = self._build_control_packet()
            self.send_raw(packet)
            time.sleep(0.04)  # 25 Hz (40 ms)

    def _build_control_packet(self) -> bytes:
        # Clamp values
        r = max(1, min(255, int(self.roll)))
        p = max(1, min(255, int(self.pitch)))
        t = max(0, min(255, int(self.throttle)))
        y = max(1, min(255, int(self.yaw)))

        if self.device_type != 10:  # GL Protocol (Type 2 - 21 Bytes)
            flags1 = 0
            if self.is_fast_fly or self.is_fast_drop:
                flags1 |= 0x01
            if self.is_emergency_stop:
                flags1 |= 0x02
            if self.is_gyro_correction:
                flags1 |= 0x04
            if self.is_circle_turn_end:
                flags1 |= 0x08
            if self.is_gesture_mode:
                flags1 |= 0x40

            flags2 = 0
            if self.is_no_head_mode:
                flags2 |= 0x01
            if self.is_fixed_height:
                flags2 |= 0x02

            # Checksum calculation
            cs = (flags1 ^ (((p ^ r) ^ t) ^ y)) ^ (flags2 & 0xFF)

            inner_payload = bytearray(20)
            inner_payload[0] = 0x66  # Header
            inner_payload[1] = 0x14  # Length (20)
            inner_payload[2] = r     # Roll
            inner_payload[3] = p     # Pitch
            inner_payload[4] = t     # Throttle
            inner_payload[5] = y     # Yaw
            inner_payload[6] = flags1
            inner_payload[7] = flags2
            # bytes 8..17 are 0x00
            inner_payload[18] = cs & 0xFF
            inner_payload[19] = 0x99 # Tail byte

            # Outer wrapper byte 0x03
            return bytes([0x03]) + bytes(inner_payload)

        else:  # Legacy Protocol (Type 10 - 9 Bytes)
            flags = 0
            if self.is_fast_fly: flags += 1
            if self.is_fast_drop: flags += 2
            if self.is_emergency_stop: flags += 4
            if self.is_circle_turn_end: flags += 8
            if self.is_no_head_mode: flags += 16
            if self.is_gyro_correction: flags += 128

            cs = (((r ^ p) ^ t) ^ y) ^ (flags & 0xFF)

            inner_payload = bytes([
                0x66, r, p, t, y, flags & 0xFF, cs & 0xFF, 0x99
            ])
            return bytes([0x03]) + inner_payload

    def _receive_telemetry_loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                if len(data) >= 1:
                    dev_id = data[0]
                    # Automatically adapt GL vs Legacy device type
                    is_gl = (90 <= dev_id <= 101) or dev_id in [103, 82, 85, 88]
                    self.device_type = 2 if is_gl else 10
                    
                # Handle hardware shutter button triggers
                if len(data) > 4:
                    if data[2] == 0x4D:  # Photo trigger 'M'
                        self.send_raw(bytes([0x09, 0x01]))
                    elif data[2] == 0x58:  # Video trigger 'X'
                        self.send_raw(bytes([0x09, 0x02]))
            except Exception:
                pass

    # High-level control helpers
    def takeoff(self):
        def _task():
            self.is_fast_fly = True
            time.sleep(1.0)
            self.is_fast_fly = False
        threading.Thread(target=_task, daemon=True).start()

    def land(self):
        def _task():
            self.is_fast_drop = True
            time.sleep(1.0)
            self.is_fast_drop = False
        threading.Thread(target=_task, daemon=True).start()

    def emergency_stop(self):
        def _task():
            self.is_emergency_stop = True
            time.sleep(1.0)
            self.is_emergency_stop = False
        threading.Thread(target=_task, daemon=True).start()

    def calibrate_gyro(self):
        def _task():
            self.is_gyro_correction = True
            time.sleep(2.0)
            self.is_gyro_correction = False
        threading.Thread(target=_task, daemon=True).start()

    def flip_360(self, direction="right"):
        def _task():
            if direction == "left":
                self.roll = 1
            elif direction == "right":
                self.roll = 255
            elif direction == "forward":
                self.pitch = 255
            elif direction == "backward":
                self.pitch = 1
            self.is_circle_turn_end = True
            time.sleep(0.6)
            self.is_circle_turn_end = False
            self.roll = 128
            self.pitch = 128
        threading.Thread(target=_task, daemon=True).start()

    def switch_camera(self, camera_id=1):
        # 1 = Front Camera, 2 = Bottom Camera
        self.send_raw(bytes([0x06, camera_id]))


def play_video_stream():
    """Live video playback loop via RTSP."""
    cap = cv2.VideoCapture("rtsp://192.168.1.1:7070/webcam", cv2.CAP_FFMPEG)
    while True:
        ret, frame = cap.read()
        if ret:
            cv2.imshow("RC UFO Drone Stream", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            time.sleep(0.01)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    drone = RCUFODroneController()
    drone.start()
    print("[+] Controller connected and transmitting heartbeat/control packets...")
    # Uncomment to launch video window:
    # play_video_stream()
```

---

### Kotlin / Android Controller Core

```kotlin
package com.example.dronecontroller

import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.Timer
import java.util.TimerTask

class DroneUdpClient {
    private val DRONE_IP = "192.168.1.1"
    private val UDP_PORT = 7099
    
    private var socket: DatagramSocket? = null
    private var address: InetAddress? = null
    private var isRunning = false
    
    private var controlTimer: Timer? = null
    private var heartbeatTimer: Timer? = null
    
    var roll = 128
    var pitch = 128
    var throttle = 128
    var yaw = 128
    
    var isFastFly = false
    var isFastDrop = false
    var isEmergencyStop = false
    var isGyroCorrection = false
    var isCircleTurnEnd = false
    var isNoHeadMode = false
    var isFixedHeightMode = true
    var isGestureMode = false
    var deviceType = 2

    fun connect() {
        socket = DatagramSocket()
        address = InetAddress.getByName(DRONE_IP)
        isRunning = true

        // Heartbeat timer (1000 ms)
        heartbeatTimer = Timer().apply {
            schedule(object : TimerTask() {
                override fun run() {
                    sendRaw(byteArrayOf(1, 1))
                }
            }, 0, 1000)
        }

        // Control command loop (40 ms -> 25 Hz)
        controlTimer = Timer().apply {
            schedule(object : TimerTask() {
                override fun run() {
                    sendFlightPacket()
                }
            }, 0, 40)
        }
    }

    private fun sendFlightPacket() {
        val r = roll.coerceIn(1, 255)
        val p = pitch.coerceIn(1, 255)
        val t = throttle.coerceIn(0, 255)
        val y = yaw.coerceIn(1, 255)

        if (deviceType != 10) { // GL Protocol (Type 2 - 21 Bytes)
            var flags1 = 0
            if (isFastFly || isFastDrop) flags1 += 1
            if (isEmergencyStop) flags1 += 2
            if (isGyroCorrection) flags1 += 4
            if (isCircleTurnEnd) flags1 += 8
            if (isGestureMode) flags1 += 64

            var flags2 = 0
            if (isNoHeadMode) flags2 += 1
            if (isFixedHeightMode) flags2 += 2

            val checksum = (flags1 xor (((p xor r) xor t) xor y)) xor (flags2 and 0xFF)

            val inner = ByteArray(20)
            inner[0] = 0x66.toByte() // 102
            inner[1] = 0x14.toByte() // 20
            inner[2] = r.toByte()
            inner[3] = p.toByte()
            inner[4] = t.toByte()
            inner[5] = y.toByte()
            inner[6] = flags1.toByte()
            inner[7] = flags2.toByte()
            inner[18] = checksum.toByte()
            inner[19] = 0x99.toByte() // -103 signed

            val packet = ByteArray(21)
            packet[0] = 3.toByte() // Outer Command ID
            System.arraycopy(inner, 0, packet, 1, 20)
            sendRaw(packet)
        } else { // Legacy Protocol (Type 10 - 9 Bytes)
            var flags = 0
            if (isFastFly) flags += 1
            if (isFastDrop) flags += 2
            if (isEmergencyStop) flags += 4
            if (isCircleTurnEnd) flags += 8
            if (isNoHeadMode) flags += 16
            if (isGyroCorrection) flags += 128

            val checksum = (((r xor p) xor t) xor y) xor (flags and 0xFF)
            val inner = byteArrayOf(
                0x66.toByte(), r.toByte(), p.toByte(), t.toByte(), y.toByte(),
                flags.toByte(), checksum.toByte(), 0x99.toByte()
            )
            val packet = ByteArray(9)
            packet[0] = 3.toByte()
            System.arraycopy(inner, 0, packet, 1, 8)
            sendRaw(packet)
        }
    }

    fun sendRaw(data: ByteArray) {
        try {
            socket?.send(DatagramPacket(data, data.size, address, UDP_PORT))
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    fun disconnect() {
        isRunning = false
        controlTimer?.cancel()
        heartbeatTimer?.cancel()
        sendRaw(byteArrayOf(8, 1))
        socket?.close()
    }
}
```

---

### Node.js UDP Controller

```javascript
const dgram = require('dgram');
const client = dgram.createSocket('udp4');

const DRONE_IP = '192.168.1.1';
const UDP_PORT = 7099;

let state = {
  roll: 128,
  pitch: 128,
  throttle: 128,
  yaw: 128,
  fastFly: false,
  fastDrop: false,
  emergency: false,
  gyro: false,
  flip: false,
  headless: false,
  fixedHeight: true
};

function buildGLPacket() {
  const flags1 = (state.fastFly || state.fastDrop ? 1 : 0) |
                 (state.emergency ? 2 : 0) |
                 (state.gyro ? 4 : 0) |
                 (state.flip ? 8 : 0);

  const flags2 = (state.headless ? 1 : 0) | (state.fixedHeight ? 2 : 0);

  const cs = (flags1 ^ (((state.pitch ^ state.roll) ^ state.throttle) ^ state.yaw)) ^ (flags2 & 0xFF);

  const buffer = Buffer.alloc(21);
  buffer[0] = 0x03; // Command ID
  buffer[1] = 0x66; // Magic header
  buffer[2] = 0x14; // Length (20)
  buffer[3] = state.roll;
  buffer[4] = state.pitch;
  buffer[5] = state.throttle;
  buffer[6] = state.yaw;
  buffer[7] = flags1;
  buffer[8] = flags2;
  // 9..18 are 0x00
  buffer[19] = cs & 0xFF;
  buffer[20] = 0x99; // Magic tail
  return buffer;
}

// 1 Hz Heartbeat
setInterval(() => {
  client.send(Buffer.from([0x01, 0x01]), UDP_PORT, DRONE_IP);
}, 1000);

// 25 Hz Control Loop (40 ms)
setInterval(() => {
  const pkt = buildGLPacket();
  client.send(pkt, UDP_PORT, DRONE_IP);
}, 40);

console.log('RC UFO Node.js Controller Running...');
```

---

## 9. Step-by-Step Implementation Guide for a New App

Follow these steps to build a fully functional controller app:

### Step 1: Connect to the Drone Wi-Fi Network
* Turn on the drone.
* Connect the mobile/desktop device to the drone's Wi-Fi network (`KY-WiFi-xxxx` or similar).

### Step 2: Establish the UDP Control Link
* Open a UDP socket targeting `192.168.1.1:7099`.
* Start a 1 Hz timer transmitting `[0x01, 0x01]` (Heartbeat).
* Listen for incoming UDP packets. If `byte[0]` matches GL codes (`>= 90`, `82`, `85`, `88`, `103`), set `DeviceType = 2`; otherwise `DeviceType = 10`.

### Step 3: Implement Joystick Axis Mapping
* Left Virtual Stick:
  - Y-axis: Throttle (`0` to `255`, neutral hover = `128`)
  - X-axis: Yaw / Turn Rate (`1` to `255`, neutral = `128`)
* Right Virtual Stick:
  - Y-axis: Pitch / Elevator (`1` to `255`, neutral = `128`)
  - X-axis: Roll / Aileron (`1` to `255`, neutral = `128`)
* Apply speed gear scaling (`30%` -> `±40`, `60%` -> `±60`, `100%` -> `±127`).

### Step 4: Transmit Continuous 25 Hz Flight Packets
* Transmit the 21-byte GL packet (or 9-byte Legacy packet) every **40 ms**.
* Continuously recalculate the XOR checksum byte.

### Step 5: Implement Flight Functions
* **Take-off**: Set `Flags1` Bit 0 for 1000 ms.
* **Land**: Set `Flags1` Bit 0 for 1000 ms.
* **Emergency Cutoff**: Set `Flags1` Bit 1 for 1000 ms.
* **Gyro Calibrate**: Set `Flags1` Bit 2 for 2000 ms on level ground.
* **360° Stunt Flip**: Set target axis to maximum deflection and set `Flags1` Bit 3 for 600 ms.

### Step 6: Render the RTSP Video Stream
* Open `rtsp://192.168.1.1:7070/webcam` using standard RTSP/MJPEG decoders (e.g., VLC, IJKPlayer, OpenCV, FFmpeg).
* Render the frames directly beneath the HUD / on-screen joystick overlays.
