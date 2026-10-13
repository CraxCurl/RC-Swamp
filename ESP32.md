# ESP32 Drone Bridge Controller Firmware & Architecture Guide

This document provides the complete, production-ready **ESP32 Firmware Code**, hardware setup instructions, protocol mappings, and connection architecture to bridge flight control commands from your laptop to the drone.

---

## 1. System Architecture (Drone Hotspot -> ESP32 Wi-Fi | ESP32 -> Laptop Bluetooth)

```
+---------------------------+                      +---------------------------+                      +---------------------------+
|        Laptop / PC        |                      |       ESP32 Bridge        |                      |     Drone Quadcopter      |
|  (GCS Web App / Script)   |                      | (NodeMCU / DevKit V1 WROOM|                      |  (RC UFO / KY / 4K / E88) |
|                           |   Bluetooth SPP      |                           |      Wi-Fi 2.4GHz    |                           |
|   Web Serial (COM11)      | <==================> |  Bluetooth Serial SPP     | <==================> | Drone AP (192.168.1.1/4.1)|
|   NO Drone Wi-Fi Needed   |  Flight Cmds (TX)    |  FreeRTOS Dual-Core       |  UDP Flight Controls | UDP Ports 7099 / 8090     |
|   Live Video Canvas (RX)  |  Live MJPEG FPV (RX) |  Wi-Fi Station Mode       |  HTTP/UDP Camera Stream| Camera Port 8080 / 80   |
+---------------------------+                      +---------------------------+                      +---------------------------+
```

### Key Workflow (Wireless Bluetooth + Wi-Fi Hotspot Topology):
1. **Drone $\rightarrow$ ESP32 over Wi-Fi**: The drone's Wi-Fi hotspot connects directly to the ESP32 in Station Mode (`WiFi.mode(WIFI_STA)`). The ESP32 discovers the drone gateway IP (`192.168.1.1` or `192.168.4.1`) automatically.
2. **ESP32 $\rightarrow$ Laptop over Bluetooth**: The laptop is **NOT** connected to the drone's Wi-Fi. It pairs with `ESP32_Drone` over Bluetooth SPP (`COM11` on Windows Web Serial API).
3. **Flight Control Pipeline (Laptop $\rightarrow$ ESP32 $\rightarrow$ Drone)**:
   - Stick movements and action commands (`T`, `L`, `S`, `G`, `F`) are transmitted from the laptop over Bluetooth Serial to the ESP32.
   - The ESP32 dispatches flight packets at 25 Hz over Wi-Fi UDP to drone ports 7099 / 8090.
4. **Live FPV Video Pipeline (Drone $\rightarrow$ ESP32 $\rightarrow$ Laptop)**:
   - A dedicated FreeRTOS background task running on **Core 0** connects to the drone camera HTTP server (`droneTargetIP:8080/?action=stream` or `/?action=snapshot` or UDP 8080).
   - Incoming JPEG frames are assembled and framed with a magic header (`0xDE, 0xAD, 0xBE, 0xEF` + 4-byte length + JPEG payload).
   - The ESP32 streams the frames across the Bluetooth SPP link to the laptop at ~8 FPS.
   - The GCS Web App decodes the binary frames in real-time and renders them live on `videoCanvas`!

---

## 2. Complete ESP32 Firmware Code (`ESP32_Drone_Bridge.ino`)

The full source code is located in [`ESP32_Drone_Bridge.ino`](file:///d:/SwarmDrone/RC-Swamp/ESP32_Drone_Bridge/ESP32_Drone_Bridge.ino). Flash this code onto your ESP32 board using Arduino IDE (select **ESP32 Dev Module**).

### Core Features Added for FPV Video Streaming:
1. **Dual-Core FreeRTOS Architecture**:
   - **Core 1**: Dedicated real-time flight control loop (`loop()`). Dispatches 25 Hz UDP flight packets, processes stick commands, and monitors failsafe keepalive without ANY interruption or jitter.
   - **Core 0**: Dedicated background video client (`videoRelayTask`). Connects to drone camera port 8080/80 over Wi-Fi, retrieves MJPEG/snapshot JPEG frames, and streams them over Bluetooth Serial.
2. **Binary Serial Demuxing & Framing**:
   - Magic Header: `0xDE, 0xAD, 0xBE, 0xEF` (4 bytes)
   - Length Header: 4-byte big-endian unsigned integer representing frame byte size $L$
   - Payload: $L$ bytes of JPEG data (`0xFF, 0xD8` to `0xFF, 0xD9`)
   - Text logs and telemetry (`[BT]`, `[WIFI]`, `STATUS:...`) remain plain ASCII strings with `\r\n` and are protected by `btMutex` so they never interleave with binary video frames.
3. **Multi-Camera Protocol Discovery**:
   - **HTTP MJPEG Stream**: `GET /?action=stream HTTP/1.1` on port 8080 or port 80.
   - **HTTP Snapshot Fallback**: `GET /?action=snapshot HTTP/1.1` every 125 ms.
   - **UDP Video Stream Listener**: Listens on UDP port 8080 with incoming JPEG marker reassembly.
4. **Bluetooth Bandwidth Optimization**:
   - Automatically rate-limits frame delivery to ~8-9 FPS (110-125 ms interval), using ~40-70 KB/s of bandwidth, fitting comfortably within the 120-180 KB/s capacity of ESP32 Bluetooth SPP (RFCOMM).
```

---

## 3. Supported Command Protocol

### A. Single Character ASCII Commands (via USB Serial Terminal / BT / GCS Web App)
| Command Character | Action | Drone Execution |
| :--- | :--- | :--- |
| `'t'` or `'T'` | **One-Key Takeoff** | Sends burst with command `0x01` / FastFly to climb and lock hover |
| `'l'` or `'L'` | **One-Key Landing** | Sends burst with command `0x02` / FastDrop for smooth descent |
| `'s'` or `'S'` | **Emergency Stop** | Cuts throttle to 0 and sends emergency disarm packet |
| `'g'` or `'G'` | **Gyro Calibration** | Sends gyro leveling pulse to internal drone IMU |
| `'f'` or `'F'` | **360° Flip** | Arms flip bit and deflects forward pitch |
| `'h'` or `'H'` | **Handshake Wakeup** | Sends `[0x01, 0x01]` heartbeat wakeup packets |
| `'p'` or `'P'` | **Ping / Status** | ESP32 replies with Wi-Fi link, RSSI, and Drone online status |
| `SCAN` | **Re-scan Networks** | Triggers fresh Wi-Fi scan and connects to strongest drone AP |
| `SSID:<name>` | **Set Custom SSID** | Connects to specific SSID (e.g. `SSID:KY-WiFi-1234` or `SSID:WIFI-UFO-289424`) |
| `MODE:<gl|legacy|e88|all>` | **Set Protocol** | Switches packet format between GL (21B), Legacy (9B), E88 (8B), or Universal |

---

## 4. How to Flash & Connect

### Step 1: Flash the Firmware
1. Open Arduino IDE.
2. Open [`ESP32_Drone_Bridge.ino`](file:///d:/SwarmDrone/RC-Swamp/ESP32_Drone_Bridge.ino).
3. Select board: **ESP32 Dev Module** (under ESP32 Arduino).
4. Select your ESP32's COM Port.
5. Click **Upload**.

### Step 2: Connect to Laptop
- **Via USB Cable**: Keep the ESP32 connected to your laptop via USB. In the Web GCS, click **"CONNECT ESP32"** and select the USB Serial COM port.
- **Via Bluetooth**: Pair your laptop with **`ESP32_Drone`**, then click **"CONNECT ESP32"** in the Web GCS.

### Step 3: Verify Drone Connection
1. Power on the drone (ensure drone LEDs are blinking or in Wi-Fi broadcast mode).
2. The ESP32's onboard LED will:
   - **Blink rapidly**: Scanning and connecting to the drone's Wi-Fi network.
   - **Stay solid blue ON**: Wi-Fi connection locked and UDP socket active!
3. Open the Serial Monitor at `115200 baud` or use the Web GCS terminal to send `P` (Ping) to verify status.

---

## 5. Onboard Drone Sensors (L3GD20H & BMP180)

The target drone physically carries **only two sensors**:
1. **ST L3GD20H (3-Axis Gyroscope)**:
   - Measures **angular velocity** ($\omega_x, \omega_y, \omega_z$ in °/s) around X, Y, and Z axes.
   - **Does NOT directly measure orientation or position**.
   - Zero-bias calibration is triggered by command `0x80` (`'g'` / `'G'`).
2. **Bosch BMP180 (Barometric Sensor)**:
   - Measures **atmospheric pressure** ($P$ in hPa / Pa) and temperature ($T$).
   - Estimates **altitude relative to ground pressure ($P_0$)** via $h = 44330 \cdot (1 - (P / P_0)^{1 / 5.255})$.
   - **Is NOT a direct height sensor** (height is calculated from barometric pressure changes relative to $P_0$).
   - Ground tare feature zeros relative altitude by setting $P_0$ to current ground pressure.
