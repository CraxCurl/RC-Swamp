# 🛸 RC UFO Drone — Web Ground Control Station & FPV Cockpit

A real-time Web Ground Control Station (GCS), FPV video streaming suite, computer vision interface, and autonomous mission runner for **RC UFO** compatible Wi-Fi quadcopters (using the `com.cooingdv.rcufo` protocol standard).

---

## 📑 Table of Contents

- [Overview](#-overview)
- [Key Features](#-key-features)
- [Hardware & Network Setup](#-hardware--network-setup)
- [Dependencies & Installation](#-dependencies--installation)
  - [Node.js Environment](#1-nodejs-environment)
  - [Python Environment](#2-python-environment)
- [Quick Start Guide](#-quick-start-guide)
  - [Option A: One-Click Launcher (Windows)](#option-a-one-click-launcher-windows)
  - [Option B: Node.js GCS Server (Default)](#option-b-nodejs-gcs-server-default)
  - [Option C: Python FastAPI GCS Server](#option-c-python-fastapi-gcs-server)
- [Computer Vision & Autonomous Flight Modules](#-computer-vision--autonomous-flight-modules)
  - [1. Autonomous Mission Runner](#1-autonomous-mission-runner)
  - [2. Real-Time OpenCV Vision Feed](#2-real-time-opencv-vision-feed)
  - [3. Frame Processing & Analytics Pipeline](#3-frame-processing--analytics-pipeline)
  - [4. Ultra-Low Latency RTSP Relay](#4-ultra-low-latency-rtsp-relay)
- [Project Architecture & File Summary](#-project-architecture--file-summary)
- [Protocol Quick Reference](#-protocol-quick-reference)

---

## 🛸 Overview

This repository provides a reverse-engineered control interface, telemetry decoder, FPV video parser, and web HUD dashboard for RC UFO drones. It enables controlling the drone directly from a web browser, executing pre-programmed autonomous flight paths, and streaming live camera feeds directly into Python/OpenCV/PyTorch for real-time AI and object tracking.

---

## ✨ Key Features

- **🌐 Web Cockpit HUD**: Interactive web interface featuring live video streaming, virtual touch joysticks, keyboard flight controls, synthetic Horizon line, roll/pitch/yaw gauges, and signal strength indicators.
- **⚡ Low-Latency Video Pipeline**: Sub-50ms latency RTSP video demuxing using PyAV/FFmpeg with automatic MJPEG passthrough or fast H.264 decoding.
- **🕹️ Dual Protocol Engine**: Automatic handshake and protocol selection for both **Type 2 (GL Protocol, 21-byte packets)** and **Type 10 (Legacy Protocol, 9-byte packets)**.
- **🤖 Autonomous Mission Control**: Scriptable waypoint and maneuver engine featuring smooth S-curve axis ramping and optical flow stabilization windows.
- **👁️ Computer Vision Ready**: Direct integration with OpenCV and NumPy arrays for vision-guided flight, object tracking, edge detection, and AI inference.

---

## 📡 Hardware & Network Setup

1. **Power on the Drone**: Wait for the LED indicators to signal Wi-Fi broadcast mode.
2. **Connect Wi-Fi**: On your host PC/Laptop, connect to the drone's Wi-Fi network.
   - **SSID Examples**: `KY-WiFi-xxxx`, `4K-WiFi-xxxx`, `RC-UFO-xxxx`
   - **Security**: Open Network (or standard 8-digit password)
3. **Default Drone Network Endpoints**:
   - **Drone Gateway IP**: `192.168.1.1`
   - **UDP Control Port**: `7099` (Flight commands & telemetry)
   - **RTSP Live Video Stream**: `rtsp://192.168.1.1:7070/webcam`
   - **HTTP Web Server**: `http://192.168.1.1:80`
   - **FTP Storage Access**: `ftp://192.168.1.1:21` (`user: ftp`, `pass: ftp`)

---

## 📦 Dependencies & Installation

### System Requirements
- **Node.js**: `v16.0.0` or higher
- **Python**: `v3.8` or higher
- **FFmpeg** (Recommended for video decoder support)

---

### 1. Node.js Environment

Install the required Node.js web server and WebSocket packages:

```bash
npm install
```

#### Required NPM Packages:
| Package | Version | Purpose |
| :--- | :--- | :--- |
| [`express`](https://www.npmjs.com/package/express) | `^4.21.2` | Serves static Web HUD assets & REST control endpoints |
| [`ws`](https://www.npmjs.com/package/ws) | `^8.18.0` | High-frequency WebSocket telemetry & video frame distribution |

---

### 2. Python Environment

Install the Python libraries required for the Python GCS server, RTSP relay, OpenCV streamer, and autonomous flight scripts:

```bash
pip install fastapi uvicorn pillow av numpy requests opencv-python
```

#### Required Python Packages:
| Package | Module Name | Purpose |
| :--- | :--- | :--- |
| `fastapi` | `fastapi` | High-performance Python GCS web server framework |
| `uvicorn` | `uvicorn` | ASGI server for running `server.py` |
| `av` | `av` (PyAV) | Binding for FFmpeg to demux and decode RTSP H.264/MJPEG streams (<50ms latency) |
| `pillow` | `PIL` | Image manipulation, cropping, frame conversion, and HUD overlays |
| `numpy` | `numpy` | High-speed array operations for OpenCV/PyTorch image buffers |
| `requests` | `requests` | HTTP client for frame extraction and REST API flight control |
| `opencv-python` | `cv2` | Real-time computer vision frame display, filtering, and model inference |

---

## 🚀 Quick Start Guide

### Option A: One-Click Launcher (Windows)

Simply double-click or run the batch script in terminal:

```cmd
start.bat
```
*This starts the Node.js GCS server and automatically opens `http://localhost:8080` in your web browser.*

---

### Option B: Node.js GCS Server (Default)

1. Start the server:
   ```bash
   npm start
   ```
   *(or `node server.js`)*

2. Open your web browser and navigate to:
   ```
   http://localhost:8080
   ```

---

### Option C: Python FastAPI GCS Server

If you prefer a pure Python control stack with direct PyAV video demuxing:

1. Run the FastAPI server:
   ```bash
   python server.py
   ```

2. Open your browser at:
   ```
   http://localhost:8080
   ```

---

## 🤖 Computer Vision & Autonomous Flight Modules

### 1. Autonomous Mission Runner
Execute scripted autonomous maneuvers with optical flow stabilization:

```bash
# Run simple 50cm takeoff and landing script
python takeoff_50cm_land.py

# Run takeoff to 1m, 360° flip & land script
python takeoff_1m_flip_land.py

# Run default pre-configured flight mission
python autonomous_mission.py

# Run custom mission script file (e.g. 1m takeoff, flip & land text script)
python autonomous_mission.py --script takeoff_1m_flip_land.txt
```

#### Example Mission Script Syntax (`takeoff_1m_flip_land.txt`):
```text
TAKEOFF
UP 50
FLIP FORWARD
LAND
```

---

### 2. Real-Time OpenCV Vision Feed
Stream live camera frames directly into OpenCV matrices (`cv2.Mat`) for image processing and computer vision development:

```bash
python cv_stream.py
```

---

### 3. Frame Processing & Analytics Pipeline
Pull live camera frames with synchronized drone telemetry (roll, pitch, speed, altitude) and perform background analytics:

```bash
python process_frames.py
```
*Saves processed edge-detected frames and color mask output to `./processed_output`.*

---

### 4. Ultra-Low Latency RTSP Relay
Run a zero-buffer standalone RTSP-to-MJPEG relay stdout pipe:

```bash
python rtsp_relay.py rtsp://192.168.1.1:7070/webcam
```

---

## 📂 Project Architecture & File Summary

| File / Folder | Description |
| :--- | :--- |
| [📁 `public/`](file:///c:/repos/swam-drone/RC-Swamp/public) | Static frontend HUD assets (`index.html`, CSS styling, joystick logic, horizon canvas) |
| [📄 `server.js`](file:///c:/repos/swam-drone/RC-Swamp/server.js) | Primary Node.js GCS server handling UDP sockets (`7099`), RTSP relay spawning, WebSockets, and REST API |
| [📄 `server.py`](file:///c:/repos/swam-drone/RC-Swamp/server.py) | Alternative Python FastAPI GCS server with native PyAV video decoder |
| [📄 `autonomous_mission.py`](file:///c:/repos/swam-drone/RC-Swamp/autonomous_mission.py) | Scriptable autonomous flight runner featuring smooth axis ramping & hover stabilization |
| [📄 `cv_stream.py`](file:///c:/repos/swam-drone/RC-Swamp/cv_stream.py) | OpenCV real-time visualizer streaming 30 FPS camera frames directly to NumPy arrays |
| [📄 `process_frames.py`](file:///c:/repos/swam-drone/RC-Swamp/process_frames.py) | Image processing pipeline for edge detection, brightness statistics, and color mask analysis |
| [📄 `rtsp_relay.py`](file:///c:/repos/swam-drone/RC-Swamp/rtsp_relay.py) | Standalone PyAV/FFmpeg RTSP stream demuxer emitting binary JPEG chunks via stdout |
| [📄 `working.md`](file:///c:/repos/swam-drone/RC-Swamp/working.md) | Comprehensive reverse-engineered specification of the RC UFO protocol |
| [📄 `package.json`](file:///c:/repos/swam-drone/RC-Swamp/package.json) | Node.js project manifest & dependency definition |
| [📄 `start.bat`](file:///c:/repos/swam-drone/RC-Swamp/start.bat) | Windows batch file to launch the cockpit UI with one click |

---

## 🛠️ Protocol Quick Reference

For detailed protocol specifications, byte-level packet structures, checksum calculations, and trim formulas, consult [`working.md`](file:///c:/repos/swam-drone/RC-Swamp/working.md).

### Control Values Mapping
- **Stick Neutral Center**: `128` (Range: `1` – `255`)
- **Roll**: `1` (Full Left) ⬅️ `128` (Center) ➡️ `255` (Full Right)
- **Pitch**: `1` (Full Down/Back) ⬇️ `128` (Center) ⬆️ `255` (Full Up/Forward)
- **Throttle**: `1` (Min Power) ⬇️ `128` (Center) ⬆️ `255` (Max Climb)
- **Yaw**: `1` (Full Rotate Left) ↩️ `128` (Center) ↪️ `255` (Full Rotate Right)
- **Speed Rates**: Gear 1 (`30%`), Gear 2 (`60%`), Gear 3 (`100%`)

---

## 📜 License & Acknowledgments

Developed for RC UFO Drone (`com.cooingdv.rcufo`) protocol research, autonomous control, and computer vision experimentation.
