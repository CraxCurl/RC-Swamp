const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const dgram = require('dgram');
const path = require('path');
const fs = require('fs');
const net = require('net');
const { spawn } = require('child_process');

// Drone Network Configuration
let DRONE_IP = process.env.DRONE_IP || '192.168.1.1';
let UDP_CONTROL_PORT = parseInt(process.env.UDP_PORT || '7099', 10);
let RTSP_URL = process.env.RTSP_URL || 'rtsp://192.168.1.1:7070/webcam';
const WEB_PORT = parseInt(process.env.PORT || '8080', 10);

// Ensure Frames Storage Directory Exists
const FRAMES_DIR = path.join(__dirname, 'camera_data');
if (!fs.existsSync(FRAMES_DIR)) {
  fs.mkdirSync(FRAMES_DIR, { recursive: true });
}

const app = express();
app.use(express.json());
const server = http.createServer(app);
const wssTelemetry = new WebSocket.Server({ noServer: true });
const wssVideo = new WebSocket.Server({ noServer: true });

// Global Drone State
const droneState = {
  deviceType: 2, // 2: GL (21-Byte), 10: Legacy (9-Byte)
  connected: false,
  droneIp: DRONE_IP,
  udpPort: UDP_CONTROL_PORT,
  rtspUrl: RTSP_URL,

  // Flight Channels (1..255, 128 center hover)
  roll: 128,
  pitch: 128,
  throttle: 128,
  yaw: 128,

  // Action Flags
  isAirborne: false,
  isFastFly: false,
  isFastDrop: false,
  isEmergencyStop: false,
  isGyroCorrection: false,
  isCircleTurnEnd: false,
  isNoHeadMode: false,
  isFixedHeight: true, // Altitude Hold & Optical Flow Lock
  isGestureMode: false,

  // Trims & Modes
  rollTrim: 0,
  pitchTrim: 0,
  yawTrim: 0,
  gear: 2, // 1: 30%, 2: 60%, 3: 100%
  cameraId: 1, // 1: Front, 2: Bottom
  isFlipping: false,
  lockedVideoSource: null,
  lastSourceFrameTime: 0,

  // Telemetry & Diagnostics
  packetsSent: 0,
  packetsReceived: 0,
  lastTelemetryTime: 0,
  lastVideoTime: 0,
  hasLiveVideo: false,
  videoFramesCount: 0,
  latestFrameBytes: null,
  latestFrameTimestamp: 0,
  activeVideoSource: 'NONE',
  fps: 0,
  connectionDiagnostic: 'AWAITING_DRONE_CONNECTION',

  // Image Processing & Auto-Capture on Disk
  autoCaptureEnabled: false,
  autoCaptureFps: 2,
  lastAutoCaptureTime: 0,
  savedFramesCount: 0
};

// Count existing frames in camera_data
try {
  droneState.savedFramesCount = fs.readdirSync(FRAMES_DIR).filter(f => f.endsWith('.jpg')).length;
} catch (e) {}

// ----------------- WebSocket Video & Telemetry Clients ----------------- //
const videoClients = new Set();
const telemetryClients = new Set();

function saveFrameToDisk(buffer, label = 'frame') {
  if (!buffer || buffer.length < 100) return null;
  const now = new Date();
  const pad = (n, l = 2) => String(n).padStart(l, '0');
  const timestampStr = `${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}_${pad(now.getMilliseconds(), 3)}`;
  const filename = `${label}_${timestampStr}.jpg`;
  const metaFilename = `${label}_${timestampStr}.json`;
  const filepath = path.join(FRAMES_DIR, filename);
  const metaFilepath = path.join(FRAMES_DIR, metaFilename);

  fs.writeFile(filepath, buffer, (err) => {
    if (!err) {
      droneState.savedFramesCount++;
      // Save sidecar flight metadata for synchronized CV/AI processing
      const meta = {
        filename,
        timestamp: now.toISOString(),
        frame_number: droneState.videoFramesCount,
        telemetry: {
          roll: droneState.roll,
          pitch: droneState.pitch,
          throttle: droneState.throttle,
          yaw: droneState.yaw,
          gear: droneState.gear,
          is_fixed_height: droneState.isFixedHeight,
          device_type: droneState.deviceType,
          connected: droneState.connected
        }
      };
      fs.writeFile(metaFilepath, JSON.stringify(meta, null, 2), () => {});
    }
  });

  return { filename, filepath, timestamp: now.toISOString(), frame_number: droneState.videoFramesCount };
}

function broadcastVideoFrame(buffer, source = 'PyAV-RTSP') {
  if (!buffer || buffer.length < 100) return;
  const now = Date.now();

  droneState.lockedVideoSource = source;
  droneState.lastSourceFrameTime = now;
  droneState.hasLiveVideo = true;
  droneState.lastVideoTime = now;
  droneState.videoFramesCount++;
  droneState.latestFrameBytes = buffer;
  droneState.latestFrameTimestamp = now;
  droneState.activeVideoSource = source;

  // Auto-Capture Frame to Disk if enabled
  if (droneState.autoCaptureEnabled) {
    const minIntervalMs = 1000 / Math.max(1, droneState.autoCaptureFps);
    if (now - droneState.lastAutoCaptureTime >= minIntervalMs) {
      droneState.lastAutoCaptureTime = now;
      saveFrameToDisk(buffer, 'autocap');
    }
  }

  if (videoClients.size === 0) return;
  for (const client of videoClients) {
    // Zero-lag queueing: Send if client socket buffer is under 256 KB backpressure limit
    if (client.readyState === WebSocket.OPEN && client.bufferedAmount < 256 * 1024) {
      client.send(buffer, { binary: true });
    }
  }
}

// ----------------- 1. PyAV Continuous RTSP Engine ----------------- //
let pyAvProcess = null;
let pyAvRestartTimer = null;

function startPyAvRtspRelay() {
  if (pyAvProcess) {
    try { pyAvProcess.kill(); } catch (e) {}
    pyAvProcess = null;
  }

  const scriptPath = path.join(__dirname, 'rtsp_relay.py');
  console.log(`[*] Spawning PyAV Continuous RTSP Relay -> ${droneState.rtspUrl}`);

  pyAvProcess = spawn('python', [scriptPath, droneState.rtspUrl], {
    stdio: ['ignore', 'pipe', 'pipe']
  });

  let lengthBuffer = Buffer.alloc(4);
  let lengthBytesRead = 0;
  let targetPayloadLength = 0;
  let payloadBuffer = null;
  let payloadBytesRead = 0;

  pyAvProcess.stdout.on('data', (chunk) => {
    let offset = 0;
    while (offset < chunk.length) {
      if (targetPayloadLength === 0) {
        const needed = 4 - lengthBytesRead;
        const available = chunk.length - offset;
        const toCopy = Math.min(needed, available);
        chunk.copy(lengthBuffer, lengthBytesRead, offset, offset + toCopy);
        lengthBytesRead += toCopy;
        offset += toCopy;

        if (lengthBytesRead === 4) {
          targetPayloadLength = lengthBuffer.readUInt32BE(0);
          lengthBytesRead = 0;
          if (targetPayloadLength > 0 && targetPayloadLength < 5 * 1024 * 1024) {
            payloadBuffer = Buffer.allocUnsafe(targetPayloadLength);
            payloadBytesRead = 0;
          } else {
            targetPayloadLength = 0;
          }
        }
      } else {
        const needed = targetPayloadLength - payloadBytesRead;
        const available = chunk.length - offset;
        const toCopy = Math.min(needed, available);
        chunk.copy(payloadBuffer, payloadBytesRead, offset, offset + toCopy);
        payloadBytesRead += toCopy;
        offset += toCopy;

        if (payloadBytesRead === targetPayloadLength) {
          broadcastVideoFrame(payloadBuffer, 'PyAV-RTSP');
          targetPayloadLength = 0;
          payloadBuffer = null;
          payloadBytesRead = 0;
        }
      }
    }
  });

  pyAvProcess.stderr.on('data', (data) => {
    const msg = data.toString().trim();
    if (msg) console.log(`[PyAV] ${msg}`);
  });

  pyAvProcess.on('exit', (code) => {
    clearTimeout(pyAvRestartTimer);
    pyAvRestartTimer = setTimeout(startPyAvRtspRelay, 800);
  });
}

startPyAvRtspRelay();

// ----------------- 2. UDP Control Socket & Protocol ----------------- //
const udpClient = dgram.createSocket({ type: 'udp4', reuseAddr: true });

udpClient.on('error', (err) => {
  console.warn('[UDP Warning]', err.message);
});

function sendDroneUdp(buffer) {
  try {
    udpClient.send(buffer, 0, buffer.length, droneState.udpPort, droneState.droneIp, (err) => {
      if (!err) {
        droneState.packetsSent++;
      }
    });
  } catch (e) {}
}

function buildControlPacket() {
  const r = Math.max(1, Math.min(255, Math.round(droneState.roll)));
  const p = Math.max(1, Math.min(255, Math.round(droneState.pitch)));
  const t = Math.max(0, Math.min(255, Math.round(droneState.throttle)));
  const y = Math.max(1, Math.min(255, Math.round(droneState.yaw)));

  if (droneState.deviceType !== 10) {
    // GL Protocol (Type 2 - 21 Bytes)
    let flags1 = 0;
    if (droneState.isFastFly || droneState.isFastDrop) flags1 |= 0x01;
    if (droneState.isEmergencyStop) flags1 |= 0x02;
    if (droneState.isGyroCorrection) flags1 |= 0x04;
    if (droneState.isCircleTurnEnd) flags1 |= 0x08;
    if (droneState.isGestureMode) flags1 |= 0x40;

    let flags2 = 0;
    if (droneState.isNoHeadMode) flags2 |= 0x01;
    if (droneState.isFixedHeight) flags2 |= 0x02;

    const cs = (flags1 ^ (((p ^ r) ^ t) ^ y)) ^ (flags2 & 0xFF);

    const pkt = Buffer.alloc(21);
    pkt[0] = 0x03;
    pkt[1] = 0x66;
    pkt[2] = 0x14;
    pkt[3] = r;
    pkt[4] = p;
    pkt[5] = t;
    pkt[6] = y;
    pkt[7] = flags1;
    pkt[8] = flags2;
    pkt[19] = cs & 0xFF;
    pkt[20] = 0x99;
    return pkt;
  } else {
    // Legacy Protocol (Type 10 - 9 Bytes)
    let flags = 0;
    if (droneState.isFastFly) flags += 1;
    if (droneState.isFastDrop) flags += 2;
    if (droneState.isEmergencyStop) flags += 4;
    if (droneState.isCircleTurnEnd) flags += 8;
    if (droneState.isNoHeadMode) flags += 16;
    if (droneState.isGyroCorrection) flags += 128;

    const cs = (((r ^ p) ^ t) ^ y) ^ (flags & 0xFF);
    return Buffer.from([
      0x03, 0x66, r, p, t, y, flags & 0xFF, cs & 0xFF, 0x99
    ]);
  }
}

// 1 Hz Heartbeat Ping to Drone Firmware (Port 7099)
// NOTE: Heartbeat is strictly [0x01, 0x01]. Do NOT send 0x06 here as 0x06 triggers hardware camera switching.
setInterval(() => {
  sendDroneUdp(Buffer.from([0x01, 0x01]));
}, 1000);

// Helper function to send immediate UDP bursts for critical flight triggers
function sendFlightPacketBurst(count = 3) {
  const pkt = buildControlPacket();
  for (let i = 0; i < count; i++) {
    sendDroneUdp(pkt);
  }
}

// 25 Hz Flight Control Loop (every 40 ms) - Continuous stream to drone port 7099
setInterval(() => {
  const pkt = buildControlPacket();
  sendDroneUdp(pkt);

  const now = Date.now();
  if (now - droneState.lastTelemetryTime > 3500) {
    if (droneState.connected) {
      droneState.connected = false;
      droneState.connectionDiagnostic = 'SEARCHING_DRONE_WIFI';
    }
  }

  if (now - droneState.lastVideoTime > 3500) {
    droneState.hasLiveVideo = false;
  }
}, 40);

// Ingest Incoming UDP Telemetry on Port 7099
udpClient.on('message', (msg, rinfo) => {
  if (rinfo && rinfo.address !== droneState.droneIp) {
    return;
  }

  droneState.packetsReceived++;
  droneState.lastTelemetryTime = Date.now();
  droneState.connected = true;
  droneState.connectionDiagnostic = 'CONNECTED_TO_DRONE';

  if (msg.length >= 1) {
    const devId = msg[0];
    const isGl = (devId >= 90 && devId <= 101) || devId === 103 || devId === 82 || devId === 85 || devId === 88;
    droneState.deviceType = isGl ? 2 : 10;
  }

  // Remote Transmitter Shutter ACK & Auto-Capture
  if (msg.length > 4) {
    if (msg[2] === 0x4D) {
      sendDroneUdp(Buffer.from([0x09, 0x01]));
      if (droneState.latestFrameBytes) {
        saveFrameToDisk(droneState.latestFrameBytes, 'remote_shutter');
      }
    } else if (msg[2] === 0x58) {
      sendDroneUdp(Buffer.from([0x09, 0x02]));
    }
  }
});

function triggerFlip360(direction = 'forward') {
  const dir = (direction || 'forward').toLowerCase();
  droneState.isFlipping = true;

  // STAGE 1: Arm Flip Mode (Flip bit ON, Sticks NEUTRAL 128 for 250ms pre-flip climb)
  droneState.isCircleTurnEnd = true;
  droneState.roll = 128;
  droneState.pitch = 128;
  droneState.throttle = 128;
  droneState.yaw = 128;

  sendDroneUdp(Buffer.from([0x07, 0x01]));
  sendDroneUdp(Buffer.from([0x08, 0x01]));
  sendFlightPacketBurst(5);

  // STAGE 2: Apply Directional Stick Deflection after 250ms pre-flip climb
  setTimeout(() => {
    if (!droneState.isFlipping) return;
    if (dir === 'left') droneState.roll = 1;
    else if (dir === 'right') droneState.roll = 255;
    else if (dir === 'backward') droneState.pitch = 1;
    else droneState.pitch = 255; // Default FORWARD flip
    sendFlightPacketBurst(5);
  }, 250);

  // STAGE 3: Reset after 900ms total flip execution window
  setTimeout(() => {
    droneState.isFlipping = false;
    droneState.isCircleTurnEnd = false;
    droneState.roll = 128;
    droneState.pitch = 128;
    droneState.throttle = 128;
    droneState.yaw = 128;
  }, 900);
}

// ----------------- WebSocket Handlers ----------------- //
wssTelemetry.on('connection', (ws) => {
  telemetryClients.add(ws);

  ws.on('message', (message) => {
    try {
      const msg = JSON.parse(message);
      if (msg.action === 'stick') {
        droneState.roll = typeof msg.roll === 'number' ? msg.roll : droneState.roll;
        droneState.pitch = typeof msg.pitch === 'number' ? msg.pitch : droneState.pitch;
        droneState.throttle = typeof msg.throttle === 'number' ? msg.throttle : droneState.throttle;
        droneState.yaw = typeof msg.yaw === 'number' ? msg.yaw : droneState.yaw;
      } else if (msg.action === 'takeoff') {
        droneState.isAirborne = true;
        droneState.isFastFly = true;
        droneState.isFastDrop = false;
        droneState.isEmergencyStop = false;
        sendFlightPacketBurst(3);
        setTimeout(() => { droneState.isFastFly = false; }, 1500);
      } else if (msg.action === 'land') {
        droneState.isAirborne = false;
        droneState.isFastDrop = true;
        droneState.isFastFly = false;
        sendFlightPacketBurst(3);
        setTimeout(() => { droneState.isFastDrop = false; }, 1500);
      } else if (msg.action === 'emergency_stop') {
        droneState.isAirborne = false;
        droneState.isEmergencyStop = true;
        droneState.isFastFly = false;
        droneState.isFastDrop = false;
        droneState.throttle = 0;
        sendFlightPacketBurst(5);
        sendDroneUdp(Buffer.from([0x08, 0x01])); // Disarm
        setTimeout(() => { droneState.isEmergencyStop = false; }, 1000);
      } else if (msg.action === 'calibrate_gyro') {
        droneState.isGyroCorrection = true;
        droneState.roll = 128;
        droneState.pitch = 128;
        droneState.throttle = 128;
        droneState.yaw = 128;
        sendFlightPacketBurst(3);
        setTimeout(() => { droneState.isGyroCorrection = false; }, 2000);
      } else if (msg.action === 'flip_360') {
        triggerFlip360(msg.direction);
      } else if (msg.action === 'toggle_headless') {
        droneState.isNoHeadMode = !droneState.isNoHeadMode;
        sendFlightPacketBurst(2);
      } else if (msg.action === 'toggle_altitude_hold') {
        droneState.isFixedHeight = !droneState.isFixedHeight;
        sendFlightPacketBurst(2);
      } else if (msg.action === 'set_gear') {
        droneState.gear = msg.gear;
      } else if (msg.action === 'switch_camera') {
        const targetCam = parseInt(msg.camera_id, 10) || 1;
        droneState.cameraId = targetCam;
        droneState.lockedVideoSource = null;
        droneState.lastSourceFrameTime = 0;
        const camPkt = Buffer.from([0x06, targetCam]);
        sendDroneUdp(camPkt);
        sendDroneUdp(camPkt);
        sendDroneUdp(camPkt);
        console.log(`[Camera Switch] Sent UDP 0x06 ${targetCam} burst to drone. Re-syncing RTSP stream...`);
        startPyAvRtspRelay();
      } else if (msg.action === 'set_trims') {
        droneState.rollTrim = msg.roll_trim;
        droneState.pitchTrim = msg.pitch_trim;
        droneState.yawTrim = msg.yaw_trim;
      } else if (msg.action === 'capture_frame_disk') {
        if (droneState.latestFrameBytes) {
          saveFrameToDisk(droneState.latestFrameBytes, 'manual_snap');
        }
      } else if (msg.action === 'set_auto_capture') {
        droneState.autoCaptureEnabled = !!msg.enabled;
        if (typeof msg.fps === 'number') droneState.autoCaptureFps = msg.fps;
      } else if (msg.action === 'disarm') {
        sendDroneUdp(Buffer.from([0x08, 0x01]));
      } else if (msg.action === 'update_config') {
        if (msg.drone_ip) {
          droneState.droneIp = msg.drone_ip;
          DRONE_IP = msg.drone_ip;
        }
        if (msg.udp_port) {
          droneState.udpPort = parseInt(msg.udp_port, 10);
        }
        if (msg.rtsp_url) {
          droneState.rtspUrl = msg.rtsp_url;
          startPyAvRtspRelay();
        }
      }
    } catch (e) {}
  });

  ws.on('close', () => telemetryClients.delete(ws));
  ws.on('error', () => telemetryClients.delete(ws));
});

wssVideo.on('connection', (ws) => {
  videoClients.add(ws);
  if (droneState.latestFrameBytes) {
    try {
      ws.send(droneState.latestFrameBytes, { binary: true });
    } catch (e) {}
  }
  ws.on('close', () => videoClients.delete(ws));
  ws.on('error', () => videoClients.delete(ws));
});

// Broadcast Telemetry (10 Hz)
setInterval(() => {
  if (telemetryClients.size === 0) return;
  const payload = JSON.stringify({
    type: 'telemetry',
    connected: droneState.connected,
    has_live_video: droneState.hasLiveVideo,
    video_source: droneState.activeVideoSource,
    frames_count: droneState.videoFramesCount,
    saved_frames_count: droneState.savedFramesCount,
    auto_capture: droneState.autoCaptureEnabled,
    auto_capture_fps: droneState.autoCaptureFps,
    device_type: droneState.deviceType === 2 ? 'GL (21-Byte)' : 'Legacy (9-Byte)',
    roll: droneState.roll,
    pitch: droneState.pitch,
    throttle: droneState.throttle,
    yaw: droneState.yaw,
    is_fast_fly: droneState.isFastFly,
    is_fast_drop: droneState.isFastDrop,
    is_emergency_stop: droneState.isEmergencyStop,
    is_gyro_correction: droneState.isGyroCorrection,
    is_no_head_mode: droneState.isNoHeadMode,
    is_fixed_height: droneState.isFixedHeight,
    is_circle_turn_end: droneState.isCircleTurnEnd,
    gear: droneState.gear,
    camera_id: droneState.cameraId,
    roll_trim: droneState.rollTrim,
    pitch_trim: droneState.pitchTrim,
    yaw_trim: droneState.yawTrim,
    fps: droneState.fps,
    packets_sent: droneState.packetsSent,
    packets_received: droneState.packetsReceived,
    diagnostic: droneState.connectionDiagnostic,
    drone_ip: droneState.droneIp,
    rtsp_url: droneState.rtspUrl
  });

  for (const client of telemetryClients) {
    if (client.readyState === WebSocket.OPEN) {
      client.send(payload);
    }
  }
}, 100);

// ----------------- HTTP Endpoints & Image Processing API ----------------- //
server.on('upgrade', (request, socket, head) => {
  const pathname = request.url;
  if (pathname === '/ws/telemetry') {
    wssTelemetry.handleUpgrade(request, socket, head, (ws) => {
      wssTelemetry.emit('connection', ws, request);
    });
  } else if (pathname === '/ws/video') {
    wssVideo.handleUpgrade(request, socket, head, (ws) => {
      wssVideo.emit('connection', ws, request);
    });
  } else {
    socket.destroy();
  }
});

// Serve static frontend and captured frames
app.use(express.static(path.join(__dirname, 'public')));
app.use('/camera_data', express.static(FRAMES_DIR));

// Hardware Physical Sensor Ingestion API (ESP32 / Arduino / Python sensor scripts)
app.post('/api/telemetry/sensor', (req, res) => {
  const sensorData = req.body;
  if (sensorData) {
    for (const client of telemetryClients) {
      if (client.readyState === WebSocket.OPEN) {
        client.send(JSON.stringify({ type: 'telemetry', hardware_imu: sensorData }));
      }
    }
    return res.json({ success: true, received: sensorData });
  }
  return res.status(400).json({ error: 'Missing sensorData in request body' });
});

// 0. REST Control API (for Autonomous Python Mission Scripts & Web clients)
app.post('/api/control', (req, res) => {
  const { roll, pitch, throttle, yaw, take_off, land, emergency, flags, action, direction, camera_id } = req.body;

  if (!droneState.isFlipping) {
    if (typeof roll === 'number') droneState.roll = Math.max(0, Math.min(255, roll));
    if (typeof pitch === 'number') droneState.pitch = Math.max(0, Math.min(255, pitch));
  }
  if (typeof throttle === 'number') droneState.throttle = Math.max(0, Math.min(255, throttle));
  if (typeof yaw === 'number') droneState.yaw = Math.max(0, Math.min(255, yaw));

  if (action === 'flip_360') {
    triggerFlip360(direction || 'forward');
  } else if (action === 'switch_camera') {
    const camId = camera_id || (droneState.cameraId === 1 ? 2 : 1);
    droneState.cameraId = camId;
    droneState.lockedVideoSource = null;
    droneState.lastSourceFrameTime = 0;
    
    // Broadcast switch packet 3x for reliable UDP delivery
    const switchPacket = Buffer.from([0x06, camId]);
    sendDroneUdp(switchPacket);
    setTimeout(() => sendDroneUdp(switchPacket), 40);
    setTimeout(() => sendDroneUdp(switchPacket), 100);

    // Also send on port 8090 for legacy E88 firmware
    try {
      udpClient.send(switchPacket, 0, switchPacket.length, 8090, droneState.droneIp, () => {});
    } catch (e) {}

    console.log(`[*] Switched Camera Lens to: Camera #${camId} (${camId === 2 ? 'BOTTOM' : 'FRONT'}). Restarting RTSP stream...`);

    // Restart PyAV RTSP relay so it reconnects to the newly switched sensor resolution/feed
    clearTimeout(pyAvRestartTimer);
    if (pyAvProcess) {
      try { pyAvProcess.kill(); } catch (e) {}
      pyAvProcess = null;
    }
    setTimeout(startPyAvRtspRelay, 400);
  }

  if (take_off) {
    droneState.isAirborne = true;
    droneState.isFastFly = true;
    setTimeout(() => { droneState.isFastFly = false; }, 400);
  }
  if (land) {
    droneState.isAirborne = false;
    droneState.isFastDrop = true;
    setTimeout(() => { droneState.isFastDrop = false; }, 400);
  }
  if (emergency) {
    droneState.isAirborne = false;
    droneState.isEmergencyStop = true;
    setTimeout(() => { droneState.isEmergencyStop = false; }, 400);
  }
  if (typeof flags === 'number') {
    droneState.flags2 = flags;
  }

  res.json({
    success: true,
    telemetry: {
      roll: droneState.roll,
      pitch: droneState.pitch,
      throttle: droneState.throttle,
      yaw: droneState.yaw,
      is_fast_fly: droneState.isFastFly,
      is_fast_drop: droneState.isFastDrop,
      is_emergency_stop: droneState.isEmergencyStop
    }
  });
});

// 1. Status API
app.get('/api/status', (req, res) => {
  res.json({
    drone_ip: droneState.droneIp,
    udp_port: droneState.udpPort,
    rtsp_url: droneState.rtspUrl,
    connected: droneState.connected,
    has_live_video: droneState.hasLiveVideo,
    video_source: droneState.activeVideoSource,
    frames_count: droneState.videoFramesCount,
    saved_frames_count: droneState.savedFramesCount,
    auto_capture: droneState.autoCaptureEnabled,
    auto_capture_fps: droneState.autoCaptureFps,
    device_type: droneState.deviceType,
    packets_sent: droneState.packetsSent,
    packets_received: droneState.packetsReceived,
    diagnostic: droneState.connectionDiagnostic
  });
});

// 2. Raw JPEG Image Stream / Single Frame for Python OpenCV & AI scripts
app.get('/api/camera-frame', (req, res) => {
  if (droneState.latestFrameBytes) {
    res.set('Content-Type', 'image/jpeg');
    res.set('Cache-Control', 'no-cache, no-store');
    res.set('X-Frame-Number', String(droneState.videoFramesCount));
    res.set('X-Timestamp', String(droneState.latestFrameTimestamp));
    res.set('X-Telemetry-Roll', String(droneState.roll));
    res.set('X-Telemetry-Pitch', String(droneState.pitch));
    res.set('X-Telemetry-Yaw', String(droneState.yaw));
    res.set('X-Telemetry-Throttle', String(droneState.throttle));
    res.send(droneState.latestFrameBytes);
  } else {
    res.status(503).json({ error: 'No camera frame received yet from drone. Connect to drone Wi-Fi.' });
  }
});

// 2b. Continuous HTTP MJPEG Multipart Stream Endpoint
app.get('/api/camera-frame/stream', (req, res) => {
  res.writeHead(200, {
    'Content-Type': 'multipart/x-mixed-replace; boundary=--myboundary',
    'Cache-Control': 'no-cache, no-store, must-revalidate',
    'Connection': 'close',
    'Pragma': 'no-cache'
  });

  const sendFrame = () => {
    if (res.writableEnded || res.destroyed) return;
    if (droneState.latestFrameBytes) {
      try {
        res.write(`--myboundary\r\nContent-Type: image/jpeg\r\nContent-Length: ${droneState.latestFrameBytes.length}\r\n\r\n`);
        res.write(droneState.latestFrameBytes);
        res.write('\r\n');
      } catch (e) {}
    }
  };

  const interval = setInterval(sendFrame, 40); // 25 FPS MJPEG HTTP stream
  req.on('close', () => clearInterval(interval));
  req.on('error', () => clearInterval(interval));
});

// 3. JSON Image Endpoint with Base64 + Synchronized Flight Telemetry
app.get('/api/camera-frame/json', (req, res) => {
  if (droneState.latestFrameBytes) {
    res.json({
      frame_number: droneState.videoFramesCount,
      timestamp: droneState.latestFrameTimestamp,
      image_base64: droneState.latestFrameBytes.toString('base64'),
      telemetry: {
        roll: droneState.roll,
        pitch: droneState.pitch,
        throttle: droneState.throttle,
        yaw: droneState.yaw,
        gear: droneState.gear,
        is_fixed_height: droneState.isFixedHeight,
        device_type: droneState.deviceType,
        connected: droneState.connected
      }
    });
  } else {
    res.status(503).json({ error: 'No camera frame available' });
  }
});

// 4. Capture and Save Snapshot to Disk
app.post('/api/capture-snapshot', (req, res) => {
  if (droneState.latestFrameBytes) {
    const result = saveFrameToDisk(droneState.latestFrameBytes, 'snapshot');
    res.json({ success: true, ...result, total_saved: droneState.savedFramesCount });
  } else {
    res.status(503).json({ error: 'No frame available to snapshot' });
  }
});

// 5. Toggle Auto-Capture to Disk
app.post('/api/auto-capture', (req, res) => {
  const { enabled, fps } = req.body;
  if (typeof enabled === 'boolean') droneState.autoCaptureEnabled = enabled;
  if (typeof fps === 'number' && fps > 0) droneState.autoCaptureFps = fps;
  res.json({
    success: true,
    auto_capture: droneState.autoCaptureEnabled,
    fps: droneState.autoCaptureFps,
    storage_dir: FRAMES_DIR,
    total_saved: droneState.savedFramesCount
  });
});

// 6. List all saved frames on disk
app.get('/api/frames-list', (req, res) => {
  try {
    const files = fs.readdirSync(FRAMES_DIR)
      .filter(f => f.endsWith('.jpg'))
      .sort((a, b) => fs.statSync(path.join(FRAMES_DIR, b)).mtimeMs - fs.statSync(path.join(FRAMES_DIR, a)).mtimeMs)
      .slice(0, 100)
      .map(filename => ({
        filename,
        url: `/camera_data/${filename}`,
        size_bytes: fs.statSync(path.join(FRAMES_DIR, filename)).size,
        created_at: fs.statSync(path.join(FRAMES_DIR, filename)).mtime.toISOString()
      }));
    res.json({ frames: files, total: droneState.savedFramesCount, storage_path: FRAMES_DIR });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// 7. Save Config Endpoint
app.post('/api/config', (req, res) => {
  const { drone_ip, udp_port, rtsp_url } = req.body;
  if (drone_ip) droneState.droneIp = drone_ip;
  if (udp_port) droneState.udpPort = parseInt(udp_port, 10);
  if (rtsp_url) {
    droneState.rtspUrl = rtsp_url;
    startPyAvRtspRelay();
  }
  res.json({ success: true, config: droneState });
});

// ----------------- 8. IMU Telemetry, 3D Coordinates & CSV Export APIs ----------------- //
const serverImuLogs = [];
let sessionFlightTimeSeconds = 0;
let currentSortieTimeSeconds = 0;
let lastTimerTick = Date.now();
let serverPosX = 0.0;
let serverPosY = 0.0;
let serverPosZ = 0.0;
let serverYawHeading = 0.0;
let serverTotalDistance = 0.0;

// 10 Hz Server-side IMU Telemetry & Coordinates Sampler
setInterval(() => {
  const now = Date.now();
  const dt = (now - lastTimerTick) / 1000;
  lastTimerTick = now;

  const isAirborne = droneState.isAirborne || droneState.isFastFly;
  if (isAirborne) {
    currentSortieTimeSeconds += dt;
    sessionFlightTimeSeconds += dt;
  } else {
    currentSortieTimeSeconds = 0;
  }

  // Calculate IMU derivatives based on calibrated physics
  const rollDeg = (droneState.roll - 128) * (28.5 / 127.0);
  const pitchDeg = (droneState.pitch - 128) * (28.5 / 127.0);
  
  // Continuous yaw heading integration (deg/s rate)
  const yawRate = (droneState.yaw - 128) / 127.0 * 130.0; // max 130 deg/s
  if (isAirborne || Math.abs(yawRate) > 3.0) {
    serverYawHeading = (serverYawHeading + yawRate * dt + 360) % 360;
  }

  const rollRad = (rollDeg * Math.PI) / 180;
  const pitchRad = (pitchDeg * Math.PI) / 180;
  const yawRad = (serverYawHeading * Math.PI) / 180;
  
  const thrOffsetG = (droneState.throttle - 128) / 127.0;
  const verticalThrustG = isAirborne ? Math.max(0.5, 1.0 + thrOffsetG * 0.35) : 1.0;

  const accelX = Math.sin(pitchRad);
  const accelY = -Math.sin(rollRad) * Math.cos(pitchRad);
  const accelZ = Math.cos(rollRad) * Math.cos(pitchRad) * verticalThrustG;
  const totalG = Math.sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  // 3D Coordinates Dead-Reckoning (Origin 0,0,0)
  const maxLinearSpeed = droneState.gear === 1 ? 0.35 : droneState.gear === 3 ? 0.85 : 0.65;
  let velX = 0, velY = 0, velZ = 0;
  if (isAirborne) {
    const bodyForwardVel = (pitchDeg / 28.5) * maxLinearSpeed;
    const bodyStrafeVel = (rollDeg / 28.5) * maxLinearSpeed;
    velX = (bodyStrafeVel * Math.cos(yawRad)) + (bodyForwardVel * Math.sin(yawRad));
    velY = (-bodyStrafeVel * Math.sin(yawRad)) + (bodyForwardVel * Math.cos(yawRad));
    
    serverPosX += velX * dt;
    serverPosY += velY * dt;
    
    // Altitude-hold climb / descent dynamics
    if (currentSortieTimeSeconds < 2.5 && serverPosZ < 0.85) {
      serverPosZ = Math.min(1.2, serverPosZ + 0.45 * dt);
    } else {
      const vz = thrOffsetG * 0.65;
      serverPosZ = Math.max(0.05, serverPosZ + vz * dt);
    }
    serverTotalDistance += Math.sqrt(velX * velX + velY * velY) * dt;
  } else {
    serverPosZ = 0;
  }

  const distHome = Math.sqrt(serverPosX * serverPosX + serverPosY * serverPosY + serverPosZ * serverPosZ);

  const sample = {
    timestamp_iso: new Date().toISOString(),
    timestamp_epoch_ms: now,
    total_session_flight_time_s: parseFloat(sessionFlightTimeSeconds.toFixed(2)),
    current_flight_time_s: parseFloat(currentSortieTimeSeconds.toFixed(2)),
    pos_x_east_m: parseFloat(serverPosX.toFixed(3)),
    pos_y_north_m: parseFloat(serverPosY.toFixed(3)),
    pos_z_alt_m: parseFloat(serverPosZ.toFixed(3)),
    distance_to_home_m: parseFloat(distHome.toFixed(2)),
    total_distance_traveled_m: parseFloat(serverTotalDistance.toFixed(2)),
    roll_deg: parseFloat(rollDeg.toFixed(2)),
    pitch_deg: parseFloat(pitchDeg.toFixed(2)),
    yaw_heading_deg: parseFloat(serverYawHeading.toFixed(1)),
    accel_x_g: parseFloat(accelX.toFixed(3)),
    accel_y_g: parseFloat(accelY.toFixed(3)),
    accel_z_g: parseFloat(accelZ.toFixed(3)),
    total_g_load: parseFloat(totalG.toFixed(2)),
    estimated_altitude_pct: Math.min(100, Math.max(0, Math.round((serverPosZ / 3.0) * 100))),
    estimated_altitude_m: parseFloat(serverPosZ.toFixed(2)),
    raw_roll_channel_1_255: droneState.roll,
    raw_pitch_channel_1_255: droneState.pitch,
    raw_throttle_channel_0_255: droneState.throttle,
    raw_yaw_channel_1_255: droneState.yaw,
    roll_trim: droneState.rollTrim,
    pitch_trim: droneState.pitchTrim,
    yaw_trim: droneState.yawTrim,
    speed_gear_pct: droneState.gear === 1 ? 30 : droneState.gear === 3 ? 100 : 60,
    altitude_hold_active: droneState.isFixedHeight ? 1 : 0,
    headless_mode_active: droneState.isNoHeadMode ? 1 : 0,
    gyro_calibration_active: droneState.isGyroCorrection ? 1 : 0,
    stunt_flip_active: droneState.isCircleTurnEnd ? 1 : 0,
    protocol_type: droneState.deviceType === 2 ? 'GL-21B' : 'Legacy-9B',
    latency_ms: 8,
    packets_sent: droneState.packetsSent,
    packets_received: droneState.packetsReceived
  };

  // Change-Detection Logging (Only record on changes or periodic 3s flight heartbeat)
  const prev = serverImuLogs.length > 0 ? serverImuLogs[serverImuLogs.length - 1] : null;
  let shouldLog = false;
  let eventName = 'DATA';

  if (!prev) {
    shouldLog = true;
    eventName = 'INITIALIZE';
  } else if (isAirborne !== (prev.is_airborne_state || false)) {
    shouldLog = true;
    eventName = isAirborne ? 'TAKEOFF' : 'LANDED';
  } else if (Math.abs(sample.raw_roll_channel_1_255 - prev.raw_roll_channel_1_255) >= 3 ||
             Math.abs(sample.raw_pitch_channel_1_255 - prev.raw_pitch_channel_1_255) >= 3 ||
             Math.abs(sample.raw_throttle_channel_0_255 - prev.raw_throttle_channel_0_255) >= 3 ||
             Math.abs(sample.raw_yaw_channel_1_255 - prev.raw_yaw_channel_1_255) >= 3) {
    shouldLog = true;
    eventName = 'STICK_INPUT';
  } else if (Math.abs(sample.roll_deg - prev.roll_deg) >= 0.8 ||
             Math.abs(sample.pitch_deg - prev.pitch_deg) >= 0.8 ||
             Math.abs(sample.yaw_heading_deg - prev.yaw_heading_deg) >= 1.5) {
    shouldLog = true;
    eventName = 'ATTITUDE_CHANGE';
  } else if (Math.abs(sample.pos_x_east_m - prev.pos_x_east_m) >= 0.05 ||
             Math.abs(sample.pos_y_north_m - prev.pos_y_north_m) >= 0.05 ||
             Math.abs(sample.pos_z_alt_m - prev.pos_z_alt_m) >= 0.05) {
    shouldLog = true;
    eventName = 'POSITION_UPDATE';
  } else if (now - (prev.timestamp_epoch_ms || 0) >= (isAirborne ? 3000 : 8000)) {
    shouldLog = true;
    eventName = isAirborne ? 'FLIGHT_HEARTBEAT' : 'IDLE_HEARTBEAT';
  }

  if (shouldLog) {
    sample.flight_event = eventName;
    sample.is_airborne_state = isAirborne;
    sample.local_time = new Date(now).toLocaleTimeString('en-US', { hour12: false }) + '.' + String(now % 1000).padStart(3, '0');
    serverImuLogs.push(sample);
    if (serverImuLogs.length > 5000) {
      serverImuLogs.shift();
    }
  }
}, 100);

// Get IMU Logs (JSON)
app.get('/api/imu/logs', (req, res) => {
  res.json({
    total_samples: serverImuLogs.length,
    total_session_flight_time_s: sessionFlightTimeSeconds,
    current_flight_time_s: currentSortieTimeSeconds,
    current_coordinates: {
      x_east_m: serverPosX,
      y_north_m: serverPosY,
      z_alt_m: serverPosZ,
      origin: "Home (0,0,0)"
    },
    latest: serverImuLogs[serverImuLogs.length - 1] || null,
    samples: serverImuLogs.slice(-500)
  });
});

// Clear IMU Logs & Reset Coordinates
app.post('/api/imu/clear', (req, res) => {
  serverImuLogs.length = 0;
  serverPosX = 0.0;
  serverPosY = 0.0;
  serverPosZ = 0.0;
  serverTotalDistance = 0.0;
  res.json({ success: true, message: 'IMU logs & coordinates reset to (0,0,0)' });
});

// Export IMU Flight Logs as CSV
app.get('/api/imu/export-csv', (req, res) => {
  const now = new Date();
  const pad = (n, l = 2) => String(n).padStart(l, '0');
  const filename = `drone_imu_coordinates_flight_log_${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}.csv`;

  const headers = [
    `# RC UFO Drone Ground Control Station — 6-DOF IMU & 3D Coordinates Flight Log`,
    `# Export Timestamp: ${now.toISOString()}`,
    `# Reference Origin: Home (0.00m, 0.00m, 0.00m)`,
    `# Total Session Flight Time (s): ${sessionFlightTimeSeconds.toFixed(2)}`,
    `# Current Sortie Flight Time (s): ${currentSortieTimeSeconds.toFixed(2)}`,
    `# Total Distance Traveled (m): ${serverTotalDistance.toFixed(2)}`,
    `# Total Samples Logged: ${serverImuLogs.length}`,
    `# Hardware IMU: 6-Axis Gyroscope + 3-Axis Accelerometer + Altitude Hold Barometer`,
    `# =========================================================================`
  ];

  const columns = [
    'timestamp_iso',
    'timestamp_epoch_ms',
    'total_session_flight_time_s',
    'current_flight_time_s',
    'pos_x_east_m',
    'pos_y_north_m',
    'pos_z_alt_m',
    'distance_to_home_m',
    'total_distance_traveled_m',
    'roll_deg',
    'pitch_deg',
    'yaw_heading_deg',
    'accel_x_g',
    'accel_y_g',
    'accel_z_g',
    'total_g_load',
    'estimated_altitude_pct',
    'estimated_altitude_m',
    'raw_roll_channel_1_255',
    'raw_pitch_channel_1_255',
    'raw_throttle_channel_0_255',
    'raw_yaw_channel_1_255',
    'roll_trim',
    'pitch_trim',
    'yaw_trim',
    'speed_gear_pct',
    'altitude_hold_active',
    'headless_mode_active',
    'gyro_calibration_active',
    'stunt_flip_active',
    'protocol_type',
    'latency_ms',
    'packets_sent',
    'packets_received'
  ];

  const rows = (serverImuLogs.length > 0 ? serverImuLogs : []).map(s => [
    s.timestamp_iso,
    s.timestamp_epoch_ms,
    s.total_session_flight_time_s,
    s.current_flight_time_s,
    s.pos_x_east_m,
    s.pos_y_north_m,
    s.pos_z_alt_m,
    s.distance_to_home_m,
    s.total_distance_traveled_m,
    s.roll_deg,
    s.pitch_deg,
    s.yaw_heading_deg,
    s.accel_x_g,
    s.accel_y_g,
    s.accel_z_g,
    s.total_g_load,
    s.estimated_altitude_pct,
    s.estimated_altitude_m,
    s.raw_roll_channel_1_255,
    s.raw_pitch_channel_1_255,
    s.raw_throttle_channel_0_255,
    s.raw_yaw_channel_1_255,
    s.roll_trim,
    s.pitch_trim,
    s.yaw_trim,
    s.speed_gear_pct,
    s.altitude_hold_active,
    s.headless_mode_active,
    s.gyro_calibration_active,
    s.stunt_flip_active,
    s.protocol_type,
    s.latency_ms,
    s.packets_sent,
    s.packets_received
  ].join(','));

  const csvContent = [...headers, columns.join(','), ...rows].join('\r\n');

  res.setHeader('Content-Type', 'text/csv');
  res.setHeader('Content-Disposition', `attachment; filename="${filename}"`);
  res.send(csvContent);
});

server.listen(WEB_PORT, '0.0.0.0', () => {
  console.log('='.repeat(65));
  console.log('RC UFO DRONE GROUND CONTROL STATION - ACTIVE');
  console.log('='.repeat(65));
  console.log(`[*] Target Drone IP: ${droneState.droneIp}:${droneState.udpPort}`);
  console.log(`[*] RTSP Stream URL: ${droneState.rtspUrl}`);
  console.log(`[*] Web Cockpit HUD: http://localhost:${WEB_PORT}`);
  console.log(`[*] Frames Storage : ${FRAMES_DIR}`);
  console.log('='.repeat(65));
});
