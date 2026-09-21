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
  lastRtspTime: 0,
  lastUdpVideoTime: 0,

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

function broadcastVideoFrame(buffer, source = 'STREAM') {
  if (!buffer || buffer.length < 100) return;
  const now = Date.now();

  const isRtsp = source === 'PyAV-RTSP';
  const selectedCam = droneState.cameraId || 1;

  // Manual camera stream lock: change stream ONLY when user manually switches lens
  if (selectedCam === 1) {
    if (!isRtsp && droneState.lastRtspTime && (now - droneState.lastRtspTime < 3000)) {
      return; // Lock to Front RTSP stream when active
    }
    if (isRtsp) droneState.lastRtspTime = now;
  } else if (selectedCam === 2) {
    if (isRtsp && droneState.lastUdpVideoTime && (now - droneState.lastUdpVideoTime < 3000)) {
      return; // Lock to Bottom UDP stream when active
    }
    if (!isRtsp) droneState.lastUdpVideoTime = now;
  }

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
    // Zero-lag queueing: Send if client socket buffer is under 128 KB backpressure limit
    if (client.readyState === WebSocket.OPEN && client.bufferedAmount < 128 * 1024) {
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
    pyAvRestartTimer = setTimeout(startPyAvRtspRelay, 1500);
  });
}

startPyAvRtspRelay();

// ----------------- 2. Fragmented Multi-Packet UDP MJPEG Reassembler ----------------- //
class UdpMjpegReassembler {
  constructor(name) {
    this.name = name;
    this.chunks = [];
    this.totalBytes = 0;
    this.inFrame = false;
    this.lastPacketTime = 0;
  }

  feedPacket(buffer) {
    const now = Date.now();
    if (this.inFrame && now - this.lastPacketTime > 250) {
      this.reset();
    }
    this.lastPacketTime = now;

    const soiIndex = buffer.indexOf(Buffer.from([0xFF, 0xD8]));
    const eoiIndex = buffer.indexOf(Buffer.from([0xFF, 0xD9]));

    if (soiIndex !== -1 && eoiIndex !== -1 && eoiIndex > soiIndex) {
      const singleFrame = buffer.subarray(soiIndex, eoiIndex + 2);
      this.reset();
      broadcastVideoFrame(singleFrame, `UDP-Single-${this.name}`);
      return;
    }

    if (soiIndex !== -1) {
      this.reset();
      this.inFrame = true;
      const startSlice = buffer.subarray(soiIndex);
      this.chunks.push(startSlice);
      this.totalBytes += startSlice.length;
      return;
    }

    if (this.inFrame) {
      if (eoiIndex !== -1) {
        const endSlice = buffer.subarray(0, eoiIndex + 2);
        this.chunks.push(endSlice);
        this.totalBytes += endSlice.length;

        if (this.totalBytes > 1000 && this.totalBytes < 2 * 1024 * 1024) {
          const completeFrame = Buffer.concat(this.chunks, this.totalBytes);
          broadcastVideoFrame(completeFrame, `UDP-Reassembled-${this.name}`);
        }
        this.reset();
      } else {
        this.chunks.push(buffer);
        this.totalBytes += buffer.length;
        if (this.totalBytes > 2 * 1024 * 1024) {
          this.reset();
        }
      }
    }
  }

  reset() {
    this.chunks = [];
    this.totalBytes = 0;
    this.inFrame = false;
  }
}

const controlSocketReassembler = new UdpMjpegReassembler('7099');

const videoUdpPorts = [7099, 7098, 7070, 7060, 8888, 8080, 8554, 9000, 50000, 5000];
videoUdpPorts.forEach(port => {
  try {
    const reasm = new UdpMjpegReassembler(port.toString());
    const sock = dgram.createSocket({ type: 'udp4', reuseAddr: true });
    sock.on('error', () => {});
    sock.on('message', (msg) => {
      reasm.feedPacket(msg);
    });
    sock.bind(port, '0.0.0.0', () => {});
  } catch (e) {}
});

// ----------------- 3. UDP Control Socket & Protocol ----------------- //
const udpClient = dgram.createSocket({ type: 'udp4', reuseAddr: true });

udpClient.on('error', (err) => {
  console.warn('[UDP Warning]', err.message);
});

udpClient.on('message', (msg) => {
  droneState.packetsReceived++;
  droneState.connected = true;
  controlSocketReassembler.feedPacket(msg);
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

// 1 Hz Heartbeat & Camera Sensor Wakeup
setInterval(() => {
  sendDroneUdp(Buffer.from([0x01, 0x01]));
  sendDroneUdp(Buffer.from([0x06, droneState.cameraId || 0x01]));
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

  controlSocketReassembler.feedPacket(msg);

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

function triggerFlip360(direction = 'right') {
  const dir = (direction || 'right').toLowerCase();
  droneState.isFlipping = true;
  droneState.isCircleTurnEnd = true;

  if (dir === 'left') droneState.roll = 1;
  else if (dir === 'right') droneState.roll = 255;
  else if (dir === 'forward') droneState.pitch = 255;
  else if (dir === 'backward') droneState.pitch = 1;
  else droneState.pitch = 255;

  sendDroneUdp(Buffer.from([0x07, 0x01]));
  sendDroneUdp(Buffer.from([0x08, 0x01]));
  sendFlightPacketBurst(5);

  setTimeout(() => {
    droneState.isFlipping = false;
    droneState.isCircleTurnEnd = false;
    droneState.roll = 128;
    droneState.pitch = 128;
  }, 800);
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
        droneState.isFastFly = true;
        droneState.isFastDrop = false;
        droneState.isEmergencyStop = false;
        sendFlightPacketBurst(3);
        setTimeout(() => { droneState.isFastFly = false; }, 1500);
      } else if (msg.action === 'land') {
        droneState.isFastDrop = true;
        droneState.isFastFly = false;
        sendFlightPacketBurst(3);
        setTimeout(() => { droneState.isFastDrop = false; }, 1500);
      } else if (msg.action === 'emergency_stop') {
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
        droneState.cameraId = msg.camera_id;
        droneState.lastRtspTime = 0;
        droneState.lastUdpVideoTime = 0;
        sendDroneUdp(Buffer.from([0x06, msg.camera_id]));
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
    droneState.lastRtspTime = 0;
    droneState.lastUdpVideoTime = 0;
    sendDroneUdp(Buffer.from([0x06, camId]));
  }

  if (take_off) {
    droneState.isFastFly = true;
    setTimeout(() => { droneState.isFastFly = false; }, 400);
  }
  if (land) {
    droneState.isFastDrop = true;
    setTimeout(() => { droneState.isFastDrop = false; }, 400);
  }
  if (emergency) {
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

server.listen(WEB_PORT, '0.0.0.0', () => {
  console.log('='.repeat(65));
  console.log('🛸 RC UFO DRONE GROUND CONTROL STATION - ACTIVE');
  console.log('='.repeat(65));
  console.log(`[*] Target Drone IP: ${droneState.droneIp}:${droneState.udpPort}`);
  console.log(`[*] RTSP Stream URL: ${droneState.rtspUrl}`);
  console.log(`[*] Web Cockpit HUD: http://localhost:${WEB_PORT}`);
  console.log(`[*] Frames Storage : ${FRAMES_DIR}`);
  console.log('='.repeat(65));
});
