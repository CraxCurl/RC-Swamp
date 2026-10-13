/**
 * AERO-LINK ESP32 Bridge Flight Control Ground Station — Frontend Controller
 * Synchronizes Real-Time Bluetooth Serial SPP / Wi-Fi UDP, Attitude Horizon Sphere,
 * FPV Video Streaming, Dual Virtual Joysticks, 6-DOF IMU Telemetry, Mission Runner, and Logs Export.
 */

document.addEventListener('DOMContentLoaded', () => {
  // ----------------- FLIGHT STATE ----------------- //
  const state = {
    // Flight channels (1..255, 128 = Neutral Center Hover)
    roll: 128,
    pitch: 128,
    throttle: 128,
    yaw: 128,

    // Status & Diagnostics
    connected: false,
    btConnected: false,
    hasLiveVideo: false,
    isAirborne: false,
    heartbeatCounter: 0,
    connectionMode: localStorage.getItem('drone_link_mode') || 'bluetooth', // 'bluetooth' | 'wifi'

    // Raw input axes (-1.0 .. +1.0)
    axisThrottle: 0,
    axisYaw: 0,
    axisPitch: 0,
    axisRoll: 0,

    // Endpoints
    droneIp: localStorage.getItem('drone_ip') || '192.168.4.1',
    udpPort: localStorage.getItem('drone_udp_port') || '8090',
    rtspUrl: localStorage.getItem('drone_rtsp_url') || 'http://192.168.4.1:8080/?action=stream',

    // Backend Telemetry Flags (updated by WebSocket relay)
    is_fast_fly: false,
    is_fast_drop: false,
    is_emergency_stop: false,
    is_gyro_correction: false,
    is_no_head_mode: false,
    is_fixed_height: true,
    gear: 2,
    packets_sent: 0,
    packets_received: 0,
    device_type: 'GL-21B'
  };

  // ----------------- GLOBAL INSTANCES ----------------- //
  window.droneImu = new DroneIMULogger();
  window.missionRunner = new MissionRunner();

  // ----------------- DOM ELEMENTS ----------------- //
  // Header Badges & Actions
  const btnBtStatusPill = document.getElementById('btnBtStatusPill');
  const btStatusText = document.getElementById('btStatusText');
  const droneUdpPill = document.getElementById('droneUdpPill');
  const droneUdpText = document.getElementById('droneUdpText');
  const heartbeatText = document.getElementById('heartbeatText');
  const btnEsp32Setup = document.getElementById('btnEsp32Setup');
  const btnConnectBt = document.getElementById('btnConnectBt');
  const btnExportExcelHeader = document.getElementById('btnExportExcelHeader');
  const btnZeroCoordsHeader = document.getElementById('btnZeroCoordsHeader');
  const btnImuStudio = document.getElementById('btnImuStudio');
  const btnMissions = document.getElementById('btnMissions');
  const btnGuide = document.getElementById('btnGuide');

  // Attitude & Telemetry Elements
  const attitudeCanvas = document.getElementById('attitudeCanvas');
  const armedStatusBadge = document.getElementById('armedStatusBadge');
  const dispThrottlePct = document.getElementById('dispThrottlePct');
  const dispThrottleRaw = document.getElementById('dispThrottleRaw');
  const dispPitchDeg = document.getElementById('dispPitchDeg');
  const dispPitchRaw = document.getElementById('dispPitchRaw');
  const dispRollDeg = document.getElementById('dispRollDeg');
  const dispRollRaw = document.getElementById('dispRollRaw');
  const dispYawDeg = document.getElementById('dispYawDeg');
  const dispYawRaw = document.getElementById('dispYawRaw');

  // Flight Commands
  const btnTakeoff = document.getElementById('btnTakeoff');
  const btnLand = document.getElementById('btnLand');
  const btnCalibrate = document.getElementById('btnCalibrate');
  const btnHandshake = document.getElementById('btnHandshake');
  const btnEmergency = document.getElementById('btnEmergency');

  // Video & FPV
  const videoCanvas = document.getElementById('videoCanvas');
  const standbyOverlay = document.getElementById('standbyOverlay');
  const standbyStreamUrl = document.getElementById('standbyStreamUrl');
  const btnOpenStream = document.getElementById('btnOpenStream');
  const selectCameraFeed = document.getElementById('selectCameraFeed');
  const btnRotateStream = document.getElementById('btnRotateStream');
  const btnCapturePhoto = document.getElementById('btnCapturePhoto');
  const btnRecordVideo = document.getElementById('btnRecordVideo');
  const videoFlashOverlay = document.getElementById('videoFlashOverlay');
  const rotateAngleLabel = document.getElementById('rotateAngleLabel');
  const hudPitchRollTag = document.getElementById('hudPitchRollTag');
  const hudThrTag = document.getElementById('hudThrTag');

  // Bluetooth Serial Terminal
  const serialTerminalBox = document.getElementById('serialTerminalBox');
  const termInput = document.getElementById('termInput');
  const btnTermSend = document.getElementById('btnTermSend');
  const btnClearTerminal = document.getElementById('btnClearTerminal');

  // Modals
  const connectionModal = document.getElementById('connectionModal');
  const btnCloseConnectionModal = document.getElementById('btnCloseConnectionModal');
  const cardWifiMode = document.getElementById('cardWifiMode');
  const cardBtMode = document.getElementById('cardBtMode');
  const btnSelectWifiMode = document.getElementById('btnSelectWifiMode');
  const btnSelectBtMode = document.getElementById('btnSelectBtMode');
  const btnConnectBtSerial = document.getElementById('btnConnectBtSerial');
  const esp32BtDot = document.getElementById('esp32BtDot');
  const esp32BtStatusText = document.getElementById('esp32BtStatusText');
  const esp32QuickTest = document.getElementById('esp32QuickTest');
  const btnBtTakeoff = document.getElementById('btnBtTakeoff');
  const btnBtLand = document.getElementById('btnBtLand');
  const btnBtStop = document.getElementById('btnBtStop');
  const btnBtPing = document.getElementById('btnBtPing');
  const btnOpenGuideFromConnModal = document.getElementById('btnOpenGuideFromConnModal');

  const settingsModal = document.getElementById('settingsModal');
  const btnCloseSettings = document.getElementById('btnCloseSettings');
  const btnSaveConfig = document.getElementById('btnSaveConfig');
  const cfgDroneIp = document.getElementById('cfgDroneIp');
  const cfgUdpPort = document.getElementById('cfgUdpPort');
  const cfgRtspUrl = document.getElementById('cfgRtspUrl');

  const imuModal = document.getElementById('imuModal');
  const btnCloseImu = document.getElementById('btnCloseImu');
  const btnOpenImuFromCockpit = document.getElementById('btnOpenImuFromCockpit');
  const btnCockpitExportCsv = document.getElementById('btnCockpitExportCsv');

  const missionModal = document.getElementById('mission-modal');
  const btnCloseMission = document.getElementById('btnCloseMission');
  const guideModal = document.getElementById('guideModal');
  const btnCloseGuide = document.getElementById('btnCloseGuide');

  // Web Serial Port for ESP32 Bluetooth
  let esp32Port = null;
  let esp32Writer = null;
  let serialReader = null;
  let isEsp32Connected = false;
  let telemetryWs = null;
  let videoWs = null;
  let latestFrameBitmap = null;
  let fallbackImg = new Image();
  let currentBlobUrl = null;
  let lastVideoPacketTime = Date.now();

  // ----------------- TERMINAL LOGGING ----------------- //
  function logTerminal(msg, type = 'normal') {
    if (!serialTerminalBox) return;
    const now = new Date();
    const timeStr = `${String(now.getHours()).padStart(2, '0')}:${String(now.getMinutes()).padStart(2, '0')}:${String(now.getSeconds()).padStart(2, '0')}.${String(now.getMilliseconds()).padStart(3, '0')}`;
    const line = document.createElement('div');
    line.className = `term-line ${type ? 'term-' + type : ''}`;
    line.textContent = `[${timeStr}] ${msg}`;
    serialTerminalBox.appendChild(line);

    // Limit buffer lines
    while (serialTerminalBox.childNodes.length > 200) {
      serialTerminalBox.removeChild(serialTerminalBox.firstChild);
    }
    serialTerminalBox.scrollTop = serialTerminalBox.scrollHeight;
  }

  if (btnClearTerminal) {
    btnClearTerminal.addEventListener('click', () => {
      serialTerminalBox.innerHTML = '<div class="term-line term-dim">Terminal buffer cleared. Ready for stream.</div>';
    });
  }

  let isConnectingSerial = false;

  async function disconnectEsp32() {
    try {
      isEsp32Connected = false;
      state.btConnected = false;
      if (serialReader) {
        try { await serialReader.cancel(); } catch (e) {}
        try { serialReader.releaseLock(); } catch (e) {}
        serialReader = null;
      }
      if (esp32Writer) {
        try { await esp32Writer.close(); } catch (e) {}
        esp32Writer = null;
      }
      if (esp32Port) {
        try { await esp32Port.close(); } catch (e) {}
        esp32Port = null;
      }
      if (btnBtStatusPill) {
        btnBtStatusPill.className = 'pill-badge pill-red';
      }
      if (btStatusText) btStatusText.textContent = 'BT DISCONNECTED';
      if (btnConnectBt) btnConnectBt.innerHTML = '<svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><path d="M6.5 6.5l11 11L12 23V1l5.5 5.5-11 11"/></svg><span>CONNECT ESP32 (BT)</span>';
      updateBtUIModal(false, 'Disconnected');
      logTerminal('ESP32 Disconnected.', 'dim');
    } catch (e) {
      logTerminal(`Disconnect note: ${e.message}`, 'dim');
    }
  }

  // ----------------- ESP32 BLUETOOTH / USB SERIAL BRIDGE ----------------- //
  async function connectEsp32Bluetooth() {
    if (!('serial' in navigator)) {
      logTerminal('Web Serial API not supported in this browser. Use Chrome, Edge, or Opera.', 'err');
      alert('Please use Google Chrome, Microsoft Edge, or Opera to connect directly to the ESP32 via Bluetooth / USB Serial.');
      return;
    }

    // If already connected, clicking toggles disconnect
    if (isEsp32Connected && esp32Port) {
      disconnectEsp32();
      return;
    }

    // Prevent concurrent open() calls
    if (isConnectingSerial) {
      logTerminal('A connection attempt is already in progress. Please wait a moment...', 'warn');
      return;
    }

    isConnectingSerial = true;
    try {
      if (esp32Port) {
        try { await esp32Port.close(); } catch (e) {}
      }

      logTerminal('Requesting ESP32 Serial / Bluetooth port...', 'dim');
      logTerminal('👉 RECOMMENDED: Select "CP210x USB to UART Bridge (COM9)" for instant high-speed USB link!', 'ack');
      logTerminal('👉 (Or select "Standard Serial (COM11)" if paired over Bluetooth).', 'dim');
      esp32Port = await navigator.serial.requestPort();
      
      logTerminal('Opening port at 115200 baud...', 'dim');
      const openPromise = esp32Port.open({ baudRate: 115200 });
      const timeoutPromise = new Promise((_, reject) => 
        setTimeout(() => reject(new Error('Connection timed out after 5 seconds. If you selected a Bluetooth port, make sure ESP32_Drone is paired, OR select "CP210x USB to UART Bridge (COM9)" to connect instantly over USB cable!')), 5000)
      );
      await Promise.race([openPromise, timeoutPromise]);

      esp32Writer = esp32Port.writable.getWriter();
      isEsp32Connected = true;
      state.btConnected = true;

      // Update UI Header
      if (btnBtStatusPill) {
        btnBtStatusPill.className = 'pill-badge pill-green';
      }
      if (btStatusText) btStatusText.textContent = 'ESP32 CONNECTED';
      if (btnConnectBt) btnConnectBt.innerHTML = '<span>DISCONNECT ESP32</span>';

      logTerminal('✅ ESP32 Connected successfully! (115200 Baud)', 'ack');
      updateBtUIModal(true, 'ESP32 Connected (115200 Baud)');
      readEsp32SerialStream(esp32Port);

      // Send initial status ping
      sendEsp32Command('P\n');
    } catch (err) {
      console.warn('ESP32 serial error:', err);
      if (err.name !== 'NotFoundError') {
        logTerminal(`Connection failed: ${err.message}`, 'err');
        if (err.message && (err.message.includes('Failed to open') || err.message.includes('Access denied'))) {
          logTerminal('💡 Check 1: Close Arduino IDE Serial Monitor (it locks COM9 exclusively).', 'warn');
          logTerminal('💡 Check 2: Select "CP210x USB to UART Bridge (COM9)" in the popup instead of Bluetooth.', 'warn');
          logTerminal('💡 Check 3: Press EN/RST button on ESP32 board to reset connection.', 'warn');
        }
        updateBtUIModal(false, `Error: ${err.message}`);
      }
    } finally {
      isConnectingSerial = false;
    }
  }

  async function readEsp32SerialStream(port) {
    if (!port || !port.readable) return;
    try {
      serialReader = port.readable.getReader();
    } catch (e) {
      logTerminal(`Error acquiring serial reader: ${e.message}`, 'err');
      return;
    }

    let rxBuffer = new Uint8Array(0);
    const textDecoder = new TextDecoder('utf-8');

    function appendChunk(buf1, buf2) {
      const merged = new Uint8Array(buf1.length + buf2.length);
      merged.set(buf1, 0);
      merged.set(buf2, buf1.length);
      return merged;
    }

    function processTextString(str) {
      const lines = str.split('\n');
      for (const l of lines) {
        const trimmed = l.trim();
        if (trimmed) {
          if (trimmed.startsWith('ACK:')) logTerminal(trimmed, 'ack');
          else if (trimmed.startsWith('STATUS:')) logTerminal(trimmed, 'out');
          else if (trimmed.startsWith('[VIDEO]') || trimmed.startsWith('[WIFI]') || trimmed.startsWith('[BT]')) logTerminal(trimmed, 'ack');
          else if (trimmed.includes('FAIL') || trimmed.includes('Error')) logTerminal(trimmed, 'err');
          else logTerminal(trimmed, 'out');
        }
      }
    }

    try {
      while (isEsp32Connected && serialReader) {
        const { value, done } = await serialReader.read();
        if (done) break;
        if (!value || value.length === 0) continue;

        rxBuffer = appendChunk(rxBuffer, value);

        // State machine to parse both binary video frames and ASCII text lines
        while (rxBuffer.length > 0) {
          // Look for 4-byte magic sync header: 0xDE, 0xAD, 0xBE, 0xEF
          let magicIdx = -1;
          for (let i = 0; i <= rxBuffer.length - 4; i++) {
            if (rxBuffer[i] === 0xDE && rxBuffer[i+1] === 0xAD && rxBuffer[i+2] === 0xBE && rxBuffer[i+3] === 0xEF) {
              magicIdx = i;
              break;
            }
          }

          if (magicIdx === -1) {
            // No magic header found. Check if there are any newline characters
            const lastNl = rxBuffer.lastIndexOf(10); // '\n' = 10
            if (lastNl !== -1) {
              const textBytes = rxBuffer.subarray(0, lastNl + 1);
              rxBuffer = rxBuffer.subarray(lastNl + 1);
              processTextString(textDecoder.decode(textBytes));
            } else if (rxBuffer.length > 4096) {
              // Safety flush
              rxBuffer = new Uint8Array(0);
            }
            break; // Await more incoming data
          }

          // If magic header is found at index > 0, the bytes before it are text
          if (magicIdx > 0) {
            const textBytes = rxBuffer.subarray(0, magicIdx);
            rxBuffer = rxBuffer.subarray(magicIdx);
            processTextString(textDecoder.decode(textBytes));
          }

          // Magic header is now at index 0. Need at least 8 bytes for magic + length
          if (rxBuffer.length < 8) {
            break; // Await more data
          }

          const frameLen = ((rxBuffer[4] << 24) >>> 0) | (rxBuffer[5] << 16) | (rxBuffer[6] << 8) | rxBuffer[7];

          // Sanity check frame length (typical JPEG is 200 bytes .. 64KB)
          if (frameLen <= 0 || frameLen > 100000) {
            // Corrupted / desynced length: discard leading byte and re-scan
            rxBuffer = rxBuffer.subarray(1);
            continue;
          }

          // Check if full JPEG payload is present
          if (rxBuffer.length < 8 + frameLen) {
            break; // Await rest of JPEG bytes
          }

          // Complete frame available!
          const jpegBytes = rxBuffer.slice(8, 8 + frameLen);
          rxBuffer = rxBuffer.subarray(8 + frameLen);

          // Render live frame
          await renderBtVideoFrame(jpegBytes);
        }
      }
    } catch (e) {
      logTerminal(`Serial read stream note: ${e.message}`, 'warn');
    } finally {
      if (serialReader) {
        try { serialReader.releaseLock(); } catch (err) {}
        serialReader = null;
      }
    }
  }

  async function renderBtVideoFrame(jpegBytes) {
    if (!jpegBytes || jpegBytes.length < 100) return;
    try {
      const blob = new Blob([jpegBytes], { type: 'image/jpeg' });
      if (window.createImageBitmap) {
        try {
          const bmp = await createImageBitmap(blob);
          if (latestFrameBitmap && latestFrameBitmap.close) {
            latestFrameBitmap.close();
          }
          latestFrameBitmap = bmp;
          lastVideoPacketTime = Date.now();
          state.hasLiveVideo = true;
          if (standbyOverlay && !standbyOverlay.classList.contains('hide')) {
            standbyOverlay.classList.add('hide');
          }
          drawVideoFrame();
        } catch (e) {
          loadFallbackImage(blob);
        }
      } else {
        loadFallbackImage(blob);
      }

      // Also forward frame to Node.js backend so disk saving, auto-capture & snapshot APIs work
      if (videoWs && videoWs.readyState === WebSocket.OPEN && videoWs.bufferedAmount < 65536) {
        videoWs.send(jpegBytes);
      }
    } catch (err) {
      console.warn('renderBtVideoFrame error:', err);
    }
  }

  function loadFallbackImage(blob) {
    try {
      const oldUrl = currentBlobUrl;
      currentBlobUrl = URL.createObjectURL(blob);
      fallbackImg.onload = () => {
        lastVideoPacketTime = Date.now();
        state.hasLiveVideo = true;
        if (standbyOverlay && !standbyOverlay.classList.contains('hide')) {
          standbyOverlay.classList.add('hide');
        }
        drawVideoFrame();
        if (oldUrl) URL.revokeObjectURL(oldUrl);
      };
      fallbackImg.src = currentBlobUrl;
    } catch (e) {}
  }

  let isSendingSerial = false;
  let lastSendErrorLogged = 0;

  async function sendEsp32Command(data, priority = false) {
    if (!esp32Writer || !isEsp32Connected) return;

    if (isSendingSerial) {
      if (!priority) return; // Drop high-frequency stick packets if writer is busy
      let waitCount = 0;
      while (isSendingSerial && waitCount < 8) {
        await new Promise(r => setTimeout(r, 20));
        waitCount++;
      }
      if (isSendingSerial) return;
    }

    isSendingSerial = true;
    try {
      if (esp32Writer.ready) {
        await esp32Writer.ready;
      }
      if (typeof data === 'string') {
        const encoder = new TextEncoder();
        await esp32Writer.write(encoder.encode(data));
      } else if (data instanceof Uint8Array) {
        await esp32Writer.write(data);
      }
    } catch (e) {
      const now = Date.now();
      // Throttle error notifications so terminal doesn't get flooded
      if (now - lastSendErrorLogged > 3500) {
        lastSendErrorLogged = now;
        logTerminal(`Send Error: ${e.message}`, 'err');
      }

      // Automatically recover from errored stream writer
      try {
        if (esp32Writer) {
          esp32Writer.releaseLock();
          esp32Writer = null;
        }
      } catch (err) {}

      if (esp32Port && esp32Port.writable) {
        try {
          esp32Writer = esp32Port.writable.getWriter();
        } catch (err) {
          disconnectEsp32();
        }
      } else {
        disconnectEsp32();
      }
    } finally {
      isSendingSerial = false;
    }
  }

  function updateBtUIModal(connected, text) {
    if (esp32BtDot) esp32BtDot.className = 'v-bt-dot ' + (connected ? 'connected' : 'error');
    if (esp32BtStatusText) esp32BtStatusText.textContent = text;
    if (esp32QuickTest) {
      if (connected) esp32QuickTest.classList.remove('hide');
      else esp32QuickTest.classList.add('hide');
    }
  }

  if (btnConnectBt) btnConnectBt.addEventListener('click', connectEsp32Bluetooth);
  if (btnBtStatusPill) btnBtStatusPill.addEventListener('click', connectEsp32Bluetooth);
  if (btnConnectBtSerial) btnConnectBtSerial.addEventListener('click', connectEsp32Bluetooth);

  // Terminal Send Action
  function submitTermInput() {
    const val = termInput.value.trim();
    if (!val) return;
    logTerminal(`TX > ${val}`, 'normal');
    sendEsp32Command(val + '\n');
    termInput.value = '';
  }

  if (btnTermSend) btnTermSend.addEventListener('click', submitTermInput);
  if (termInput) {
    termInput.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') submitTermInput();
    });
  }

  // ----------------- WEBSOCKET BACKEND RELAY & AUTO-RECONNECT ----------------- //
  let telemetryReconnectTimer = null;
  let videoReconnectTimer = null;
  lastVideoPacketTime = Date.now();

  function initSockets() {
    const loc = window.location;
    const wsProto = loc.protocol === 'https:' ? 'wss:' : 'ws:';

    function connectTelemetry() {
      clearTimeout(telemetryReconnectTimer);
      try {
        telemetryWs = new WebSocket(`${wsProto}//${loc.host}/ws/telemetry`);
        telemetryWs.onopen = () => {
          logTerminal('Telemetry WebSocket linked', 'dim');
        };
        telemetryWs.onmessage = (event) => {
          try {
            const msg = JSON.parse(event.data);
            if (msg.type === 'telemetry') {
              state.connected = !!msg.connected;
              if (droneUdpPill) {
                droneUdpPill.className = state.connected ? 'pill-badge pill-green' : 'pill-badge pill-darkblue';
              }
              if (droneUdpText) {
                droneUdpText.textContent = `DRONE UDP :${msg.udp_port || '8090'} (${state.connected ? 'ONLINE' : 'STANDBY'})`;
              }

              // Store backend telemetry flags in state
              if (msg.is_fast_fly) state.is_fast_fly = true;
              state.is_fast_drop = !!msg.is_fast_drop;
              state.is_emergency_stop = !!msg.is_emergency_stop;
              state.is_gyro_correction = !!msg.is_gyro_correction;
              state.is_no_head_mode = !!msg.is_no_head_mode;
              state.is_fixed_height = msg.is_fixed_height !== undefined ? !!msg.is_fixed_height : true;
              state.gear = msg.gear || 2;
              state.packets_sent = msg.packets_sent || 0;
              state.packets_received = msg.packets_received || 0;
              state.device_type = msg.device_type || 'GL-21B';

              // If user is not currently active on keys/joysticks, sync stick values from backend
              if (state.axisThrottle === 0 && state.axisYaw === 0 && state.axisPitch === 0 && state.axisRoll === 0 && !window.missionRunner?.isRunning) {
                if (typeof msg.roll === 'number') state.roll = msg.roll;
                if (typeof msg.pitch === 'number') state.pitch = msg.pitch;
                if (typeof msg.throttle === 'number') state.throttle = msg.throttle;
                if (typeof msg.yaw === 'number') state.yaw = msg.yaw;
              }

              if (msg.hardware_imu && window.droneImu) {
                window.droneImu.injectHardwareTelemetry(msg.hardware_imu);
              }
            }
          } catch (e) {}
        };
        telemetryWs.onclose = () => {
          telemetryReconnectTimer = setTimeout(connectTelemetry, 1500);
        };
        telemetryWs.onerror = () => {
          try { telemetryWs.close(); } catch (e) {}
        };
      } catch (e) {
        telemetryReconnectTimer = setTimeout(connectTelemetry, 2000);
      }
    }

    function connectVideo() {
      clearTimeout(videoReconnectTimer);
      try {
        videoWs = new WebSocket(`${wsProto}//${loc.host}/ws/video`);
        videoWs.binaryType = 'blob';
        videoWs.onopen = () => {
          logTerminal('Live Video WebSocket linked', 'dim');
        };
        videoWs.onmessage = async (event) => {
          if (activeVideoSource === 'webcam' || activeVideoSource === 'drone_bluetooth') return;

          if (window.createImageBitmap) {
            try {
              const bmp = await createImageBitmap(event.data);
              if (latestFrameBitmap && latestFrameBitmap.close) {
                latestFrameBitmap.close();
              }
              latestFrameBitmap = bmp;
              lastVideoPacketTime = Date.now();
              state.hasLiveVideo = true;
              if (standbyOverlay && !standbyOverlay.classList.contains('hide')) {
                standbyOverlay.classList.add('hide');
              }
              drawVideoFrame();
              return;
            } catch (err) {}
          }

          // Fallback to Image element
          try {
            const oldUrl = currentBlobUrl;
            currentBlobUrl = URL.createObjectURL(event.data);
            fallbackImg.onload = () => {
              lastVideoPacketTime = Date.now();
              state.hasLiveVideo = true;
              if (standbyOverlay && !standbyOverlay.classList.contains('hide')) {
                standbyOverlay.classList.add('hide');
              }
              drawVideoFrame();
              if (oldUrl) URL.revokeObjectURL(oldUrl);
            };
            fallbackImg.src = currentBlobUrl;
          } catch (e) {}
        };
        videoWs.onclose = () => {
          videoReconnectTimer = setTimeout(connectVideo, 1200);
        };
        videoWs.onerror = () => {
          try { videoWs.close(); } catch (e) {}
        };
      } catch (e) {
        videoReconnectTimer = setTimeout(connectVideo, 2000);
      }
    }

    connectTelemetry();
    connectVideo();
  }

  // ----------------- VIDEO FEED & ROTATION ENGINE ----------------- //
  let videoRotation = parseInt(localStorage.getItem('drone_video_rotation') || '0', 10) || 0;
  let activeVideoSource = localStorage.getItem('drone_video_source') || 'drone_bluetooth';
  let webcamStream = null;
  let webcamVideoElement = null;
  let webcamAnimFrame = null;

  let fpvTick = 0;

  function drawVideoFrame() {
    if (!videoCanvas) return;
    fpvTick += 0.016;

    try {
      const hasLiveStream = latestFrameBitmap && (Date.now() - lastVideoPacketTime < 2500);
      const isWebcam = (activeVideoSource === 'webcam' && webcamVideoElement && webcamVideoElement.readyState >= 2);

      // If activeVideoSource is drone_bluetooth and no live frames arriving yet, render clean standby screen
      if (activeVideoSource === 'drone_bluetooth' && !hasLiveStream) {
        if (videoCanvas.width !== 640 || videoCanvas.height !== 360) {
          videoCanvas.width = 640;
          videoCanvas.height = 360;
        }
        const ctx = videoCanvas.getContext('2d');
        ctx.fillStyle = '#080c14';
        ctx.fillRect(0, 0, 640, 360);

        // Tech grid
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
        ctx.lineWidth = 1;
        for (let x = 40; x < 640; x += 40) {
          ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, 360); ctx.stroke();
        }
        for (let y = 30; y < 360; y += 30) {
          ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(640, y); ctx.stroke();
        }

        // Reticle
        ctx.strokeStyle = 'rgba(6, 182, 212, 0.4)';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(320, 150, 26, 0, Math.PI * 2);
        ctx.stroke();

        ctx.fillStyle = '#06b6d4';
        ctx.font = '600 13px Inter, monospace';
        ctx.textAlign = 'center';
        ctx.fillText('ESP32 BLUETOOTH FPV — STANDBY', 320, 195);

        ctx.fillStyle = 'rgba(255, 255, 255, 0.6)';
        ctx.font = '11.5px Inter, sans-serif';
        ctx.fillText('Awaiting Drone Wi-Fi Connection...', 320, 220);

        ctx.fillStyle = 'rgba(255, 255, 255, 0.35)';
        ctx.font = '10.5px Inter, sans-serif';
        ctx.fillText('Power ON your drone, or type SSID:<name> in terminal', 320, 240);
        return;
      }

      // If activeVideoSource is synthetic OR no live camera frames arriving, render high-fidelity synthetic FPV HUD
      if (activeVideoSource === 'synthetic' || (!hasLiveStream && !isWebcam)) {
        if (videoCanvas.width !== 640 || videoCanvas.height !== 360) {
          videoCanvas.width = 640;
          videoCanvas.height = 360;
        }
        DroneHUD.renderSyntheticFPV(videoCanvas, state, window.droneImu?.currentIMU, fpvTick);
        return;
      }

      const ctx = videoCanvas.getContext('2d');
      const source = isWebcam ? webcamVideoElement : (latestFrameBitmap || fallbackImg);
      if (!source) {
        DroneHUD.renderSyntheticFPV(videoCanvas, state, window.droneImu?.currentIMU, fpvTick);
        return;
      }

      const rawW = source.videoWidth || source.naturalWidth || source.width || 1280;
      const rawH = source.videoHeight || source.naturalHeight || source.height || 720;
      if (rawW <= 0 || rawH <= 0) {
        DroneHUD.renderSyntheticFPV(videoCanvas, state, window.droneImu?.currentIMU, fpvTick);
        return;
      }

      const isRotated90 = (videoRotation === 90 || videoRotation === 270);
      const canvasW = isRotated90 ? rawH : rawW;
      const canvasH = isRotated90 ? rawW : rawH;

      if (videoCanvas.width !== canvasW || videoCanvas.height !== canvasH) {
        videoCanvas.width = canvasW;
        videoCanvas.height = canvasH;
      }

      ctx.save();
      ctx.imageSmoothingEnabled = true;
      ctx.imageSmoothingQuality = 'medium';

      if (videoRotation === 0) {
        ctx.drawImage(source, 0, 0, canvasW, canvasH);
      } else {
        ctx.fillStyle = '#000000';
        ctx.fillRect(0, 0, canvasW, canvasH);
        ctx.translate(canvasW / 2, canvasH / 2);
        ctx.rotate((videoRotation * Math.PI) / 180);
        ctx.drawImage(source, -rawW / 2, -rawH / 2, rawW, rawH);
      }
      ctx.restore();
    } catch (err) {
      console.warn('drawVideoFrame error:', err);
      try {
        DroneHUD.renderSyntheticFPV(videoCanvas, state, window.droneImu?.currentIMU, fpvTick);
      } catch (e) {}
    }
  }

  // Continuous 60 FPS Video & Synthetic HUD Render Loop
  function videoLoop() {
    drawVideoFrame();
    requestAnimationFrame(videoLoop);
  }
  requestAnimationFrame(videoLoop);

  async function startWebcamFeed() {
    stopWebcamFeed();
    try {
      webcamVideoElement = document.createElement('video');
      webcamVideoElement.setAttribute('autoplay', '');
      webcamVideoElement.setAttribute('muted', '');
      webcamVideoElement.setAttribute('playsinline', '');
      
      webcamStream = await navigator.mediaDevices.getUserMedia({
        video: { width: { ideal: 1280 }, height: { ideal: 720 } }
      });
      webcamVideoElement.srcObject = webcamStream;
      await webcamVideoElement.play();

      state.hasLiveVideo = true;
      if (standbyOverlay) standbyOverlay.classList.add('hide');

      function loop() {
        if (activeVideoSource === 'webcam' && webcamVideoElement && webcamVideoElement.readyState >= 2) {
          drawVideoFrame();
          webcamAnimFrame = requestAnimationFrame(loop);
        }
      }
      loop();
      logTerminal('Local Webcam / USB FPV Receiver active', 'ack');
    } catch (err) {
      logTerminal(`Camera access error: ${err.message}`, 'err');
      alert(`Could not start local camera stream: ${err.message}`);
      if (selectCameraFeed) selectCameraFeed.value = 'drone_rtsp';
      activeVideoSource = 'drone_rtsp';
      localStorage.setItem('drone_video_source', 'drone_rtsp');
    }
  }

  function stopWebcamFeed() {
    if (webcamAnimFrame) {
      cancelAnimationFrame(webcamAnimFrame);
      webcamAnimFrame = null;
    }
    if (webcamStream) {
      webcamStream.getTracks().forEach(t => t.stop());
      webcamStream = null;
    }
    if (webcamVideoElement) {
      webcamVideoElement.srcObject = null;
      webcamVideoElement = null;
    }
  }

  async function applyCameraSource(sourceKey, customUrl = null) {
    activeVideoSource = sourceKey;
    localStorage.setItem('drone_video_source', sourceKey);
    if (selectCameraFeed) selectCameraFeed.value = sourceKey;

    if (sourceKey === 'webcam') {
      startWebcamFeed();
      if (standbyStreamUrl) standbyStreamUrl.textContent = 'Active Feed: Local Browser Webcam / USB FPV Dongle';
      return;
    }

    // Stop webcam when using network feeds
    stopWebcamFeed();

    let targetUrl = '';
    if (sourceKey === 'drone_bluetooth') {
      targetUrl = 'bluetooth://ESP32_Drone';
      if (standbyStreamUrl) standbyStreamUrl.textContent = 'Active Feed: ESP32 FPV Video Stream (USB COM9 / Bluetooth)';
    } else if (sourceKey === 'drone_rtsp') {
      targetUrl = 'rtsp://192.168.1.1:7070/webcam';
      if (standbyStreamUrl) standbyStreamUrl.textContent = 'Active Feed: Direct Drone RTSP (Requires Laptop on Drone Wi-Fi)';
    } else if (sourceKey === 'drone_http') {
      targetUrl = 'http://192.168.1.1:8080/?action=stream';
      if (standbyStreamUrl) standbyStreamUrl.textContent = 'Active Feed: Drone HTTP MJPEG (Port 8080)';
    } else if (sourceKey === 'synthetic') {
      targetUrl = 'synthetic';
    } else if (sourceKey === 'custom') {
      targetUrl = customUrl || localStorage.getItem('drone_rtsp_url') || 'rtsp://192.168.1.1:7070/webcam';
    }

    state.rtspUrl = targetUrl;
    localStorage.setItem('drone_rtsp_url', targetUrl);
    if (standbyStreamUrl) standbyStreamUrl.textContent = `Target Feed: ${targetUrl}`;

    // Dispatch to Node.js Backend over Telemetry WS and HTTP Config
    if (telemetryWs && telemetryWs.readyState === WebSocket.OPEN) {
      telemetryWs.send(JSON.stringify({ rtsp_url: targetUrl }));
    }
    fetch('/api/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ rtsp_url: targetUrl })
    }).catch(() => {});

    logTerminal(`Camera feed switched to: ${sourceKey} (${targetUrl})`, 'ack');
  }

  initSockets();

  // ----------------- UNIFIED FLIGHT COMMAND DISPATCHER ----------------- //
  let lastBtStickTime = 0;
  let lastBtRoll = 128, lastBtPitch = 128, lastBtThrottle = 128, lastBtYaw = 128;

  function dispatchFlightCommand(payload) {
    if (!payload) return;

    // Update internal state from dispatched flight commands (supports live simulation & manual controls)
    if (payload.action === 'takeoff') {
      state.isAirborne = true;
      if (window.droneImu) window.droneImu.setAirborne(true);
      if (armedStatusBadge) {
        armedStatusBadge.className = 'badge-armed';
        armedStatusBadge.textContent = 'ARMED / FLYING';
      }
    } else if (payload.action === 'land') {
      state.isAirborne = false;
      if (window.droneImu) window.droneImu.setAirborne(false);
      if (armedStatusBadge) {
        armedStatusBadge.className = 'badge-disarmed';
        armedStatusBadge.textContent = 'DISARMED';
      }
    } else if (payload.action === 'stick') {
      if (typeof payload.roll === 'number') state.roll = payload.roll;
      if (typeof payload.pitch === 'number') state.pitch = payload.pitch;
      if (typeof payload.throttle === 'number') state.throttle = payload.throttle;
      if (typeof payload.yaw === 'number') state.yaw = payload.yaw;
    }

    // 1. Send via Bluetooth Serial to ESP32 Bridge
    if (isEsp32Connected && esp32Writer) {
      if (payload.action === 'takeoff') {
        sendEsp32Command('T\n', true);
        logTerminal('TX > TAKEOFF [T]', 'ack');
      } else if (payload.action === 'land') {
        sendEsp32Command('L\n', true);
        logTerminal('TX > LAND [L]', 'ack');
      } else if (payload.action === 'emergency_stop') {
        sendEsp32Command('S\n', true);
        logTerminal('TX > EMERGENCY STOP [S]', 'err');
      } else if (payload.action === 'calibrate_gyro') {
        sendEsp32Command('G\n', true);
        logTerminal('TX > GYRO CALIBRATION [G]', 'ack');
      } else if (payload.action === 'handshake') {
        sendEsp32Command('H\n', true);
        logTerminal('TX > HANDSHAKE [H]', 'out');
      } else if (payload.action === 'stick') {
        const r = payload.roll !== undefined ? payload.roll : 128;
        const p = payload.pitch !== undefined ? payload.pitch : 128;
        const t = payload.throttle !== undefined ? payload.throttle : 128;
        const y = payload.yaw !== undefined ? payload.yaw : 128;

        const now = Date.now();
        const stickChanged = (r !== lastBtRoll || p !== lastBtPitch || t !== lastBtThrottle || y !== lastBtYaw);
        
        // Rate-limit stick frames over Bluetooth: send when sticks change, or 5 Hz heartbeat
        if (stickChanged || (now - lastBtStickTime >= 200)) {
          lastBtStickTime = now;
          lastBtRoll = r;
          lastBtPitch = p;
          lastBtThrottle = t;
          lastBtYaw = y;
          const cs = (r ^ p ^ t ^ y) & 0xFF;
          const pkt = new Uint8Array([0x66, r, p, t, y, 0x00, cs, 0x99]);
          sendEsp32Command(pkt, false);
        }
      }
    }

    // 2. Relay to Node.js Backend UDP Socket
    if (telemetryWs && telemetryWs.readyState === WebSocket.OPEN) {
      telemetryWs.send(JSON.stringify(payload));
    }
  }

  // Expose globally for MissionRunner
  window.droneDispatchFlightCommand = dispatchFlightCommand;

  // ----------------- 25Hz ACTIVE FLIGHT LOOP & TELEMETRY UI ----------------- //
  setInterval(() => {
    state.heartbeatCounter++;
    if (heartbeatText) {
      heartbeatText.textContent = `25Hz HEARTBEAT #${state.heartbeatCounter % 1000}`;
    }

    // Compute raw values
    const deadzone = (v) => Math.abs(v) < 0.05 ? 0 : v;
    const cleanThrottle = deadzone(state.axisThrottle);
    const cleanYaw = deadzone(state.axisYaw);
    const cleanPitch = deadzone(state.axisPitch);
    const cleanRoll = deadzone(state.axisRoll);

    if (!window.missionRunner?.isRunning) {
      // Dynamic speed gear scaling according to protocol spec: Gear 1 (30%): 40, Gear 2 (60%): 60, Gear 3 (100%): 127
      const halfLen = state.gear === 1 ? 40 : state.gear === 3 ? 127 : 60;
      state.roll = Math.max(1, Math.min(255, Math.round(128 + (cleanRoll * halfLen))));
      state.pitch = Math.max(1, Math.min(255, Math.round(128 + (cleanPitch * halfLen))));
      state.throttle = Math.max(0, Math.min(255, Math.round(128 + (cleanThrottle * 127))));
      state.yaw = Math.max(1, Math.min(255, Math.round(128 + (cleanYaw * 127))));

      // Send continuous 25Hz stick frame
      dispatchFlightCommand({
        action: 'stick',
        roll: state.roll,
        pitch: state.pitch,
        throttle: state.throttle,
        yaw: state.yaw
      });
    }

    // Dynamic flight state detection for manual control, simulation, and autonomous missions:
    const isMovingStick = (Math.abs(cleanPitch) > 0.05 || Math.abs(cleanRoll) > 0.05 || Math.abs(cleanYaw) > 0.05);
    const isApplyingThrottle = (cleanThrottle > 0.05 || state.throttle > 135);

    if ((state.is_fast_fly || isApplyingThrottle || window.missionRunner?.isRunning) && !state.is_fast_drop && !state.is_emergency_stop) {
      if (!state.isAirborne) {
        state.isAirborne = true;
        if (window.droneImu) window.droneImu.setAirborne(true);
      }
    } else if (state.is_fast_drop || state.is_emergency_stop || (state.axisThrottle < -0.85 && state.throttle <= 20)) {
      if (state.isAirborne) {
        state.isAirborne = false;
        if (window.droneImu) window.droneImu.setAirborne(false);
      }
    }

    if (armedStatusBadge) {
      if (state.isAirborne) {
        armedStatusBadge.className = 'badge-armed';
        armedStatusBadge.textContent = 'ARMED / FLYING';
      } else {
        armedStatusBadge.className = 'badge-disarmed';
        armedStatusBadge.textContent = 'DISARMED';
      }
    }

    // Feed state to DroneIMULogger (authoritative 25Hz physics source)
    if (window.droneImu) {
      window.droneImu.update({
        roll: state.roll,
        pitch: state.pitch,
        throttle: state.throttle,
        yaw: state.yaw,
        is_airborne: state.isAirborne,
        is_fast_fly: state.is_fast_fly,
        is_fast_drop: state.is_fast_drop,
        is_emergency_stop: state.is_emergency_stop,
        is_gyro_correction: state.is_gyro_correction,
        is_no_head_mode: state.is_no_head_mode,
        is_fixed_height: state.is_fixed_height,
        gear: state.gear,
        packets_sent: state.packets_sent,
        packets_received: state.packets_received,
        device_type: state.device_type
      });
    }

    // Update UI Metrics from physical IMU state
    const thrPct = Math.round((state.throttle / 255) * 100);
    const rollAngle = window.droneImu ? window.droneImu.currentIMU.rollDeg : parseFloat(((state.roll - 128) * (28.5 / 127.0)).toFixed(1));
    const pitchAngle = window.droneImu ? window.droneImu.currentIMU.pitchDeg : parseFloat(((state.pitch - 128) * (28.5 / 127.0)).toFixed(1));
    const yawHeading = window.droneImu ? window.droneImu.currentIMU.yawDeg : Math.round((state.yaw - 128) * (180.0 / 127.0));

    if (dispThrottlePct) dispThrottlePct.textContent = `${thrPct}%`;
    if (dispThrottleRaw) dispThrottleRaw.textContent = `RAW: ${state.throttle}`;
    if (dispPitchDeg) dispPitchDeg.textContent = `${pitchAngle > 0 ? '+' : ''}${pitchAngle.toFixed(1)}°`;
    if (dispPitchRaw) dispPitchRaw.textContent = `RAW: ${state.pitch}`;
    if (dispRollDeg) dispRollDeg.textContent = `${rollAngle > 0 ? '+' : ''}${rollAngle.toFixed(1)}°`;
    if (dispRollRaw) dispRollRaw.textContent = `RAW: ${state.roll}`;
    if (dispYawDeg) dispYawDeg.textContent = `${yawHeading.toFixed(1)}°`;
    if (dispYawRaw) dispYawRaw.textContent = `RAW: ${state.yaw}`;

    if (hudPitchRollTag) hudPitchRollTag.textContent = `PITCH: ${pitchAngle.toFixed(1)}° | ROLL: ${rollAngle.toFixed(1)}°`;
    if (hudThrTag) hudThrTag.textContent = `THR: ${thrPct}%`;

    // Render Artificial Horizon Sphere
    DroneHUD.renderAttitudeSphere(attitudeCanvas, rollAngle, pitchAngle);
  }, 40);

  // ----------------- ACTION BUTTON EVENTS ----------------- //
  function triggerTakeoff() {
    state.isAirborne = true;
    if (armedStatusBadge) {
      armedStatusBadge.className = 'badge-armed';
      armedStatusBadge.textContent = 'ARMED / FLYING';
    }
    if (window.droneImu) window.droneImu.setAirborne(true);
    dispatchFlightCommand({ action: 'takeoff' });
  }

  function triggerLand() {
    state.isAirborne = false;
    if (armedStatusBadge) {
      armedStatusBadge.className = 'badge-disarmed';
      armedStatusBadge.textContent = 'DISARMED';
    }
    if (window.droneImu) window.droneImu.setAirborne(false);
    dispatchFlightCommand({ action: 'land' });
  }

  function triggerEmergency() {
    state.isAirborne = false;
    state.throttle = 0;
    if (armedStatusBadge) {
      armedStatusBadge.className = 'badge-disarmed';
      armedStatusBadge.textContent = 'DISARMED (KILL)';
    }
    if (window.droneImu) window.droneImu.setAirborne(false);
    dispatchFlightCommand({ action: 'emergency_stop' });
  }

  if (btnTakeoff) btnTakeoff.addEventListener('click', triggerTakeoff);
  if (btnLand) btnLand.addEventListener('click', triggerLand);
  if (btnEmergency) btnEmergency.addEventListener('click', triggerEmergency);
  if (btnCalibrate) btnCalibrate.addEventListener('click', () => dispatchFlightCommand({ action: 'calibrate_gyro' }));
  if (btnHandshake) btnHandshake.addEventListener('click', () => dispatchFlightCommand({ action: 'handshake' }));

  if (btnBtTakeoff) btnBtTakeoff.addEventListener('click', triggerTakeoff);
  if (btnBtLand) btnBtLand.addEventListener('click', triggerLand);
  if (btnBtStop) btnBtStop.addEventListener('click', triggerEmergency);
  if (btnBtPing) btnBtPing.addEventListener('click', () => sendEsp32Command('P\n'));

  // ----------------- VIRTUAL JOYSTICKS ----------------- //
  new VirtualJoystick('leftJoystickWrapper', 'leftJoystickNipple', (x, y) => {
    state.axisYaw = x;
    state.axisThrottle = y;
  });

  new VirtualJoystick('rightJoystickWrapper', 'rightJoystickNipple', (x, y) => {
    state.axisRoll = x;
    state.axisPitch = y;
  });

  // ----------------- KEYBOARD FLIGHT CONTROLS ----------------- //
  const keyState = {};
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
    keyState[e.code] = true;

    if (e.code === 'KeyT') triggerTakeoff();
    else if (e.code === 'KeyL') triggerLand();
    else if (e.code === 'Space' || e.code === 'Escape') triggerEmergency();
    else if (e.code === 'KeyC') dispatchFlightCommand({ action: 'calibrate_gyro' });
    else if (e.code === 'KeyH') dispatchFlightCommand({ action: 'handshake' });
    else if (e.code === 'KeyI' && imuModal) imuModal.classList.toggle('hide');
    else if (e.code === 'KeyM' && missionModal) missionModal.classList.toggle('hide');
    else if (e.code === 'KeyG' && guideModal) guideModal.classList.toggle('hide');
    else if (e.code === 'KeyE' && window.droneImu) window.droneImu.downloadCSV();

    processKeyboardAxes();
  });

  window.addEventListener('keyup', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
    keyState[e.code] = false;
    processKeyboardAxes();
  });

  function processKeyboardAxes() {
    let kt = 0, ky = 0, kp = 0, kr = 0;
    if (keyState['KeyW']) kt += 1.0;
    if (keyState['KeyS']) kt -= 1.0;
    if (keyState['KeyA']) ky -= 1.0;
    if (keyState['KeyD']) ky += 1.0;

    if (keyState['ArrowUp']) kp += 1.0;
    if (keyState['ArrowDown']) kp -= 1.0;
    if (keyState['ArrowLeft']) kr -= 1.0;
    if (keyState['ArrowRight']) kr += 1.0;

    state.axisThrottle = kt;
    state.axisYaw = ky;
    state.axisPitch = kp;
    state.axisRoll = kr;
  }

  // ----------------- MODALS MANAGEMENT ----------------- //
  if (btnEsp32Setup) btnEsp32Setup.addEventListener('click', () => connectionModal.classList.remove('hide'));
  if (btnCloseConnectionModal) btnCloseConnectionModal.addEventListener('click', () => connectionModal.classList.add('hide'));

  if (btnSelectWifiMode) {
    btnSelectWifiMode.addEventListener('click', () => {
      state.connectionMode = 'wifi';
      localStorage.setItem('drone_link_mode', 'wifi');
      cardWifiMode.classList.add('active');
      cardBtMode.classList.remove('active');
      connectionModal.classList.add('hide');
      logTerminal('Switched to Direct Wi-Fi Connection Mode', 'normal');
    });
  }

  if (btnSelectBtMode) {
    btnSelectBtMode.addEventListener('click', () => {
      state.connectionMode = 'bluetooth';
      localStorage.setItem('drone_link_mode', 'bluetooth');
      cardBtMode.classList.add('active');
      cardWifiMode.classList.remove('active');
      connectionModal.classList.add('hide');
      logTerminal('Switched to ESP32 Bluetooth Bridge Mode', 'normal');
    });
  }

  if (btnImuStudio && imuModal) btnImuStudio.addEventListener('click', () => imuModal.classList.remove('hide'));
  if (btnCloseImu && imuModal) btnCloseImu.addEventListener('click', () => imuModal.classList.add('hide'));
  if (btnOpenImuFromCockpit && imuModal) btnOpenImuFromCockpit.addEventListener('click', () => imuModal.classList.remove('hide'));
  if (btnCockpitExportCsv && window.droneImu) btnCockpitExportCsv.addEventListener('click', () => window.droneImu.downloadCSV('current'));
  if (btnExportExcelHeader && window.droneImu) btnExportExcelHeader.addEventListener('click', () => window.droneImu.downloadCSV('current'));
  
  const btnCockpitResetLogs = document.getElementById('btnCockpitResetLogs');
  if (btnCockpitResetLogs && window.droneImu) {
    btnCockpitResetLogs.addEventListener('click', () => {
      window.droneImu.clearLogs();
      logTerminal('Flight log buffer wiped and reset to 0.0', 'success');
    });
  }

  if (btnMissions && missionModal) btnMissions.addEventListener('click', () => missionModal.classList.remove('hide'));
  if (btnCloseMission && missionModal) btnCloseMission.addEventListener('click', () => missionModal.classList.add('hide'));

  if (btnGuide && guideModal) btnGuide.addEventListener('click', () => guideModal.classList.remove('hide'));
  if (btnCloseGuide && guideModal) btnCloseGuide.addEventListener('click', () => guideModal.classList.add('hide'));

  if (btnOpenStream) {
    btnOpenStream.addEventListener('click', () => {
      if (standbyOverlay) standbyOverlay.classList.toggle('hide');
    });
  }

  // Snapshot Photo Capture Handler
  function capturePhoto() {
    if (!videoCanvas) return;
    try {
      if (videoFlashOverlay) {
        videoFlashOverlay.classList.add('flash');
        setTimeout(() => videoFlashOverlay.classList.remove('flash'), 150);
      }

      const dataUrl = videoCanvas.toDataURL('image/jpeg', 0.95);
      const link = document.createElement('a');
      const now = new Date();
      const pad = (n, l = 2) => String(n).padStart(l, '0');
      const ts = `${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
      link.href = dataUrl;
      link.download = `drone_snapshot_${ts}.jpg`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      logTerminal(`Photo snapshot captured: drone_snapshot_${ts}.jpg`, 'ack');
    } catch (e) {
      logTerminal(`Snapshot capture error: ${e.message}`, 'err');
    }
  }

  if (btnCapturePhoto) btnCapturePhoto.addEventListener('click', capturePhoto);

  // Video Recording Handler
  let mediaRecorder = null;
  let recordedChunks = [];
  let isRecordingVideo = false;

  function toggleVideoRecording() {
    if (!videoCanvas) return;

    if (!isRecordingVideo) {
      try {
        const stream = videoCanvas.captureStream(30);
        recordedChunks = [];
        const mimeType = MediaRecorder.isTypeSupported('video/webm;codecs=vp9') ? 'video/webm;codecs=vp9' : 'video/webm';
        mediaRecorder = new MediaRecorder(stream, { mimeType });

        mediaRecorder.ondataavailable = (event) => {
          if (event.data && event.data.size > 0) {
            recordedChunks.push(event.data);
          }
        };

        mediaRecorder.onstop = () => {
          const blob = new Blob(recordedChunks, { type: 'video/webm' });
          const url = URL.createObjectURL(blob);
          const link = document.createElement('a');
          const now = new Date();
          const pad = (n, l = 2) => String(n).padStart(l, '0');
          const ts = `${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
          link.href = url;
          link.download = `drone_flight_video_${ts}.webm`;
          document.body.appendChild(link);
          link.click();
          document.body.removeChild(link);
          URL.revokeObjectURL(url);
          logTerminal(`Video recording saved: drone_flight_video_${ts}.webm`, 'ack');
        };

        mediaRecorder.start(250);
        isRecordingVideo = true;
        if (btnRecordVideo) {
          btnRecordVideo.classList.add('recording');
          btnRecordVideo.setAttribute('title', 'Stop Recording Video (Recording Active)');
        }
        logTerminal('Video recording started [LIVE]', 'ack');
      } catch (err) {
        logTerminal(`Could not start video recording: ${err.message}`, 'err');
      }
    } else {
      if (mediaRecorder && mediaRecorder.state !== 'inactive') {
        mediaRecorder.stop();
      }
      isRecordingVideo = false;
      if (btnRecordVideo) {
        btnRecordVideo.classList.remove('recording');
        btnRecordVideo.setAttribute('title', 'Start / Stop Video Recording');
      }
    }
  }

  if (btnRecordVideo) btnRecordVideo.addEventListener('click', toggleVideoRecording);

  // Video Rotation Button Handler
  if (rotateAngleLabel) rotateAngleLabel.textContent = `${videoRotation}°`;
  if (btnRotateStream) {
    btnRotateStream.addEventListener('click', () => {
      videoRotation = (videoRotation + 90) % 360;
      localStorage.setItem('drone_video_rotation', videoRotation);
      if (rotateAngleLabel) rotateAngleLabel.textContent = `${videoRotation}°`;
      btnRotateStream.setAttribute('title', `Rotate Video 90° Clockwise (Current: ${videoRotation}°)`);
      drawVideoFrame();
      logTerminal(`Camera stream rotated to ${videoRotation}°`, 'ack');
    });
  }

  // Dual Camera Lens Toggle Handler (Front vs Bottom Downward Camera)
  const btnToggleCameraLens = document.getElementById('btnToggleCameraLens');
  const cameraLensLabel = document.getElementById('cameraLensLabel');
  let currentCameraId = 1; // 1 = Front FPV, 2 = Bottom Downward Optical Flow

  if (btnToggleCameraLens) {
    btnToggleCameraLens.addEventListener('click', () => {
      currentCameraId = currentCameraId === 1 ? 2 : 1;
      const isBottom = currentCameraId === 2;
      if (cameraLensLabel) {
        cameraLensLabel.textContent = isBottom ? 'BOTTOM CAM' : 'FRONT CAM';
      }
      btnToggleCameraLens.className = isBottom ? 'btn-slate btn-highlight btn-xs' : 'btn-slate btn-xs';

      // Send UDP [0x06, camera_id] command to drone via backend
      dispatchFlightCommand({ action: 'switch_camera', camera_id: currentCameraId });
      logTerminal(`Switched to: ${isBottom ? 'BOTTOM (Downward Facing Camera)' : 'FRONT (Primary FPV Camera)'}`, 'ack');
    });
  }

  // Camera Feed Selector Handler
  if (selectCameraFeed) {
    selectCameraFeed.value = activeVideoSource;
    selectCameraFeed.addEventListener('change', (e) => {
      const chosen = e.target.value;
      if (chosen === 'custom') {
        const prev = localStorage.getItem('drone_rtsp_url') || state.rtspUrl || 'rtsp://192.168.1.1:7070/webcam';
        const entered = prompt('Enter Custom Video Stream URL (RTSP or HTTP):', prev);
        if (entered && entered.trim()) {
          applyCameraSource('custom', entered.trim());
        } else {
          selectCameraFeed.value = activeVideoSource;
        }
      } else {
        applyCameraSource(chosen);
      }
    });
  }

  // Initial startup for active video source if webcam was persisted
  if (activeVideoSource === 'webcam') {
    startWebcamFeed();
  }

  window.addEventListener('resize', () => {
    drawVideoFrame();
  });

  if (btnSaveConfig) {
    btnSaveConfig.addEventListener('click', () => {
      state.droneIp = cfgDroneIp.value.trim();
      state.udpPort = cfgUdpPort.value.trim();
      state.rtspUrl = cfgRtspUrl.value.trim();
      localStorage.setItem('drone_ip', state.droneIp);
      localStorage.setItem('drone_udp_port', state.udpPort);
      localStorage.setItem('drone_rtsp_url', state.rtspUrl);
      logTerminal(`Settings updated: Drone IP ${state.droneIp}:${state.udpPort}`, 'ack');
      settingsModal.classList.add('hide');
    });
  }

  // Resize canvas initially
  if (attitudeCanvas) {
    attitudeCanvas.width = 220;
    attitudeCanvas.height = 220;
  }
});
