/**
 * AERO-LINK ESP32 Bridge Flight Control Ground Station — Frontend Controller
 * Synchronizes Real-Time Bluetooth Serial SPP / Wi-Fi UDP, Attitude Horizon Sphere,
 * FPV Video Streaming, Dual Virtual Joysticks, and Real-Time Terminal Logs.
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
    rtspUrl: localStorage.getItem('drone_rtsp_url') || 'http://192.168.4.1:8080/?action=stream'
  };

  // ----------------- DOM ELEMENTS ----------------- //
  // Header Badges & Actions
  const btnBtStatusPill = document.getElementById('btnBtStatusPill');
  const btStatusText = document.getElementById('btStatusText');
  const droneUdpPill = document.getElementById('droneUdpPill');
  const droneUdpText = document.getElementById('droneUdpText');
  const heartbeatText = document.getElementById('heartbeatText');
  const btnEsp32Setup = document.getElementById('btnEsp32Setup');
  const btnConnectBt = document.getElementById('btnConnectBt');
  const btnSwitchMeeting = document.getElementById('btnSwitchMeeting');
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
  const missionModal = document.getElementById('mission-modal');
  const btnCloseMission = document.getElementById('btnCloseMission');
  const guideModal = document.getElementById('guideModal');
  const btnCloseGuide = document.getElementById('btnCloseGuide');

  // Web Serial Port for ESP32 Bluetooth
  let esp32Port = null;
  let esp32Writer = null;
  let isEsp32Connected = false;
  let telemetryWs = null;
  let videoWs = null;
  let latestDroneBitmap = null;

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

  // ----------------- ESP32 BLUETOOTH SERIAL BRIDGE ----------------- //
  async function connectEsp32Bluetooth() {
    if (!('serial' in navigator)) {
      logTerminal('Web Serial API not supported in this browser. Use Chrome, Edge, or Opera.', 'err');
      alert('Please use Google Chrome, Microsoft Edge, or Opera to connect directly to the ESP32 via Bluetooth / USB Serial.');
      return;
    }

    try {
      if (esp32Port) {
        try { await esp32Port.close(); } catch (e) {}
      }

      logTerminal('Requesting Bluetooth / Serial COM port...', 'dim');
      esp32Port = await navigator.serial.requestPort();
      await esp32Port.open({ baudRate: 115200 });
      esp32Writer = esp32Port.writable.getWriter();
      isEsp32Connected = true;
      state.btConnected = true;

      // Update UI Header
      if (btnBtStatusPill) {
        btnBtStatusPill.className = 'pill-badge pill-green';
      }
      if (btStatusText) btStatusText.textContent = 'BT CONNECTED';
      if (btnConnectBt) btnConnectBt.innerHTML = '<span>ESP32 LINKED</span>';

      logTerminal('ESP32_Drone Connected via Bluetooth SPP (115200 Baud)', 'ack');
      updateBtUIModal(true, 'ESP32_Drone Connected (115200 Baud)');
      readEsp32SerialStream(esp32Port);

      // Send initial status ping
      sendEsp32Command('P\n');
    } catch (err) {
      console.warn('Bluetooth serial error:', err);
      if (err.name !== 'NotFoundError') {
        logTerminal(`Connection failed: ${err.message}`, 'err');
        updateBtUIModal(false, `Error: ${err.message}`);
      }
    }
  }

  async function readEsp32SerialStream(port) {
    const textDecoder = new TextDecoderStream();
    port.readable.pipeTo(textDecoder.writable).catch(() => {});
    const reader = textDecoder.readable.getReader();

    try {
      while (isEsp32Connected) {
        const { value, done } = await reader.read();
        if (done) break;
        if (value) {
          const lines = value.split('\n');
          lines.forEach((l) => {
            const trimmed = l.trim();
            if (trimmed) {
              if (trimmed.startsWith('ACK:')) logTerminal(trimmed, 'ack');
              else if (trimmed.startsWith('STATUS:')) logTerminal(trimmed, 'out');
              else if (trimmed.includes('FAIL') || trimmed.includes('Error')) logTerminal(trimmed, 'err');
              else logTerminal(trimmed, 'out');
            }
          });
        }
      }
    } catch (e) {
      logTerminal(`Serial read stream closed: ${e.message}`, 'warn');
    } finally {
      reader.releaseLock();
    }
  }

  async function sendEsp32Command(data) {
    if (!esp32Writer || !isEsp32Connected) return;
    try {
      if (typeof data === 'string') {
        const encoder = new TextEncoder();
        await esp32Writer.write(encoder.encode(data));
      } else if (data instanceof Uint8Array) {
        await esp32Writer.write(data);
      }
    } catch (e) {
      logTerminal(`Send Error: ${e.message}`, 'err');
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

  // ----------------- WEBSOCKET BACKEND RELAY ----------------- //
  function initSockets() {
    const loc = window.location;
    const wsProto = loc.protocol === 'https:' ? 'wss:' : 'ws:';
    telemetryWs = new WebSocket(`${wsProto}//${loc.host}/ws/telemetry`);

    telemetryWs.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === 'telemetry') {
          state.connected = !!msg.connected;
          if (droneUdpText && msg.drone_ip) {
            droneUdpText.textContent = `DRONE UDP :${msg.udp_port || '8090'}`;
          }
        }
      } catch (e) {}
    };

    videoWs = new WebSocket(`${wsProto}//${loc.host}/ws/video`);
    videoWs.binaryType = 'arraybuffer';
    videoWs.onmessage = (event) => {
      const blob = new Blob([event.data], { type: 'image/jpeg' });
      createImageBitmap(blob).then((bm) => {
        const old = latestDroneBitmap;
        latestDroneBitmap = bm;
        if (old) old.close();
        state.hasLiveVideo = true;
        if (standbyOverlay) standbyOverlay.classList.add('hide');
        drawVideoFrame();
      }).catch(() => {});
    };
  }

  function drawVideoFrame() {
    if (!videoCanvas || !latestDroneBitmap) return;
    const ctx = videoCanvas.getContext('2d');
    videoCanvas.width = videoCanvas.parentElement.clientWidth;
    videoCanvas.height = videoCanvas.parentElement.clientHeight;
    ctx.drawImage(latestDroneBitmap, 0, 0, videoCanvas.width, videoCanvas.height);
  }

  initSockets();

  // ----------------- UNIFIED FLIGHT COMMAND DISPATCHER ----------------- //
  function dispatchFlightCommand(payload) {
    // 1. Send via Bluetooth Serial to ESP32 Bridge
    if (isEsp32Connected && esp32Writer) {
      if (payload.action === 'takeoff') {
        sendEsp32Command('T\n');
        logTerminal('TX > TAKEOFF [T]', 'ack');
      } else if (payload.action === 'land') {
        sendEsp32Command('L\n');
        logTerminal('TX > LAND [L]', 'ack');
      } else if (payload.action === 'emergency_stop') {
        sendEsp32Command('S\n');
        logTerminal('TX > EMERGENCY STOP [S]', 'err');
      } else if (payload.action === 'calibrate_gyro') {
        sendEsp32Command('G\n');
        logTerminal('TX > GYRO CALIBRATION [G]', 'ack');
      } else if (payload.action === 'handshake') {
        sendEsp32Command('H\n');
        logTerminal('TX > HANDSHAKE [H]', 'out');
      } else if (payload.action === 'stick') {
        // Standard E88 8-Byte Packet: [0x66, roll, pitch, throttle, yaw, cmd, checksum, 0x99]
        const r = payload.roll || 128;
        const p = payload.pitch || 128;
        const t = payload.throttle || 128;
        const y = payload.yaw || 128;
        const cs = (r ^ p ^ t ^ y) & 0xFF;
        const pkt = new Uint8Array([0x66, r, p, t, y, 0x00, cs, 0x99]);
        sendEsp32Command(pkt);
      }
    }

    // 2. Relay to Node.js Backend UDP Socket
    if (telemetryWs && telemetryWs.readyState === WebSocket.OPEN) {
      telemetryWs.send(JSON.stringify(payload));
    }
  }

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

    state.roll = Math.max(1, Math.min(255, Math.round(128 + (cleanRoll * 60))));
    state.pitch = Math.max(1, Math.min(255, Math.round(128 + (cleanPitch * 60))));
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

    // Update UI Metrics
    const thrPct = Math.round((state.throttle / 255) * 100);
    const rollAngle = Math.round((state.roll - 128) * 0.35);
    const pitchAngle = Math.round((state.pitch - 128) * 0.25);
    const yawAngle = Math.round((state.yaw - 128) * 1.4);

    if (dispThrottlePct) dispThrottlePct.textContent = `${thrPct}%`;
    if (dispThrottleRaw) dispThrottleRaw.textContent = `RAW: ${state.throttle}`;
    if (dispPitchDeg) dispPitchDeg.textContent = `${pitchAngle}°`;
    if (dispPitchRaw) dispPitchRaw.textContent = `RAW: ${state.pitch}`;
    if (dispRollDeg) dispRollDeg.textContent = `${rollAngle}°`;
    if (dispRollRaw) dispRollRaw.textContent = `RAW: ${state.roll}`;
    if (dispYawDeg) dispYawDeg.textContent = `${yawAngle}°`;
    if (dispYawRaw) dispYawRaw.textContent = `RAW: ${state.yaw}`;

    if (hudPitchRollTag) hudPitchRollTag.textContent = `PITCH: ${pitchAngle}° | ROLL: ${rollAngle}°`;
    if (hudThrTag) hudThrTag.textContent = `THR: ${thrPct}%`;

    // Render Artificial Horizon Sphere
    DroneHUD.renderAttitudeSphere(attitudeCanvas, rollAngle, -pitchAngle);
  }, 40);

  // ----------------- ACTION BUTTON EVENTS ----------------- //
  function triggerTakeoff() {
    state.isAirborne = true;
    if (armedStatusBadge) {
      armedStatusBadge.className = 'badge-armed';
      armedStatusBadge.textContent = 'ARMED / FLYING';
    }
    dispatchFlightCommand({ action: 'takeoff' });
  }

  function triggerLand() {
    state.isAirborne = false;
    if (armedStatusBadge) {
      armedStatusBadge.className = 'badge-disarmed';
      armedStatusBadge.textContent = 'DISARMED';
    }
    dispatchFlightCommand({ action: 'land' });
  }

  function triggerEmergency() {
    state.isAirborne = false;
    state.throttle = 0;
    if (armedStatusBadge) {
      armedStatusBadge.className = 'badge-disarmed';
      armedStatusBadge.textContent = 'DISARMED (KILL)';
    }
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

  if (btnMissions && missionModal) btnMissions.addEventListener('click', () => missionModal.classList.remove('hide'));
  if (btnCloseMission && missionModal) btnCloseMission.addEventListener('click', () => missionModal.classList.add('hide'));

  if (btnGuide && guideModal) btnGuide.addEventListener('click', () => guideModal.classList.remove('hide'));
  if (btnCloseGuide && guideModal) btnCloseGuide.addEventListener('click', () => guideModal.classList.add('hide'));

  if (btnOpenStream) {
    btnOpenStream.addEventListener('click', () => {
      if (standbyOverlay) standbyOverlay.classList.toggle('hide');
    });
  }

  if (btnSwitchMeeting) {
    btnSwitchMeeting.addEventListener('click', () => {
      if (missionModal) missionModal.classList.remove('hide');
    });
  }

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
