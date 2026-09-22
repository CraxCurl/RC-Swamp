/**
 * RC UFO Drone Ground Control Station — Vercel Cockpit Application
 * Coordinates continuous WebSocket video stream, telemetry sync, gamepad/keyboard inputs, HUD, controls guide, and image processing studio.
 */

document.addEventListener('DOMContentLoaded', () => {
  // ----------------- FLIGHT STATE ----------------- //
  const state = {
    // Flight channels (1..255, 128 neutral center hover)
    roll: 128,
    pitch: 128,
    throttle: 128,
    yaw: 128,

    // Trims & Modes
    rollTrim: 0,
    pitchTrim: 0,
    yawTrim: 0,
    gear: 2,           // 1: 30% (Slow/Indoor), 2: 60% (Medium/Cruising), 3: 100% (Sport)
    halfLen: 60,       // Stick deflection scale
    trimMultiplier: 3, // Multiplier based on gear
    
    // Status
    connected: false,
    hasLiveVideo: false,
    videoSource: 'NONE',
    deviceType: 'GL-21B',
    fps: 0,
    packetsSent: 0,
    packetsReceived: 0,
    voiceEnabled: true,
    cameraId: 1,       // 1: Front, 2: Bottom
    isRecording: false,
    isNoHeadMode: false,
    isFixedHeight: true, // Altitude Hold & Optical Flow Lock

    // Raw input axes (-1.0 .. +1.0)
    axisThrottle: 0,
    axisYaw: 0,
    axisPitch: 0,
    axisRoll: 0,

    // Image Processing & Auto-Capture
    savedFramesCount: 0,
    autoCapture: false,
    autoCaptureFps: 2,

    // Camera Transformations
    cameraRotation: parseInt(localStorage.getItem('drone_cam_rotation') || '0', 10), // 0, 90, 180, 270
    cameraInvert: localStorage.getItem('drone_cam_invert') === 'true', // Vertical Flip (Invert)
    cameraMirror: localStorage.getItem('drone_cam_mirror') === 'true'  // Horizontal Flip (Mirror)
  };

  // ----------------- DOM ELEMENTS ----------------- //
  const videoCanvas = document.getElementById('videoCanvas');
  const videoCtx = videoCanvas.getContext('2d');
  const standbyOverlay = document.getElementById('standbyOverlay');
  
  // Status Bar
  const connectionStatus = document.getElementById('connectionStatus');
  const connectionText = document.getElementById('connectionText');
  const protocolBadge = document.getElementById('protocolBadge');
  const fpsValue = document.getElementById('fpsValue');
  const latencyValue = document.getElementById('latencyValue');
  const savedFramesStat = document.getElementById('savedFramesStat');
  const gearBadge = document.getElementById('gearBadge');
  
  // HUD Overlays
  const throttleBar = document.getElementById('throttleBar');
  const throttleVal = document.getElementById('throttleVal');
  const pitchBar = document.getElementById('pitchBar');
  const pitchVal = document.getElementById('pitchVal');
  const hudToast = document.getElementById('hudToast');
  const gamepadIndicator = document.getElementById('gamepadIndicator');
  
  // Trims & Modes
  const trimYawVal = document.getElementById('trimYawVal');
  const trimRollVal = document.getElementById('trimRollVal');
  const trimPitchVal = document.getElementById('trimPitchVal');
  const headlessStatus = document.getElementById('headlessStatus');
  const altHoldStatus = document.getElementById('altHoldStatus');
  const camIdText = document.getElementById('camIdText');
  const recDot = document.getElementById('recDot');
  
  // Action Buttons
  const btnTakeoff = document.getElementById('btnTakeoff');
  const btnLand = document.getElementById('btnLand');
  const btnFlipMenu = document.getElementById('btnFlipMenu');
  const flipDropdown = document.getElementById('flipDropdown');
  const btnAltHold = document.getElementById('btnAltHold');
  const btnHeadless = document.getElementById('btnHeadless');
  const btnCamSwitch = document.getElementById('btnCamSwitch');
  const btnRotateFeed = document.getElementById('btnRotateFeed');
  const rotationValText = document.getElementById('rotationValText');
  const btnInvertFeed = document.getElementById('btnInvertFeed');
  const invertStatus = document.getElementById('invertStatus');
  const btnMirrorFeed = document.getElementById('btnMirrorFeed');
  const mirrorStatus = document.getElementById('mirrorStatus');
  const btnGyro = document.getElementById('btnGyro');
  const btnEmergency = document.getElementById('btnEmergency');

  // Header Buttons
  const btnMissions = document.getElementById('btnMissions');
  const btnImagesStudio = document.getElementById('btnImagesStudio');
  const btnGuide = document.getElementById('btnGuide');
  const btnSnapshot = document.getElementById('btnSnapshot');
  const btnRecord = document.getElementById('btnRecord');
  const btnVoice = document.getElementById('btnVoice');
  const btnSettings = document.getElementById('btnSettings');
  const btnFullscreen = document.getElementById('btnFullscreen');

  // Standby Elements
  const dispRtspUrl = document.getElementById('dispRtspUrl');
  const dispDroneIp = document.getElementById('dispDroneIp');
  const dispLinkStatus = document.getElementById('dispLinkStatus');
  const btnOpenGuideFromStandby = document.getElementById('btnOpenGuideFromStandby');
  const btnOpenSettingsFromStandby = document.getElementById('btnOpenSettingsFromStandby');

  // Modals & Studio
  const missionModal = document.getElementById('mission-modal');
  const btnCloseMission = document.getElementById('btnCloseMission');
  const imagesStudioModal = document.getElementById('imagesStudioModal');
  const btnCloseStudio = document.getElementById('btnCloseStudio');
  const btnCloseStudioBottom = document.getElementById('btnCloseStudioBottom');
  const btnCaptureFrameDisk = document.getElementById('btnCaptureFrameDisk');
  const chkAutoCapture = document.getElementById('chkAutoCapture');
  const selAutoCaptureFps = document.getElementById('selAutoCaptureFps');
  const framesGalleryGrid = document.getElementById('framesGalleryGrid');
  const galleryCount = document.getElementById('galleryCount');
  const btnRefreshGallery = document.getElementById('btnRefreshGallery');

  const guideModal = document.getElementById('guideModal');
  const btnCloseGuide = document.getElementById('btnCloseGuide');
  const btnCloseGuideBottom = document.getElementById('btnCloseGuideBottom');
  const settingsModal = document.getElementById('settingsModal');
  const btnCloseSettings = document.getElementById('btnCloseSettings');
  const btnSaveConfig = document.getElementById('btnSaveConfig');
  const cfgDroneIp = document.getElementById('cfgDroneIp');
  const cfgUdpPort = document.getElementById('cfgUdpPort');
  const cfgRtspUrl = document.getElementById('cfgRtspUrl');

  let mediaRecorder = null;
  let recordedChunks = [];
  let telemetryWs = null;
  let videoWs = null;
  const hud = new DroneHUD('hudCanvas');

  // Continuous Video Stream State
  let lastDroneFrameTime = 0;
  let latestDroneBitmap = null;
  let frameCount = 0;
  let lastFpsCalcTime = performance.now();

  function resizeCanvases() {
    const w = videoCanvas.parentElement.clientWidth;
    const h = videoCanvas.parentElement.clientHeight;
    videoCanvas.width = w;
    videoCanvas.height = h;
    hud.resize();
  }
  resizeCanvases();
  window.addEventListener('resize', resizeCanvases);

  // ----------------- TOAST & VOICE NOTIFICATIONS ----------------- //
  let toastTimer = null;
  function showToast(msg, duration = 2000) {
    if (!hudToast) return;
    hudToast.textContent = msg;
    hudToast.classList.remove('hide');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => {
      hudToast.classList.add('hide');
    }, duration);
  }

  function speak(text) {
    if (!state.voiceEnabled || !window.speechSynthesis) return;
    try {
      window.speechSynthesis.cancel();
      const utterance = new SpeechSynthesisUtterance(text);
      utterance.rate = 1.15;
      utterance.pitch = 1.0;
      window.speechSynthesis.speak(utterance);
    } catch (e) {}
  }

  // ----------------- WEBSOCKET COMMUNICATIONS ----------------- //
  const loc = window.location;
  const wsProto = loc.protocol === 'https:' ? 'wss:' : 'ws:';
  const telemetryWsUrl = `${wsProto}//${loc.host}/ws/telemetry`;
  const videoWsUrl = `${wsProto}//${loc.host}/ws/video`;

  function initTelemetryWebSocket() {
    telemetryWs = new WebSocket(telemetryWsUrl);

    telemetryWs.onopen = () => {
      showToast('GCS GROUND STATION LINK ACTIVE');
    };

    telemetryWs.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === 'telemetry') {
          updateTelemetryUI(msg);
        }
      } catch (e) {}
    };

    telemetryWs.onclose = () => {
      setTimeout(initTelemetryWebSocket, 1500);
    };
    telemetryWs.onerror = () => {
      telemetryWs.close();
    };
  }

  function initVideoWebSocket() {
    videoWs = new WebSocket(videoWsUrl);
    videoWs.binaryType = 'arraybuffer';

    let isFrameDecoding = false;
    let newestData = null;

    videoWs.onmessage = (event) => {
      newestData = event.data;
      if (isFrameDecoding) return; // Always keep processing freshest buffer

      isFrameDecoding = true;
      const dataToDecode = newestData;
      newestData = null;

      const blob = (dataToDecode instanceof Blob) ? dataToDecode : new Blob([dataToDecode], { type: 'image/jpeg' });
      createImageBitmap(blob).then((imgBitmap) => {
        const old = latestDroneBitmap;
        latestDroneBitmap = imgBitmap;
        if (old) old.close();
        lastDroneFrameTime = performance.now();
        state.hasLiveVideo = true;
        frameCount++;
        isFrameDecoding = false;

        if (newestData) {
          const nextData = newestData;
          newestData = null;
          isFrameDecoding = true;
          const nextBlob = (nextData instanceof Blob) ? nextData : new Blob([nextData], { type: 'image/jpeg' });
          createImageBitmap(nextBlob).then((bm) => {
            const prev = latestDroneBitmap;
            latestDroneBitmap = bm;
            if (prev) prev.close();
            lastDroneFrameTime = performance.now();
            state.hasLiveVideo = true;
            frameCount++;
            isFrameDecoding = false;
          }).catch(() => { isFrameDecoding = false; });
        }
      }).catch(() => {
        isFrameDecoding = false;
      });
    };

    videoWs.onclose = () => {
      setTimeout(initVideoWebSocket, 1500);
    };
    videoWs.onerror = () => {
      videoWs.close();
    };
  }

  initTelemetryWebSocket();
  initVideoWebSocket();

  function sendCommand(payload) {
    if (telemetryWs && telemetryWs.readyState === WebSocket.OPEN) {
      telemetryWs.send(JSON.stringify(payload));
    }
    // Redundant HTTP REST fallback for critical triggers
    if (payload.action === 'takeoff') {
      fetch('/api/control', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ take_off: true }) }).catch(() => {});
    } else if (payload.action === 'land') {
      fetch('/api/control', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ land: true }) }).catch(() => {});
    } else if (payload.action === 'emergency_stop') {
      fetch('/api/control', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ emergency: true }) }).catch(() => {});
    }
  }

  // ----------------- TELEMETRY UI SYNC ----------------- //
  function updateTelemetryUI(data) {
    state.connected = !!data.connected;
    state.hasLiveVideo = !!data.has_live_video;
    state.videoSource = data.video_source || 'NONE';
    state.savedFramesCount = data.saved_frames_count || 0;

    if (savedFramesStat) savedFramesStat.textContent = state.savedFramesCount;
    if (dispRtspUrl && data.rtsp_url) dispRtspUrl.textContent = data.rtsp_url;
    if (dispDroneIp && data.drone_ip) dispDroneIp.textContent = data.drone_ip;

    if (data.connected || state.hasLiveVideo) {
      connectionStatus.className = 'v-status-pill online';
      connectionText.textContent = state.hasLiveVideo ? `LIVE FEED (${state.videoSource})` : 'DRONE LINK ONLINE';
      if (protocolBadge) protocolBadge.textContent = data.device_type || 'GL-21B';
      if (latencyValue) latencyValue.textContent = '8 ms';
      if (dispLinkStatus) {
        dispLinkStatus.textContent = state.hasLiveVideo ? `Streaming via ${state.videoSource}` : 'Connected (7099 UDP)';
        dispLinkStatus.className = 'text-emerald';
      }
    } else {
      connectionStatus.className = 'v-status-pill standby';
      connectionText.textContent = 'SEARCHING DRONE...';
      if (protocolBadge) protocolBadge.textContent = 'STANDBY';
      if (latencyValue) latencyValue.textContent = '-- ms';
      if (dispLinkStatus) {
        dispLinkStatus.textContent = 'Searching for Wi-Fi Link...';
        dispLinkStatus.className = 'text-amber';
      }
    }
    
    // Update Tapes
    const throttlePercent = Math.round(((data.throttle || 128) / 255) * 100);
    if (throttleBar) throttleBar.style.height = `${throttlePercent}%`;
    if (throttleVal) throttleVal.textContent = `${throttlePercent}%`;

    const pitchDeg = Math.round(((data.pitch || 128) - 128) * 0.25);
    const pitchFill = Math.min(100, Math.max(0, 50 + pitchDeg * 2));
    if (pitchBar) pitchBar.style.height = `${pitchFill}%`;
    if (pitchVal) pitchVal.textContent = `${pitchDeg}°`;

    // Update HUD attitude
    hud.updateAttitude(data.roll || 128, data.pitch || 128, data.yaw || 128, data.throttle || 128);
  }

  // ----------------- FLIGHT STABILIZATION & CHANNEL MAPPING ----------------- //
  function applyExpo(val, expo = 0.25) {
    return val * (1 - expo) + Math.pow(val, 3) * expo;
  }

  function updateFlightChannels() {
    if (state.gear === 1) {
      state.halfLen = 40;
      state.trimMultiplier = 2;
    } else if (state.gear === 2) {
      state.halfLen = 60;
      state.trimMultiplier = 3;
    } else {
      state.halfLen = 127;
      state.trimMultiplier = 4;
    }

    const deadband = (v) => Math.abs(v) < 0.04 ? 0 : v;

    const cleanRoll = deadband(applyExpo(state.axisRoll));
    const cleanPitch = deadband(applyExpo(state.axisPitch));
    const cleanYaw = deadband(applyExpo(state.axisYaw));
    const cleanThrottle = deadband(applyExpo(state.axisThrottle));

    const isUserTouch = (cleanRoll !== 0 || cleanPitch !== 0 || cleanYaw !== 0 || cleanThrottle !== 0);

    if (isUserTouch || !missionRunner || !missionRunner.isRunning) {
      let r = Math.round(128 + (cleanRoll * state.halfLen) + (state.rollTrim * state.trimMultiplier));
      let p = Math.round(128 + (cleanPitch * state.halfLen) + (state.pitchTrim * state.trimMultiplier));
      let t = Math.round(128 + (cleanThrottle * 127.0));
      let y = Math.round(128 + (cleanYaw * 127.0) + (state.yawTrim * 4));

      state.roll = Math.max(1, Math.min(255, r));
      state.pitch = Math.max(1, Math.min(255, p));
      state.throttle = Math.max(0, Math.min(255, t));
      state.yaw = Math.max(1, Math.min(255, y));
    }

    sendCommand({
      action: 'stick',
      roll: state.roll,
      pitch: state.pitch,
      throttle: state.throttle,
      yaw: state.yaw
    });

    hud.updateAttitude(state.roll, state.pitch, state.yaw, state.throttle);
  }

  // Left Joystick: Throttle (Y: Up=+1.0, Down=-1.0) & Yaw (X: Right=+1.0, Left=-1.0)
  const joyLeft = new VirtualJoystick('leftJoystickWrapper', 'leftJoystickNipple', (x, y) => {
    state.axisYaw = x;
    state.axisThrottle = y;
    updateFlightChannels();
  });

  // Right Joystick: Pitch (Y: Up/Forward=+1.0, Down/Back=-1.0) & Roll (X: Right=+1.0, Left=-1.0)
  const joyRight = new VirtualJoystick('rightJoystickWrapper', 'rightJoystickNipple', (x, y) => {
    state.axisRoll = x;
    state.axisPitch = y;
    updateFlightChannels();
  });

  // ----------------- KEYBOARD CONTROLS ----------------- //
  const keyState = {};
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    keyState[e.code] = true;

    if (e.code === 'Space') {
      e.preventDefault();
      triggerTakeoff();
    } else if (e.code === 'ShiftLeft' || e.code === 'ShiftRight') {
      triggerLand();
    } else if (e.code === 'Escape') {
      triggerEmergency();
    } else if (e.code === 'KeyG') {
      triggerCalibrate();
    } else if (e.code === 'KeyF') {
      triggerFlip('right');
    } else if (e.code === 'KeyC') {
      triggerCamSwitch();
    } else if (e.code === 'KeyR') {
      triggerRotateCamera();
    } else if (e.code === 'KeyV') {
      triggerInvertCamera();
    } else if (e.code === 'KeyH') {
      triggerMirrorCamera();
    } else if (e.code === 'KeyP') {
      triggerCaptureFrameDisk();
    } else if (e.code === 'KeyM') {
      toggleMissionModal();
    } else if (e.code === 'Digit1') {
      setGear(1);
    } else if (e.code === 'Digit2') {
      setGear(2);
    } else if (e.code === 'Digit3') {
      setGear(3);
    }
    processKeyboardFlightAxes();
  });

  window.addEventListener('keyup', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    keyState[e.code] = false;
    processKeyboardFlightAxes();
  });

  function processKeyboardFlightAxes() {
    let kt = 0, ky = 0, kp = 0, kr = 0;
    if (keyState['KeyW']) kt += 1.0;
    if (keyState['KeyS']) kt -= 1.0;
    if (keyState['KeyA']) ky -= 1.0;
    if (keyState['KeyD']) ky += 1.0;

    if (keyState['ArrowUp']) kp += 1.0;
    if (keyState['ArrowDown']) kp -= 1.0;
    if (keyState['ArrowLeft']) kr -= 1.0;
    if (keyState['ArrowRight']) kr += 1.0;

    if (!joyLeft.isDragging) {
      state.axisThrottle = kt;
      state.axisYaw = ky;
    }
    if (!joyRight.isDragging) {
      state.axisPitch = kp;
      state.axisRoll = kr;
    }
    updateFlightChannels();
  }

  // ----------------- GAMEPAD POLLING ----------------- //
  function pollGamepads() {
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    let gp = null;
    for (let i = 0; i < gamepads.length; i++) {
      if (gamepads[i]) { gp = gamepads[i]; break; }
    }

    if (gp) {
      gamepadIndicator.classList.remove('hide');
      const deadzone = 0.12;
      const applyDeadzone = (v) => (Math.abs(v) > deadzone ? v : 0);

      const ax0 = applyDeadzone(gp.axes[0] || 0); // Yaw
      const ax1 = applyDeadzone(gp.axes[1] || 0); // Throttle
      const ax2 = applyDeadzone(gp.axes[2] || 0); // Roll
      const ax3 = applyDeadzone(gp.axes[3] || 0); // Pitch

      if (!joyLeft.isDragging && !keyState['KeyW'] && !keyState['KeyS'] && !keyState['KeyA'] && !keyState['KeyD']) {
        state.axisYaw = ax0;
        state.axisThrottle = -ax1;
      }
      if (!joyRight.isDragging && !keyState['ArrowUp'] && !keyState['ArrowDown'] && !keyState['ArrowLeft'] && !keyState['ArrowRight']) {
        state.axisRoll = ax2;
        state.axisPitch = -ax3;
      }
      updateFlightChannels();

      if (gp.buttons[0]?.pressed) triggerTakeoff();
      if (gp.buttons[1]?.pressed) triggerLand();
      if (gp.buttons[9]?.pressed) triggerEmergency();
      if (gp.buttons[5]?.pressed) triggerFlip('right');
      if (gp.buttons[4]?.pressed) triggerCalibrate();
      if (gp.buttons[7]?.pressed) triggerCaptureFrameDisk();
    } else {
      gamepadIndicator.classList.add('hide');
    }
  }
  setInterval(pollGamepads, 50);

  // ----------------- ACTION HANDLERS ----------------- //
  function triggerTakeoff() {
    btnTakeoff.classList.add('active');
    setTimeout(() => btnTakeoff.classList.remove('active'), 1200);
    showToast('TAKEOFF SEQUENCE ENGAGED');
    speak('Takeoff initiated');
    sendCommand({ action: 'takeoff' });
  }

  function triggerLand() {
    btnLand.classList.add('active');
    setTimeout(() => btnLand.classList.remove('active'), 1200);
    showToast('AUTO-LAND ENGAGED');
    speak('Landing sequence engaged');
    sendCommand({ action: 'land' });
  }

  function triggerEmergency() {
    btnEmergency.classList.add('active');
    setTimeout(() => btnEmergency.classList.remove('active'), 1000);
    showToast('EMERGENCY MOTOR CUTOFF!');
    speak('Emergency stop activated');
    sendCommand({ action: 'emergency_stop' });
  }

  function triggerCalibrate() {
    btnGyro.classList.add('active');
    setTimeout(() => btnGyro.classList.remove('active'), 2000);
    showToast('GYRO CALIBRATION INITIATED — KEEP LEVEL');
    speak('Calibrating sensors');
    sendCommand({ action: 'calibrate_gyro' });
  }

  function triggerFlip(dir = 'right') {
    showToast(`360° ${dir.toUpperCase()} STUNT FLIP`);
    speak(`Performing ${dir} flip`);
    sendCommand({ action: 'flip_360', direction: dir });
  }

  function triggerCamSwitch() {
    state.cameraId = state.cameraId === 1 ? 2 : 1;
    camIdText.textContent = state.cameraId === 1 ? 'FRONT' : 'BOTTOM';
    showToast(`SWITCHED TO ${camIdText.textContent} LENS`);
    speak(`${camIdText.textContent} camera active`);
    sendCommand({ action: 'switch_camera', camera_id: state.cameraId });
  }

  function triggerRotateCamera() {
    state.cameraRotation = (state.cameraRotation + 90) % 360;
    localStorage.setItem('drone_cam_rotation', String(state.cameraRotation));
    updateCameraTransformUI();
    showToast(`CAMERA ROTATION: ${state.cameraRotation}°`);
    speak(`Camera rotated to ${state.cameraRotation} degrees`);
  }

  function triggerInvertCamera() {
    state.cameraInvert = !state.cameraInvert;
    localStorage.setItem('drone_cam_invert', String(state.cameraInvert));
    updateCameraTransformUI();
    showToast(`CAMERA INVERT (V-FLIP): ${state.cameraInvert ? 'ON' : 'OFF'}`);
    speak(`Camera vertical invert ${state.cameraInvert ? 'on' : 'off'}`);
  }

  function triggerMirrorCamera() {
    state.cameraMirror = !state.cameraMirror;
    localStorage.setItem('drone_cam_mirror', String(state.cameraMirror));
    updateCameraTransformUI();
    showToast(`CAMERA MIRROR (H-FLIP): ${state.cameraMirror ? 'ON' : 'OFF'}`);
    speak(`Camera horizontal mirror ${state.cameraMirror ? 'on' : 'off'}`);
  }

  function updateCameraTransformUI() {
    if (rotationValText) rotationValText.textContent = `${state.cameraRotation}°`;
    if (invertStatus) {
      invertStatus.textContent = state.cameraInvert ? 'ON' : 'OFF';
      invertStatus.className = state.cameraInvert ? 'btn-sub status-on' : 'btn-sub status-off';
    }
    if (btnInvertFeed) btnInvertFeed.classList.toggle('active', state.cameraInvert);
    if (mirrorStatus) {
      mirrorStatus.textContent = state.cameraMirror ? 'ON' : 'OFF';
      mirrorStatus.className = state.cameraMirror ? 'btn-sub status-on' : 'btn-sub status-off';
    }
    if (btnMirrorFeed) btnMirrorFeed.classList.toggle('active', state.cameraMirror);
  }

  updateCameraTransformUI();

  function triggerCaptureFrameDisk() {
    fetch('/api/capture-snapshot', { method: 'POST' })
      .then(r => r.json())
      .then(data => {
        if (data.success) {
          showToast(`FRAME SAVED TO DISK (${data.filename})`);
          speak('Frame captured');
          if (savedFramesStat) savedFramesStat.textContent = data.total_saved;
          loadFramesGallery();
        } else {
          showToast('NO CAMERA FRAME AVAILABLE YET');
        }
      })
      .catch(() => showToast('FAILED TO SAVE FRAME'));
  }

  btnTakeoff.addEventListener('click', triggerTakeoff);
  btnLand.addEventListener('click', triggerLand);
  btnEmergency.addEventListener('click', triggerEmergency);
  btnGyro.addEventListener('click', triggerCalibrate);
  btnCamSwitch.addEventListener('click', triggerCamSwitch);
  if (btnRotateFeed) btnRotateFeed.addEventListener('click', triggerRotateCamera);
  if (btnInvertFeed) btnInvertFeed.addEventListener('click', triggerInvertCamera);
  if (btnMirrorFeed) btnMirrorFeed.addEventListener('click', triggerMirrorCamera);

  btnFlipMenu.addEventListener('click', (e) => {
    e.stopPropagation();
    flipDropdown.classList.toggle('hide');
  });

  document.addEventListener('click', () => {
    flipDropdown.classList.add('hide');
  });

  document.querySelectorAll('[data-flip]').forEach((btn) => {
    btn.addEventListener('click', (e) => {
      e.stopPropagation();
      triggerFlip(e.currentTarget.dataset.flip);
      flipDropdown.classList.add('hide');
    });
  });

  btnHeadless.addEventListener('click', () => {
    state.isNoHeadMode = !state.isNoHeadMode;
    btnHeadless.classList.toggle('active', state.isNoHeadMode);
    headlessStatus.textContent = state.isNoHeadMode ? 'ACTIVE' : 'INACTIVE';
    headlessStatus.className = state.isNoHeadMode ? 'btn-sub status-on' : 'btn-sub status-off';
    showToast(`HEADLESS MODE: ${headlessStatus.textContent}`);
    speak(`Headless mode ${headlessStatus.textContent}`);
    sendCommand({ action: 'toggle_headless' });
  });

  btnAltHold.addEventListener('click', () => {
    state.isFixedHeight = !state.isFixedHeight;
    btnAltHold.classList.toggle('active', state.isFixedHeight);
    altHoldStatus.textContent = state.isFixedHeight ? 'ACTIVE' : 'INACTIVE';
    altHoldStatus.className = state.isFixedHeight ? 'btn-sub status-on' : 'btn-sub status-off';
    showToast(`ALTITUDE & OPTICAL FLOW HOLD: ${altHoldStatus.textContent}`);
    speak(`Altitude hold ${altHoldStatus.textContent}`);
    sendCommand({ action: 'toggle_altitude_hold' });
  });

  // Gear Selector (30% Slow / 60% Standard / 100% Sport)
  function setGear(g) {
    state.gear = g;
    document.querySelectorAll('.v-gear-btn').forEach((b) => {
      b.classList.toggle('active', parseInt(b.dataset.gear) === g);
    });
    const gearPct = g === 1 ? '30%' : g === 2 ? '60%' : '100%';
    gearBadge.textContent = gearPct;
    showToast(`SPEED GEAR: ${gearPct}`);
    speak(`Speed gear ${gearPct}`);
    sendCommand({ action: 'set_gear', gear: g });
    updateFlightChannels();
  }

  document.querySelectorAll('.v-gear-btn').forEach((b) => {
    b.addEventListener('click', () => setGear(parseInt(b.dataset.gear)));
  });

  // ----------------- TRIMS ----------------- //
  document.getElementById('trimYawMinus').addEventListener('click', () => adjustTrim('yaw', -1));
  document.getElementById('trimYawPlus').addEventListener('click', () => adjustTrim('yaw', 1));
  document.getElementById('trimRollMinus').addEventListener('click', () => adjustTrim('roll', -1));
  document.getElementById('trimRollPlus').addEventListener('click', () => adjustTrim('roll', 1));
  document.getElementById('trimPitchMinus').addEventListener('click', () => adjustTrim('pitch', -1));
  document.getElementById('trimPitchPlus').addEventListener('click', () => adjustTrim('pitch', 1));

  function adjustTrim(type, delta) {
    if (type === 'yaw') {
      state.yawTrim = Math.max(-10, Math.min(10, state.yawTrim + delta));
      trimYawVal.textContent = state.yawTrim;
    } else if (type === 'roll') {
      state.rollTrim = Math.max(-10, Math.min(10, state.rollTrim + delta));
      trimRollVal.textContent = state.rollTrim;
    } else if (type === 'pitch') {
      state.pitchTrim = Math.max(-10, Math.min(10, state.pitchTrim + delta));
      trimPitchVal.textContent = state.pitchTrim;
    }
    showToast(`${type.toUpperCase()} TRIM: ${state[type + 'Trim']}`);
    sendCommand({
      action: 'set_trims',
      roll_trim: state.rollTrim,
      pitch_trim: state.pitchTrim,
      yaw_trim: state.yawTrim
    });
    updateFlightChannels();
  }

  // ----------------- SNAPSHOT & RECORDING ----------------- //
  btnSnapshot.addEventListener('click', () => {
    triggerCaptureFrameDisk();
  });

  btnRecord.addEventListener('click', () => {
    if (!state.isRecording) {
      try {
        recordedChunks = [];
        const stream = videoCanvas.captureStream(30);
        mediaRecorder = new MediaRecorder(stream, { mimeType: 'video/webm' });
        mediaRecorder.ondataavailable = (e) => {
          if (e.data.size > 0) recordedChunks.push(e.data);
        };
        mediaRecorder.onstop = () => {
          const blob = new Blob(recordedChunks, { type: 'video/webm' });
          const url = URL.createObjectURL(blob);
          const a = document.createElement('a');
          a.href = url;
          a.download = `rc_ufo_flight_${Date.now()}.webm`;
          a.click();
        };
        mediaRecorder.start();
        state.isRecording = true;
        recDot.classList.remove('hide');
        btnRecord.classList.add('active');
        showToast('RECORDING STARTED');
        speak('Recording started');
      } catch (e) {
        showToast('RECORDING UNAVAILABLE');
      }
    } else {
      if (mediaRecorder) mediaRecorder.stop();
      state.isRecording = false;
      recDot.classList.add('hide');
      btnRecord.classList.remove('active');
      showToast('VIDEO RECORDING SAVED');
      speak('Recording saved');
    }
  });

  // ----------------- AUTONOMOUS MISSION RUNNER STUDIO ----------------- //
  let missionRunner = null;
  if (window.MissionRunner) {
    missionRunner = new window.MissionRunner({
      droneState: state,
      triggerTakeoff,
      triggerLand,
      triggerEmergency,
      triggerFlip,
      updateFlightChannels
    });
  }

  function openMissionModal() {
    if (missionModal) missionModal.classList.remove('hide');
  }
  function closeMissionModal() {
    if (missionModal) missionModal.classList.add('hide');
  }
  function toggleMissionModal() {
    if (missionModal) {
      if (missionModal.classList.contains('hide')) openMissionModal();
      else closeMissionModal();
    }
  }

  if (btnMissions) btnMissions.addEventListener('click', openMissionModal);
  if (btnCloseMission) btnCloseMission.addEventListener('click', closeMissionModal);
  if (missionModal) {
    missionModal.addEventListener('click', (e) => {
      if (e.target === missionModal) closeMissionModal();
    });
  }

  // ----------------- IMAGE PROCESSING & FRAME GALLERY STUDIO ----------------- //
  function openStudio() {
    imagesStudioModal.classList.remove('hide');
    loadFramesGallery();
  }
  function closeStudio() {
    imagesStudioModal.classList.add('hide');
  }

  btnImagesStudio.addEventListener('click', openStudio);
  btnCloseStudio.addEventListener('click', closeStudio);
  btnCloseStudioBottom.addEventListener('click', closeStudio);
  imagesStudioModal.addEventListener('click', (e) => {
    if (e.target === imagesStudioModal) closeStudio();
  });

  btnCaptureFrameDisk.addEventListener('click', triggerCaptureFrameDisk);

  chkAutoCapture.addEventListener('change', (e) => {
    const enabled = e.target.checked;
    const fps = parseInt(selAutoCaptureFps.value, 10) || 2;
    fetch('/api/auto-capture', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled, fps })
    }).then(r => r.json()).then(d => {
      showToast(d.auto_capture ? `AUTO-CAPTURE ACTIVE (${d.fps} FPS)` : 'AUTO-CAPTURE DISABLED');
    });
  });

  selAutoCaptureFps.addEventListener('change', (e) => {
    if (chkAutoCapture.checked) {
      const fps = parseInt(e.target.value, 10) || 2;
      fetch('/api/auto-capture', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enabled: true, fps })
      }).then(r => r.json()).then(d => {
        showToast(`AUTO-CAPTURE SPEED: ${d.fps} FPS`);
      });
    }
  });

  function loadFramesGallery() {
    fetch('/api/frames-list')
      .then(r => r.json())
      .then(data => {
        if (!framesGalleryGrid) return;
        if (galleryCount) galleryCount.textContent = data.total;
        if (savedFramesStat) savedFramesStat.textContent = data.total;

        if (!data.frames || data.frames.length === 0) {
          framesGalleryGrid.innerHTML = `<div class="gallery-empty">No frames captured yet. Connect to drone and click 'Capture Frame Now' or press 'P'.</div>`;
          return;
        }

        framesGalleryGrid.innerHTML = data.frames.map(f => `
          <div class="gallery-item">
            <img src="${f.url}" class="gallery-img" alt="${f.filename}" loading="lazy">
            <div class="gallery-meta-overlay">
              <span>${f.filename.slice(0, 16)}...</span>
              <a href="${f.url}" download="${f.filename}" class="gallery-download-link" title="Download High-Res Image">DL</a>
            </div>
          </div>
        `).join('');
      })
      .catch(() => {});
  }

  btnRefreshGallery.addEventListener('click', loadFramesGallery);

  // ----------------- FULLSCREEN & VOICE ----------------- //
  btnFullscreen.addEventListener('click', () => {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen().catch(() => {});
    } else {
      document.exitFullscreen().catch(() => {});
    }
  });

  btnVoice.addEventListener('click', () => {
    state.voiceEnabled = !state.voiceEnabled;
    btnVoice.classList.toggle('active', state.voiceEnabled);
    showToast(`VOICE ALERTS: ${state.voiceEnabled ? 'ON' : 'OFF'}`);
  });

  // ----------------- GUIDE MODAL & TABS ----------------- //
  function openGuide() {
    guideModal.classList.remove('hide');
  }
  function closeGuide() {
    guideModal.classList.add('hide');
  }

  btnGuide.addEventListener('click', openGuide);
  if (btnOpenGuideFromStandby) btnOpenGuideFromStandby.addEventListener('click', openGuide);
  btnCloseGuide.addEventListener('click', closeGuide);
  btnCloseGuideBottom.addEventListener('click', closeGuide);
  guideModal.addEventListener('click', (e) => {
    if (e.target === guideModal) closeGuide();
  });

  document.querySelectorAll('.v-tab-btn').forEach((tabBtn) => {
    tabBtn.addEventListener('click', (e) => {
      const targetId = e.currentTarget.dataset.tab;
      document.querySelectorAll('.v-tab-btn').forEach((b) => b.classList.remove('active'));
      document.querySelectorAll('.guide-panel').forEach((p) => p.classList.remove('active'));
      e.currentTarget.classList.add('active');
      const targetPanel = document.getElementById(targetId);
      if (targetPanel) targetPanel.classList.add('active');
    });
  });

  // ----------------- SETTINGS MODAL ----------------- //
  function openSettings() {
    settingsModal.classList.remove('hide');
  }
  function closeSettings() {
    settingsModal.classList.add('hide');
  }

  btnSettings.addEventListener('click', openSettings);
  if (btnOpenSettingsFromStandby) btnOpenSettingsFromStandby.addEventListener('click', openSettings);
  btnCloseSettings.addEventListener('click', closeSettings);
  settingsModal.addEventListener('click', (e) => {
    if (e.target === settingsModal) closeSettings();
  });

  if (btnSaveConfig) {
    btnSaveConfig.addEventListener('click', () => {
      const drone_ip = cfgDroneIp.value.trim();
      const udp_port = cfgUdpPort.value.trim();
      const rtsp_url = cfgRtspUrl.value.trim();
      showToast(`CONFIGURING ${drone_ip}...`);
      speak(`Reconnecting to drone at ${drone_ip}`);
      sendCommand({
        action: 'update_config',
        drone_ip,
        udp_port,
        rtsp_url
      });
      fetch('/api/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ drone_ip, udp_port, rtsp_url })
      }).catch(() => {});
      closeSettings();
    });
  }

  // ----------------- MAIN CONTINUOUS VIDEO & HUD RENDER LOOP ----------------- //
  const hudOverlays = document.querySelector('.v-hud-overlays');

  function renderLoop() {
    const w = videoCanvas.width;
    const h = videoCanvas.height;
    const now = performance.now();

    const hasRecentDroneFrame = (now - lastDroneFrameTime < 4000) && latestDroneBitmap;

    if (hasRecentDroneFrame) {
      videoCtx.save();
      videoCtx.clearRect(0, 0, w, h);
      videoCtx.translate(w / 2, h / 2);

      if (state.cameraRotation !== 0) {
        videoCtx.rotate((state.cameraRotation * Math.PI) / 180);
      }

      const scaleX = state.cameraMirror ? -1 : 1;
      const scaleY = state.cameraInvert ? -1 : 1;
      if (scaleX !== 1 || scaleY !== 1) {
        videoCtx.scale(scaleX, scaleY);
      }

      const isRotated90 = (state.cameraRotation === 90 || state.cameraRotation === 270);
      const drawW = isRotated90 ? h : w;
      const drawH = isRotated90 ? w : h;
      videoCtx.drawImage(latestDroneBitmap, -drawW / 2, -drawH / 2, drawW, drawH);
      videoCtx.restore();

      standbyOverlay.classList.add('hide');
      if (hudOverlays) hudOverlays.classList.remove('hide');
      hud.render();
    } else {
      videoCtx.clearRect(0, 0, w, h);
      standbyOverlay.classList.remove('hide');
      if (hudOverlays) hudOverlays.classList.add('hide');
      hud.clear();
    }

    if (now - lastFpsCalcTime >= 1000) {
      state.fps = hasRecentDroneFrame ? frameCount : 0;
      if (fpsValue) fpsValue.textContent = state.fps;
      frameCount = 0;
      lastFpsCalcTime = now;
    }

    requestAnimationFrame(renderLoop);
  }

  requestAnimationFrame(renderLoop);
});
