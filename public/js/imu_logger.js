/**
 * Drone 6-DOF IMU Telemetry, 3D Local Coordinate Tracker (0,0,0 Origin), Real-Time Flight Logger & CSV/Excel Exporter
 * 
 * Captures, computes, and logs:
 * - 3D Local Coordinates: X (East/Lateral m), Y (North/Forward m), Z (Altitude m) with start point at (0,0,0)
 * - Navigation & Odometry: Distance from Home (m), Total Distance Traveled (m), Ground Speed (m/s & km/h), Climb Velocity (m/s)
 * - Attitude Angles: Roll (deg), Pitch (deg), Yaw / Heading (deg)
 * - Tri-Axial Angular Velocity / Gyro Rates: wx (deg/s), wy (deg/s), wz (deg/s)
 * - Tri-Axial Linear Acceleration: Ax (g), Ay (g), Az (g), and Total G-Force (g)
 * - Altitude Hold & Vertical Velocity: Estimated Alt (%), Estimated Alt (m), Vz (m/s)
 * - Raw 8-bit Stick & Protocol Channels: Roll (1-255), Pitch (1-255), Throttle (0-255), Yaw (1-255)
 * - Flight Timers: Active Sortie Flight Time (seconds), Total Cumulative Session Flight Time (seconds)
 * - Flight Modes & Trims: Roll/Pitch/Yaw Trims, Altitude Hold, Headless, Gyro Calibration, 360 Stunt Flip
 * - Network Link Stats: Latency (ms), Packets TX/RX
 */

class DroneIMULogger {
  constructor() {
    this.maxLogs = 10000; // In-memory sample buffer (over 15 minutes of continuous 10Hz data)
    this.logs = [];
    this.isLogging = true;
    this.autoLogOnArm = true;

    // Flight Sortie Architecture (Separate records per flight simulation / takeoff)
    this.sortieCount = 1;
    this.currentSortieName = 'Flight Sortie #1';
    this.currentSortieLogs = []; // Samples for ONLY the current active flight simulation
    this.allSessionLogs = [];    // Cumulative samples across all sorties
    this.sessionSorties = [];    // Archive of finished sorties
    this.logs = this.currentSortieLogs; // Primary display buffer

    // Hardware Physical Sensor Ingestion (USB Serial / Bluetooth / MPU-6050)
    this.hardwareSensorActive = false;
    this.externalSensorData = null;
    
    // Flight Timers
    this.sessionStartTime = Date.now();
    this.totalSessionFlightTime = 0; // Cumulative seconds spent flying across session
    this.currentSortieFlightTime = 0;  // Seconds spent in current active flight sortie
    this.isAirborne = false;
    this.airborneStartTime = 0;
    this.lastFlightTimerTick = Date.now();

    // Change-Detection Logging State (Records only on change / events)
    this.lastLoggedSample = null;
    this.lastLogTimestamp = 0;
    this.heartbeatIntervalMs = 5000; // 5s periodic summary heartbeat when flying, 10s when idle

    // ----------------- 3D LOCAL COORDINATES (Origin 0,0,0) ----------------- //
    this.posX = 0.0; // East / Lateral displacement (meters) relative to start point (0,0,0)
    this.posY = 0.0; // North / Forward displacement (meters) relative to start point (0,0,0)
    this.posZ = 0.0; // Altitude / Vertical displacement (meters) relative to start point (0,0,0)
    
    this.velX = 0.0; // Lateral ground velocity (m/s)
    this.velY = 0.0; // Forward ground velocity (m/s)
    this.velZ = 0.0; // Vertical climb rate (m/s)
    this.groundSpeed = 0.0; // 2D Ground speed magnitude (m/s)
    this.distanceToHome = 0.0; // 3D Euclidean distance to (0,0,0)
    this.totalDistanceTraveled = 0.0; // Cumulative odometer (meters)
    this.groundReferenceAlt = 0.0; // Takeoff reference altitude

    // 2D Flight Path Breadcrumb Trail for Top-Down Radar Minimap
    this.maxTrailPoints = 350;
    this.pathTrail = [{ x: 0, y: 0, z: 0, heading: 0, time: 0 }];

    // Continuous 6-DOF Dynamic State Integrators (Floating Point Precision)
    this.curRollDeg = 0.0;
    this.curPitchDeg = 0.0;
    this.curYawDeg = 0.0;
    this.curGyroRoll = 0.0;
    this.curGyroPitch = 0.0;
    this.curGyroYaw = 0.0;

    // Previous state for numerical derivatives (calculating angular rates & accelerations)
    this.lastSampleTime = performance.now();
    this.prevRoll = 0.0;
    this.prevPitch = 0.0;
    this.prevYaw = 0.0;
    this.prevAltitude = 0.0;

    // Filtered / Smoothed IMU State
    this.currentIMU = {
      timestampIso: new Date().toISOString(),
      timestampEpoch: Date.now(),
      localTime: new Date().toLocaleTimeString('en-US', { hour12: false }),
      flightTime: 0,
      totalFlightTime: 0,
      flightEvent: 'INITIALIZE',
      flightStatus: 'GROUND',

      // 3D Coordinates (0,0,0 Origin)
      posX: 0.0,
      posY: 0.0,
      posZ: 0.0,
      velX: 0.0,
      velY: 0.0,
      velZ: 0.0,
      groundSpeed: 0.0,
      distanceToHome: 0.0,
      totalDistanceTraveled: 0.0,
      
      // Attitude (deg)
      rollDeg: 0.0,
      pitchDeg: 0.0,
      yawDeg: 0.0,

      // Gyro Rates (deg/s)
      gyroRollRate: 0.0,
      gyroPitchRate: 0.0,
      gyroYawRate: 0.0,

      // Accelerometer (g & m/s^2)
      accelX: 0.0,
      accelY: 0.0,
      accelZ: 1.0,
      totalG: 1.0,

      // Altitude
      altitudePct: 0,
      altitudeMeters: 0,
      climbRateMs: 0,

      // Raw Hardware Channels (1..255)
      rawRoll: 128,
      rawPitch: 128,
      rawThrottle: 128,
      rawYaw: 128,

      // Modes & Trims
      rollTrim: 0,
      pitchTrim: 0,
      yawTrim: 0,
      gearPct: 60,
      altHold: 1,
      headless: 0,
      gyroCalibrating: 0,
      stuntFlipping: 0,
      deviceType: 'GL-21B',
      latencyMs: 8,
      packetsSent: 0,
      packetsRecv: 0
    };

    // Waveform Oscilloscope History (last 120 points for live canvas graphs)
    this.historyLen = 120;
    this.history = {
      time: [],
      roll: [],
      pitch: [],
      yaw: [],
      accelZ: [],
      totalG: [],
      gyroRoll: [],
      gyroPitch: [],
      gyroYaw: [],
      posX: [],
      posY: [],
      posZ: []
    };

    // UI Tab & Filter State
    this.currentTab = 'tab-overview';
    this.logSearchTerm = '';
    this.activeWaveChannels = {
      roll: true,
      pitch: true,
      accelZ: true,
      gyroRoll: true
    };

    // Start 50ms high-precision flight timer loop
    setInterval(() => this._tickFlightTimer(), 50);

    // Bind DOM events once DOM is ready
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', () => this.initDOM());
    } else {
      setTimeout(() => this.initDOM(), 50);
    }
  }

  /**
   * Reset the drone coordinates and dynamic state to 0,0,0 (Home / Start Point Calibration)
   */
  resetCoordinates(x = 0, y = 0, z = 0, heading = 0) {
    this.posX = x;
    this.posY = y;
    this.posZ = z;
    this.velX = 0.0;
    this.velY = 0.0;
    this.velZ = 0.0;
    this.groundSpeed = 0.0;
    this.distanceToHome = 0.0;
    this.totalDistanceTraveled = 0.0;
    this.groundReferenceAlt = this.prevAltitude;
    this.curRollDeg = 0.0;
    this.curPitchDeg = 0.0;
    this.curYawDeg = heading;
    this.curGyroRoll = 0.0;
    this.curGyroPitch = 0.0;
    this.curGyroYaw = 0.0;
    this.prevRoll = 0.0;
    this.prevPitch = 0.0;
    this.prevYaw = heading;
    this.pathTrail = [{ x: this.posX, y: this.posY, z: this.posZ, heading: this.curYawDeg, time: this.currentSortieFlightTime }];
    this.updateUI();
  }

  /**
   * Starts a clean new Flight Sortie / Simulation.
   * Archives previous flight data so each flight simulation produces its own unique, isolated dataset!
   */
  startNewSortie(sortieName = '') {
    // If the active sortie had recorded flights, archive it
    if (this.currentSortieLogs && this.currentSortieLogs.length > 0) {
      this.sessionSorties.push({
        sortieNumber: this.sortieCount,
        sortieName: this.currentSortieName,
        sampleCount: this.currentSortieLogs.length,
        duration: this.currentSortieFlightTime,
        distance: this.totalDistanceTraveled,
        samples: [...this.currentSortieLogs]
      });
      this.sortieCount++;
    }

    this.currentSortieName = sortieName || `Flight Sortie #${this.sortieCount}`;
    this.currentSortieFlightTime = 0.0;
    this.currentSortieLogs = [];
    this.logs = this.currentSortieLogs;
    this.lastLoggedSample = null;
    this.lastLogTimestamp = 0;
    this.resetCoordinates(0, 0, 0);

    // Initial baseline sample at origin
    const initSample = this._createSampleObject(this.currentIMU, 'INITIALIZE');
    this._pushSample(initSample);

    const badgeSortie = document.getElementById('imuSortieBadge');
    if (badgeSortie) {
      badgeSortie.textContent = `Sortie #${this.sortieCount}: ${this.currentSortieName}`;
    }

    this.renderLogsTable();
    this.updateUI();
    return this.sortieCount;
  }

  /**
   * Appends sample to both the active sortie buffer and the global session archive
   */
  _pushSample(sample) {
    sample.sortieNumber = this.sortieCount;
    sample.sortieName = this.currentSortieName;

    this.currentSortieLogs.push(sample);
    this.allSessionLogs.push(sample);
    this.logs = this.currentSortieLogs;

    this.lastLoggedSample = Object.assign({}, sample);
    this.lastLogTimestamp = sample.timestampEpoch;

    if (this.currentSortieLogs.length > this.maxLogs) {
      this.currentSortieLogs.shift();
    }
    if (this.allSessionLogs.length > this.maxLogs * 3) {
      this.allSessionLogs.shift();
    }

    if (this.currentTab === 'tab-logs') {
      this.renderLogsTable();
    }
  }

  /**
   * Logs a specific event immediately (e.g. Mission steps, custom maneuvers)
   */
  logForcedEvent(eventName, desc = '') {
    if (!this.isLogging) return;
    const sample = this._createSampleObject(this.currentIMU, eventName);
    if (desc) sample.eventDescription = desc;
    this._pushSample(sample);
  }

  /**
   * Ingests real physical telemetry from external hardware (ESP32, Arduino, MPU-6050, or WebSocket)
   */
  injectHardwareTelemetry(sensorData = {}) {
    this.hardwareSensorActive = true;
    this.externalSensorData = sensorData;

    const pill = document.getElementById('imuHardwarePill');
    if (pill) {
      pill.textContent = 'HARDWARE SENSOR: ACTIVE';
      pill.className = 'v-tag v-tag-rec text-emerald';
    }
  }

  /**
   * Connect to real physical hardware sensor via Web Serial API (USB COM / ESP32 / Arduino / MPU6050)
   */
  async connectSerialSensor() {
    if (!('serial' in navigator)) {
      alert("Web Serial API is not supported in this browser. Please open in Google Chrome or Microsoft Edge to connect real hardware sensors over USB.");
      return;
    }
    try {
      const port = await navigator.serial.requestPort();
      await port.open({ baudRate: 115200 });
      this.hardwareSensorActive = true;

      const pill = document.getElementById('imuHardwarePill');
      if (pill) {
        pill.textContent = 'HARDWARE SENSOR: CONNECTED';
        pill.className = 'v-tag v-tag-rec text-emerald';
      }

      const textDecoder = new TextDecoderStream();
      const readableStreamClosed = port.readable.pipeTo(textDecoder.writable);
      const reader = textDecoder.readable.getReader();
      let buffer = '';

      while (true) {
        const { value, done } = await reader.read();
        if (done) break;
        buffer += value;
        const lines = buffer.split('\n');
        buffer = lines.pop(); // keep partial trailing line

        for (const line of lines) {
          const trimmed = line.trim();
          if (!trimmed) continue;
          try {
            // Check if JSON: {"roll": 1.2, "pitch": -0.5, "yaw": 180, "ax": 0.02, "ay": -0.01, "az": 1.0, "alt": 0.85}
            if (trimmed.startsWith('{') && trimmed.endsWith('}')) {
              const parsed = JSON.parse(trimmed);
              this.injectHardwareTelemetry({
                roll: parsed.roll ?? parsed.r,
                pitch: parsed.pitch ?? parsed.p,
                yaw: parsed.yaw ?? parsed.y,
                accelX: parsed.accelX ?? parsed.ax,
                accelY: parsed.accelY ?? parsed.ay,
                accelZ: parsed.accelZ ?? parsed.az,
                gyroRoll: parsed.gyroRoll ?? parsed.gx,
                gyroPitch: parsed.gyroPitch ?? parsed.gy,
                gyroYaw: parsed.gyroYaw ?? parsed.gz,
                altitude: parsed.altitude ?? parsed.alt
              });
            } else if (trimmed.startsWith('IMU,') || trimmed.startsWith('MPU,')) {
              // CSV line: IMU,roll,pitch,yaw,ax,ay,az,alt
              const p = trimmed.split(',');
              this.injectHardwareTelemetry({
                roll: parseFloat(p[1]),
                pitch: parseFloat(p[2]),
                yaw: parseFloat(p[3]),
                accelX: parseFloat(p[4]),
                accelY: parseFloat(p[5]),
                accelZ: parseFloat(p[6]),
                altitude: p[7] ? parseFloat(p[7]) : undefined
              });
            }
          } catch (e) {}
        }
      }
    } catch (err) {
      console.warn("Serial connection canceled or failed:", err);
    }
  }

  /**
   * Transition airborne flight status with automatic timer start/freeze & origin zeroing
   */
  setAirborne(airborne) {
    const was = this.isAirborne;
    this.isAirborne = !!airborne;
    
    if (this.isAirborne && !was) {
      // Transition to Takeoff / Airborne
      this.lastFlightTimerTick = performance.now();
      this.airborneStartPerf = performance.now();
      this.airborneStartTime = Date.now();
      this.currentSortieFlightTime = 0.0;

      // If a previous flight in this session already landed, start a fresh new sortie
      const hasPreviousLandedFlight = this.currentSortieLogs && this.currentSortieLogs.some(s => s.flightEvent === 'LANDED');
      if (hasPreviousLandedFlight) {
        this.startNewSortie(`Flight Sortie #${this.sortieCount + 1}`);
      } else {
        this.resetCoordinates(0, 0, 0);
      }

      if (this.isLogging) {
        const sample = this._createSampleObject(this.currentIMU, 'TAKEOFF');
        this._pushSample(sample);
      }
    } else if (!this.isAirborne && was) {
      // Transition to Touchdown / Landed
      if (this.isLogging) {
        const sample = this._createSampleObject(this.currentIMU, 'LANDED');
        this._pushSample(sample);
      }
    }
  }

  _tickFlightTimer() {
    const now = performance.now();
    const dt = (now - this.lastFlightTimerTick) / 1000;
    this.lastFlightTimerTick = now;

    // Elapsed flight time ONLY increments when drone is actively flying / airborne
    if (this.isAirborne && dt > 0 && dt < 1.0) {
      this.currentSortieFlightTime += dt;
      this.totalSessionFlightTime += dt;
    }
  }

  /**
   * Evaluates if telemetry has changed significantly enough to record a clean new row (2 Hz when flying)
   */
  shouldLogChange(sample) {
    if (!this.lastLoggedSample) {
      sample.flightEvent = 'INITIALIZE';
      return true;
    }

    const prev = this.lastLoggedSample;
    const now = sample.timestampEpoch;
    let eventName = '';

    // 1. Airborne State Transition (critical event — bypass rate limit, log immediately)
    if (sample.flightStatus !== prev.flightStatus) {
      sample.flightEvent = sample.flightStatus === 'AIRBORNE' ? 'TAKEOFF' : 'LANDED';
      return true;
    }

    // Rate limit: strictly 2 samples per second (500ms minimum interval)
    if (now - this.lastLogTimestamp < 500) return false;

    // 2. Hardware / Mode Triggers
    if (sample.gyroCalibrating && !prev.gyroCalibrating) {
      eventName = 'GYRO_CALIBRATE';
    } else if (sample.stuntFlipping && !prev.stuntFlipping) {
      eventName = 'STUNT_FLIP';
    } else if (sample.gearPct !== prev.gearPct) {
      eventName = `GEAR_${sample.gearPct}%`;
    }
    // 3. Significant Stick Channel Movement (delta >= 3 out of 255)
    else if (Math.abs(sample.rawRoll - prev.rawRoll) >= 3 ||
             Math.abs(sample.rawPitch - prev.rawPitch) >= 3 ||
             Math.abs(sample.rawThrottle - prev.rawThrottle) >= 3 ||
             Math.abs(sample.rawYaw - prev.rawYaw) >= 3) {
      eventName = 'STICK_INPUT';
    }
    // 4. Significant Attitude Tilt Change (delta >= 0.7 deg)
    else if (Math.abs(sample.rollDeg - prev.rollDeg) >= 0.7 ||
             Math.abs(sample.pitchDeg - prev.pitchDeg) >= 0.7 ||
             Math.abs(sample.yawDeg - prev.yawDeg) >= 1.5) {
      eventName = 'ATTITUDE_CHANGE';
    }
    // 5. Significant 3D Position Displacement (delta >= 4 cm)
    else if (Math.abs(sample.posX - prev.posX) >= 0.04 ||
             Math.abs(sample.posY - prev.posY) >= 0.04 ||
             Math.abs(sample.posZ - prev.posZ) >= 0.04) {
      eventName = 'POSITION_UPDATE';
    }
    // 6. Airborne Continuous 2Hz Telemetry Stream (every 500ms during flight)
    else if (this.isAirborne && (now - this.lastLogTimestamp >= 500)) {
      eventName = this.groundSpeed > 0.05 ? 'CRUISE' : 'HOVER';
    }
    // 7. Periodic Idle Heartbeat (every 10s when stationary on ground)
    else if (!this.isAirborne && (now - this.lastLogTimestamp >= 10000)) {
      eventName = 'IDLE_HEARTBEAT';
    }

    if (eventName) {
      sample.flightEvent = eventName;
      return true;
    }
    return false;
  }

  _createSampleObject(src, forcedEvent = '') {
    const now = Date.now();
    const iso = new Date(now).toISOString();
    const d = new Date(now);
    const pad = (n, l = 2) => String(n).padStart(l, '0');
    const localTime = `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${pad(d.getMilliseconds(), 3)}`;

    return {
      timestampIso: iso,
      timestampEpoch: now,
      localTime: localTime,
      flightTime: parseFloat(this.currentSortieFlightTime.toFixed(2)),
      flightDuration: this.formatDuration(this.currentSortieFlightTime),
      totalFlightTime: parseFloat(this.totalSessionFlightTime.toFixed(2)),
      flightEvent: forcedEvent || src.flightEvent || (this.isAirborne ? 'AIRBORNE' : 'GROUND'),
      flightStatus: this.isAirborne ? 'AIRBORNE' : 'GROUND',

      // 3D Local Coordinates (0,0,0 Origin)
      posX: parseFloat(this.posX.toFixed(3)),
      posY: parseFloat(this.posY.toFixed(3)),
      posZ: parseFloat(this.posZ.toFixed(3)),
      velX: parseFloat(this.velX.toFixed(2)),
      velY: parseFloat(this.velY.toFixed(2)),
      velZ: parseFloat(this.velZ.toFixed(2)),
      groundSpeed: parseFloat(this.groundSpeed.toFixed(2)),
      groundSpeedKmh: parseFloat((this.groundSpeed * 3.6).toFixed(1)),
      distanceToHome: parseFloat(this.distanceToHome.toFixed(2)),
      totalDistanceTraveled: parseFloat(this.totalDistanceTraveled.toFixed(2)),

      // Attitude (deg)
      rollDeg: typeof src.rollDeg === 'number' ? src.rollDeg : 0,
      pitchDeg: typeof src.pitchDeg === 'number' ? src.pitchDeg : 0,
      yawDeg: typeof src.yawDeg === 'number' ? src.yawDeg : 0,

      // Gyro Rates (deg/s)
      gyroRollRate: typeof src.gyroRollRate === 'number' ? src.gyroRollRate : 0,
      gyroPitchRate: typeof src.gyroPitchRate === 'number' ? src.gyroPitchRate : 0,
      gyroYawRate: typeof src.gyroYawRate === 'number' ? src.gyroYawRate : 0,

      // Accelerometer (g)
      accelX: typeof src.accelX === 'number' ? src.accelX : 0,
      accelY: typeof src.accelY === 'number' ? src.accelY : 0,
      accelZ: typeof src.accelZ === 'number' ? src.accelZ : 1.0,
      totalG: typeof src.totalG === 'number' ? src.totalG : 1.0,

      // Altitude
      altitudePct: src.altitudePct || 0,
      altitudeMeters: parseFloat(this.posZ.toFixed(2)),
      climbRateMs: parseFloat(this.velZ.toFixed(2)),

      // Raw Hardware Channels (1..255)
      rawRoll: src.rawRoll !== undefined ? src.rawRoll : 128,
      rawPitch: src.rawPitch !== undefined ? src.rawPitch : 128,
      rawThrottle: src.rawThrottle !== undefined ? src.rawThrottle : 128,
      rawYaw: src.rawYaw !== undefined ? src.rawYaw : 128,

      // Modes & Trims
      rollTrim: src.rollTrim || 0,
      pitchTrim: src.pitchTrim || 0,
      yawTrim: src.yawTrim || 0,
      gearPct: src.gearPct || 60,
      altHold: src.altHold !== undefined ? src.altHold : 1,
      headless: src.headless || 0,
      gyroCalibrating: src.gyroCalibrating || 0,
      stuntFlipping: src.stuntFlipping || 0,
      deviceType: src.deviceType || 'GL-21B',
      latencyMs: src.latencyMs || 8,
      packetsSent: src.packetsSent || 0,
      packetsRecv: src.packetsRecv || 0
    };
  }

  /**
   * Updates IMU math, kinematics, and dead-reckoning coordinates from incoming telemetry
   */
  update(telemetry = {}) {
    const now = performance.now();
    const dt = (typeof telemetry.dt === 'number' && telemetry.dt > 0)
      ? telemetry.dt
      : Math.max(0.001, (now - this.lastSampleTime) / 1000);
    this.lastSampleTime = now;

    const rawRoll = typeof telemetry.roll === 'number' ? telemetry.roll : 128;
    const rawPitch = typeof telemetry.pitch === 'number' ? telemetry.pitch : 128;
    const rawThrottle = typeof telemetry.throttle === 'number' ? telemetry.throttle : 128;
    const rawYaw = typeof telemetry.yaw === 'number' ? telemetry.yaw : 128;

    // Strict Airborne Status Management:
    // Drone is airborne ONLY when explicitly initiated via Takeoff (button/key), Mission Runner, or Physical Sensor.
    // Stick or throttle movement on the ground does NOT prematurely start flight or accumulate distance.
    if (typeof telemetry.is_airborne === 'boolean') {
      if (telemetry.is_airborne && !this.isAirborne) {
        this.setAirborne(true);
      } else if (!telemetry.is_airborne && this.isAirborne) {
        this.setAirborne(false);
      }
    } else if (telemetry.is_fast_fly) {
      if (!this.isAirborne) this.setAirborne(true);
    } else if (telemetry.is_emergency_stop || telemetry.is_fast_drop) {
      if (this.isAirborne) this.setAirborne(false);
    }

    const isMotorsSpinning = this.isAirborne;

    // 1. Continuous 6-DOF Attitude & Angular Dynamics with Strict Deadband
    // Eliminates phantom ADC noise and stick drift when hovering in neutral
    const rawOffsetRoll = rawRoll - 128;
    const rawOffsetPitch = rawPitch - 128;
    const rawOffsetYaw = rawYaw - 128;

    const deadbandRoll = Math.abs(rawOffsetRoll) <= 8 ? 0 : (rawOffsetRoll > 0 ? rawOffsetRoll - 8 : rawOffsetRoll + 8);
    const deadbandPitch = Math.abs(rawOffsetPitch) <= 8 ? 0 : (rawOffsetPitch > 0 ? rawOffsetPitch - 8 : rawOffsetPitch + 8);
    const deadbandYaw = Math.abs(rawOffsetYaw) <= 8 ? 0 : (rawOffsetYaw > 0 ? rawOffsetYaw - 8 : rawOffsetYaw + 8);

    const normPitch = Math.max(-1.0, Math.min(1.0, deadbandPitch / 87.0));
    const normRoll = Math.max(-1.0, Math.min(1.0, deadbandRoll / 87.0));
    const normYaw = Math.max(-1.0, Math.min(1.0, deadbandYaw / 87.0));

    // Dynamic target angles and yaw angular rate (calibrated to max 28.5° quadcopter tilt)
    const targetPitch = normPitch * 28.5;
    const targetRoll = normRoll * 28.5;
    const targetYawRate = normYaw * 120.0; // 120.0 deg/s max rotation rate

    // Low-pass response modeling quadcopter rotational inertia & aerodynamic damping
    const alphaAttitude = Math.min(1.0, dt * 10.0);
    const alphaYaw = Math.min(1.0, dt * 12.0);

    this.curPitchDeg += (targetPitch - this.curPitchDeg) * alphaAttitude;
    this.curRollDeg += (targetRoll - this.curRollDeg) * alphaAttitude;
    this.curGyroYaw += (targetYawRate - this.curGyroYaw) * alphaYaw;

    // Zero out residual micro-angles below threshold to maintain steady hover
    if (Math.abs(this.curPitchDeg) < 0.05) this.curPitchDeg = 0.0;
    if (Math.abs(this.curRollDeg) < 0.05) this.curRollDeg = 0.0;
    if (Math.abs(this.curGyroYaw) < 0.05) this.curGyroYaw = 0.0;

    // Integrate continuous heading (persists rotation across maneuvers)
    if (this.isAirborne || Math.abs(this.curGyroYaw) > 0.05) {
      this.curYawDeg = (this.curYawDeg + this.curGyroYaw * dt + 360) % 360;
    }

    let rollDeg = parseFloat(this.curRollDeg.toFixed(2));
    let pitchDeg = parseFloat(this.curPitchDeg.toFixed(2));
    let yawDeg = parseFloat(this.curYawDeg.toFixed(2));

    // If external physical hardware IMU sensor is transmitting, override with real hardware values!
    if (this.hardwareSensorActive && this.externalSensorData) {
      if (typeof this.externalSensorData.roll === 'number') { rollDeg = this.externalSensorData.roll; this.curRollDeg = rollDeg; }
      if (typeof this.externalSensorData.pitch === 'number') { pitchDeg = this.externalSensorData.pitch; this.curPitchDeg = pitchDeg; }
      if (typeof this.externalSensorData.yaw === 'number') { yawDeg = this.externalSensorData.yaw; this.curYawDeg = yawDeg; }
    }

    // 2. Continuous Angular Velocities (Gyroscope Rates in deg/s)
    let gyroRollRate = parseFloat(((this.curRollDeg - this.prevRoll) / dt).toFixed(2));
    let gyroPitchRate = parseFloat(((this.curPitchDeg - this.prevPitch) / dt).toFixed(2));
    let gyroYawRate = parseFloat(this.curGyroYaw.toFixed(2));

    // 3. Accelerometer & G-Force Load
    const rollRad = (this.curRollDeg * Math.PI) / 180;
    const pitchRad = (this.curPitchDeg * Math.PI) / 180;
    const yawRad = (this.curYawDeg * Math.PI) / 180;
    
    // Natural micro-vibrations from motor spin
    const motorVibX = isMotorsSpinning ? (Math.sin(now * 0.035) * 0.006 + (Math.random() - 0.5) * 0.004) : 0;
    const motorVibY = isMotorsSpinning ? (Math.cos(now * 0.041) * 0.006 + (Math.random() - 0.5) * 0.004) : 0;
    const motorVibZ = isMotorsSpinning ? (Math.sin(now * 0.048) * 0.008 + (Math.random() - 0.5) * 0.005) : 0;

    const thrOffsetG = (rawThrottle - 128) / 127.0;
    const verticalThrustG = isMotorsSpinning ? Math.max(0.85, 1.0 + thrOffsetG * 0.20) : 1.0;
    let accelX = parseFloat((Math.sin(pitchRad) + motorVibX).toFixed(3));
    let accelY = parseFloat((-Math.sin(rollRad) * Math.cos(pitchRad) + motorVibY).toFixed(3));
    let accelZ = parseFloat((Math.cos(rollRad) * Math.cos(pitchRad) * verticalThrustG + motorVibZ).toFixed(3));

    if (this.hardwareSensorActive && this.externalSensorData) {
      if (typeof this.externalSensorData.accelX === 'number') accelX = this.externalSensorData.accelX;
      if (typeof this.externalSensorData.accelY === 'number') accelY = this.externalSensorData.accelY;
      if (typeof this.externalSensorData.accelZ === 'number') accelZ = this.externalSensorData.accelZ;
      if (typeof this.externalSensorData.gyroRoll === 'number') gyroRollRate = this.externalSensorData.gyroRoll;
      if (typeof this.externalSensorData.gyroPitch === 'number') gyroPitchRate = this.externalSensorData.gyroPitch;
      if (typeof this.externalSensorData.gyroYaw === 'number') gyroYawRate = this.externalSensorData.gyroYaw;
    }

    const totalG = parseFloat(Math.sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ).toFixed(2));

    // 4. Realistic Optical-Flow Altitude Hold & Climb Dynamics
    let maxLinearSpeed = 0.65; // m/s (Gear 2 baseline ~0.65 m/s)
    if (telemetry.gear === 1) maxLinearSpeed = 0.35;
    else if (telemetry.gear === 3) maxLinearSpeed = 1.05;

    const isFlightActive = this.isAirborne || (rawThrottle > 135 || Math.abs(this.curPitchDeg) > 0.4 || Math.abs(this.curRollDeg) > 0.4);

    if (this.hardwareSensorActive && this.externalSensorData && typeof this.externalSensorData.altitude === 'number') {
      this.posZ = parseFloat(this.externalSensorData.altitude.toFixed(3));
      this.velZ = parseFloat(((this.posZ - this.prevAltitude) / dt).toFixed(2));
    } else if (isFlightActive) {
      // Altitude Hold Optical Flow Dynamic Model:
      // Neutral deadband: 116..140 holds steady hover altitude (0.75m default on takeoff)
      // Throttle > 140 commands smooth vertical climb
      // Throttle < 116 commands smooth descent
      let targetVz = 0.0;
      if (rawThrottle > 140) {
        targetVz = ((rawThrottle - 140) / 115.0) * 0.55; // max +0.55 m/s climb
      } else if (rawThrottle < 116 && rawThrottle > 30) {
        targetVz = -((116 - rawThrottle) / 86.0) * 0.45; // max -0.45 m/s descent
      }

      this.velZ = parseFloat((this.velZ + (targetVz - this.velZ) * Math.min(1.0, dt * 5.0)).toFixed(2));
      if (Math.abs(this.velZ) < 0.01) this.velZ = 0.0;

      if (this.posZ < 0.75 && (this.isAirborne || rawThrottle > 135)) {
        this.posZ = Math.min(0.75, this.posZ + 0.45 * dt);
      } else if (this.velZ !== 0.0) {
        this.posZ = Math.min(3.5, Math.max(0.05, this.posZ + this.velZ * dt));
      }
    } else {
      this.velZ = 0.0;
      this.posZ = 0.0;
    }

    const altitudePct = Math.min(100, Math.max(0, Math.round((this.posZ / 3.0) * 100)));
    const climbRateMs = this.velZ;

    // 5. 3D Dead-Reckoning Position Propagation with Aerodynamic Drag
    if (isFlightActive) {
      // Body-frame target velocities calculated from tilt
      const targetBodyFwd = (this.curPitchDeg / 28.5) * maxLinearSpeed;
      const targetBodyStrafe = (this.curRollDeg / 28.5) * maxLinearSpeed;

      // World-frame target velocities via integrated yaw heading
      const targetVelX = (targetBodyStrafe * Math.cos(yawRad)) + (targetBodyFwd * Math.sin(yawRad));
      const targetVelY = (-targetBodyStrafe * Math.sin(yawRad)) + (targetBodyFwd * Math.cos(yawRad));

      // Aerodynamic velocity response & optical flow braking when sticks are neutral
      const accelRate = (Math.abs(normPitch) > 0 || Math.abs(normRoll) > 0) ? 6.0 : 8.0;
      this.velX += (targetVelX - this.velX) * Math.min(1.0, dt * accelRate);
      this.velY += (targetVelY - this.velY) * Math.min(1.0, dt * accelRate);

      if (Math.abs(this.velX) < 0.01) this.velX = 0.0;
      if (Math.abs(this.velY) < 0.01) this.velY = 0.0;

      this.groundSpeed = parseFloat(Math.sqrt(this.velX * this.velX + this.velY * this.velY).toFixed(2));

      // Integrate position displacements only if actually moving
      if (Math.abs(this.velX) > 0 || Math.abs(this.velY) > 0) {
        this.posX += this.velX * dt;
        this.posY += this.velY * dt;
        this.totalDistanceTraveled += Math.sqrt(this.velX * this.velX + this.velY * this.velY + this.velZ * this.velZ) * dt;
      }

      // Distance from Home (0,0,0)
      this.distanceToHome = parseFloat(Math.sqrt(this.posX * this.posX + this.posY * this.posY + this.posZ * this.posZ).toFixed(2));

      // Add breadcrumb point to radar trail (throttled every 50ms)
      const lastPt = this.pathTrail[this.pathTrail.length - 1];
      const distFromLast = lastPt ? Math.hypot(this.posX - lastPt.x, this.posY - lastPt.y) : 1;
      if (distFromLast > 0.03 || this.pathTrail.length === 0) {
        this.pathTrail.push({
          x: parseFloat(this.posX.toFixed(3)),
          y: parseFloat(this.posY.toFixed(3)),
          z: parseFloat(this.posZ.toFixed(3)),
          heading: yawDeg,
          time: parseFloat(this.currentSortieFlightTime.toFixed(1))
        });
        if (this.pathTrail.length > this.maxTrailPoints) {
          this.pathTrail.shift();
        }
      }
    } else {
      this.velX = 0.0;
      this.velY = 0.0;
      this.velZ = 0.0;
      this.groundSpeed = 0.0;
      this.posZ = 0.0;
      this.distanceToHome = parseFloat(Math.sqrt(this.posX * this.posX + this.posY * this.posY).toFixed(2));
    }

    // Store previous
    this.prevRoll = rollDeg;
    this.prevPitch = pitchDeg;
    this.prevYaw = yawDeg;
    this.prevAltitude = this.posZ;

    const sample = this._createSampleObject({
      rollDeg, pitchDeg, yawDeg,
      gyroRollRate, gyroPitchRate, gyroYawRate,
      accelX, accelY, accelZ, totalG,
      altitudePct,
      rawRoll, rawPitch, rawThrottle, rawYaw,
      rollTrim: telemetry.roll_trim || telemetry.rollTrim || 0,
      pitchTrim: telemetry.pitch_trim || telemetry.pitchTrim || 0,
      yawTrim: telemetry.yaw_trim || telemetry.yawTrim || 0,
      gearPct: telemetry.gear === 1 ? 30 : telemetry.gear === 3 ? 100 : 60,
      altHold: telemetry.is_fixed_height !== undefined ? (telemetry.is_fixed_height ? 1 : 0) : 1,
      headless: telemetry.is_no_head_mode ? 1 : 0,
      gyroCalibrating: telemetry.is_gyro_correction ? 1 : 0,
      stuntFlipping: telemetry.is_circle_turn_end || telemetry.isFlipping ? 1 : 0,
      deviceType: telemetry.device_type || 'GL-21B',
      latencyMs: telemetry.latency || 8,
      packetsSent: telemetry.packets_sent || 0,
      packetsRecv: telemetry.packets_received || 0
    });

    this.currentIMU = sample;

    // Push into rolling waveform history buffer
    this.history.time.push(sample.flightTime);
    this.history.roll.push(sample.rollDeg);
    this.history.pitch.push(sample.pitchDeg);
    this.history.yaw.push(sample.yawDeg);
    this.history.accelZ.push(sample.accelZ);
    this.history.totalG.push(sample.totalG);
    this.history.gyroRoll.push(sample.gyroRollRate);
    this.history.gyroPitch.push(sample.gyroPitchRate);
    this.history.gyroYaw.push(sample.gyroYawRate);
    this.history.posX.push(sample.posX);
    this.history.posY.push(sample.posY);
    this.history.posZ.push(sample.posZ);

    while (this.history.roll.length > this.historyLen) {
      this.history.time.shift();
      this.history.roll.shift();
      this.history.pitch.shift();
      this.history.yaw.shift();
      this.history.accelZ.shift();
      this.history.totalG.shift();
      this.history.gyroRoll.shift();
      this.history.gyroPitch.shift();
      this.history.gyroYaw.shift();
      this.history.posX.shift();
      this.history.posY.shift();
      this.history.posZ.shift();
    }

    // Append to Flight Data Logger Buffer if recording is active AND change is detected!
    if (this.isLogging && this.shouldLogChange(sample)) {
      this._pushSample(sample);
    }

    this.updateUI();

    return sample;
  }

  /**
   * Initialize DOM event listeners for tabs, buttons, search, and exports
   */
  initDOM() {
    // Tab switching
    document.querySelectorAll('.v-imu-tab-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const tabId = btn.getAttribute('data-tab');
        this.switchTab(tabId);
      });
    });

    // Zero coordinates buttons
    const btnZero1 = document.getElementById('btnZeroCoords');
    if (btnZero1) btnZero1.addEventListener('click', () => this.resetCoordinates(0, 0, 0));
    
    const btnZeroHeader = document.getElementById('btnZeroCoordsHeader');
    if (btnZeroHeader) btnZeroHeader.addEventListener('click', () => this.resetCoordinates(0, 0, 0));

    const btnZeroCockpit = document.getElementById('btnCockpitZero');
    if (btnZeroCockpit) btnZeroCockpit.addEventListener('click', () => this.resetCoordinates(0, 0, 0));

    // Connect real physical hardware sensor (ESP32, Arduino, MPU-6050 via USB)
    const btnConnectSerial = document.getElementById('btnConnectSerialSensor');
    if (btnConnectSerial) btnConnectSerial.addEventListener('click', () => this.connectSerialSensor());

    // Export buttons
    const btnCsv = document.getElementById('btnExportCsv');
    if (btnCsv) btnCsv.addEventListener('click', () => this.downloadCSV('current'));

    const btnCsvAll = document.getElementById('btnExportAllCsv');
    if (btnCsvAll) btnCsvAll.addEventListener('click', () => this.downloadCSV('all'));

    const btnCsvHeader = document.getElementById('btnExportExcelHeader');
    if (btnCsvHeader) btnCsvHeader.addEventListener('click', () => this.downloadCSV('current'));

    const btnJson = document.getElementById('btnExportJson');
    if (btnJson) btnJson.addEventListener('click', () => this.downloadJSON());

    const btnServerCsv = document.getElementById('btnExportServerCsv');
    if (btnServerCsv) {
      btnServerCsv.addEventListener('click', () => {
        window.open('/api/imu/export-csv', '_blank');
      });
    }

    // Clear and toggle logging
    const btnClear = document.getElementById('btnClearImuLogs');
    if (btnClear) {
      btnClear.addEventListener('click', () => {
        this.clearLogs();
        fetch('/api/imu/clear', { method: 'POST' }).catch(() => {});
        this.renderLogsTable();
      });
    }

    const btnToggle = document.getElementById('btnToggleLogging');
    if (btnToggle) {
      btnToggle.addEventListener('click', () => {
        const active = this.toggleLogging();
        btnToggle.textContent = active ? 'Pause Recording' : 'Resume Recording';
        const pill = document.getElementById('imuLogStatusPill');
        if (pill) {
          pill.textContent = active ? 'LOGGING ACTIVE' : 'LOGGING PAUSED';
          pill.className = `v-tag ${active ? 'v-tag-rec' : 'v-tag-mono'}`;
        }
      });
    }

    // Search input
    const searchInput = document.getElementById('imuLogSearch');
    if (searchInput) {
      searchInput.addEventListener('input', (e) => {
        this.logSearchTerm = (e.target.value || '').toLowerCase().trim();
        this.renderLogsTable();
      });
    }

    // Waveform channel toggles
    document.querySelectorAll('.wave-toggle-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const ch = btn.getAttribute('data-channel');
        if (ch && this.activeWaveChannels[ch] !== undefined) {
          this.activeWaveChannels[ch] = !this.activeWaveChannels[ch];
          btn.classList.toggle('active', this.activeWaveChannels[ch]);
        }
      });
    });
  }

  switchTab(tabId) {
    this.currentTab = tabId;
    document.querySelectorAll('.v-imu-tab-btn').forEach(b => {
      b.classList.toggle('active', b.getAttribute('data-tab') === tabId);
    });
    document.querySelectorAll('.v-imu-tab-pane').forEach(p => {
      p.classList.toggle('active', p.id === tabId);
    });

    if (tabId === 'tab-logs') {
      this.renderLogsTable();
    }
  }

  /**
   * Formats seconds into MM:SS.s string
   */
  formatDuration(seconds) {
    const s = Math.max(0, seconds || 0);
    const mins = Math.floor(s / 60);
    const secs = (s % 60).toFixed(1);
    const padSecs = secs < 10 ? `0${secs}` : secs;
    return `${mins}:${padSecs}`;
  }

  /**
   * Formats seconds into HH:MM:SS string
   */
  formatDurationLong(seconds) {
    const s = Math.max(0, seconds || 0);
    const hrs = Math.floor(s / 3600);
    const mins = Math.floor((s % 3600) / 60);
    const secs = Math.floor(s % 60);
    const pad = (n) => String(n).padStart(2, '0');
    return `${pad(hrs)}:${pad(mins)}:${pad(secs)}`;
  }

  /**
   * Clear in-memory log buffer, wipe historical samples, and reset all reference counters to 0.0
   */
  clearLogs() {
    this.currentSortieLogs = [];
    this.allSessionLogs = [];
    this.sessionSorties = [];
    this.logs = this.currentSortieLogs;
    this.sortieCount = 1;
    this.currentSortieName = 'Flight Sortie #1';
    this.currentSortieFlightTime = 0.0;
    this.totalDistanceTraveled = 0.0;
    this.totalSessionFlightTime = 0.0;
    this.isAirborne = false;
    this.lastLoggedSample = null;
    this.lastLogTimestamp = 0;
    this.resetCoordinates(0, 0, 0, 0);

    // Clear Waveform history arrays
    for (const key in this.history) {
      if (Array.isArray(this.history[key])) {
        this.history[key] = [];
      }
    }

    const badgeSortie = document.getElementById('imuSortieBadge');
    if (badgeSortie) {
      badgeSortie.textContent = `Sortie #1: Flight Sortie #1`;
    }

    this.renderLogsTable();
    this.updateUI();
    console.log('[IMU Logger] Complete telemetry buffer and flight logs wiped clean.');
  }

  /**
   * Toggle recording state
   */
  toggleLogging() {
    this.isLogging = !this.isLogging;
    return this.isLogging;
  }

  /**
   * Generate RFC 4180 compliant CSV file string containing all collected IMU hardware data & 3D coordinates.
   * Enforces strict missing-data policy: outputs NaN for dropped/missing sensor fields, never fake mock data.
   * @param {string} mode - 'current' (default, exports only active flight simulation) or 'all' (exports entire multi-sortie session)
   */
  exportCSV(mode = 'current') {
    const isCurrent = mode === 'current';
    const targetArray = isCurrent 
      ? (this.currentSortieLogs && this.currentSortieLogs.length > 0 ? this.currentSortieLogs : [])
      : (this.allSessionLogs && this.allSessionLogs.length > 0 ? this.allSessionLogs : []);

    if (targetArray.length === 0) {
      console.error('[ERROR] IMU telemetry offline / no flight samples recorded for export');
      return null;
    }

    const samples = targetArray;
    
    // Header summary comments
    const now = new Date();
    const headers = [
      `# RC UFO Drone Ground Control Station — 6-DOF IMU & 3D Telemetry Flight Log`,
      `# Flight Sortie: ${isCurrent ? `${this.currentSortieName} (Sortie #${this.sortieCount})` : `Complete Session (Total Sorties: ${this.sortieCount})`}`,
      `# Export Scope: ${isCurrent ? 'Single Flight Simulation (Active Sortie Isolated)' : 'Complete Session (All Recorded Sorties)'}`,
      `# Export Timestamp: ${now.toISOString()}`,
      `# Reference Origin: Home (0.00m, 0.00m, 0.00m)`,
      `# Current Sortie Flight Time (s): ${this.currentSortieFlightTime.toFixed(2)} (${this.formatDuration(this.currentSortieFlightTime)})`,
      `# Total Session Flight Time (s): ${this.totalSessionFlightTime.toFixed(2)} (${this.formatDurationLong(this.totalSessionFlightTime)})`,
      `# Total Distance Traveled (m): ${this.totalDistanceTraveled.toFixed(2)} m`,
      `# Total Samples Logged (Event & 2Hz Rate): ${samples.length}`,
      `# Sensor Hardware: 6-Axis IMU (Gyro + Accel) + Barometric Optical Flow`,
      `# Missing Data Policy: Strict (Outputs NaN, No Hardcoded Mock Substitutions)`,
      `# =========================================================================`
    ];

    const columns = [
      'sortie_number',
      'sortie_name',
      'timestamp_iso',
      'local_time',
      'flight_time_sec',
      'flight_duration_formatted',
      'total_session_flight_time_sec',
      'flight_status',
      'flight_event',
      'roll_deg',
      'pitch_deg',
      'yaw_deg',
      'gyro_x_dps',
      'gyro_y_dps',
      'gyro_z_dps',
      'acc_x_g',
      'acc_y_g',
      'acc_z_g',
      'total_g_load',
      'pos_x_m',
      'pos_y_m',
      'pos_z_m',
      'vel_x_ms',
      'vel_y_ms',
      'vel_z_ms',
      'ground_speed_ms',
      'ground_speed_kmh',
      'distance_to_home_m',
      'total_distance_m',
      'altitude_pct',
      'raw_roll_channel',
      'raw_pitch_channel',
      'raw_throttle_channel',
      'raw_yaw_channel',
      'roll_trim',
      'pitch_trim',
      'yaw_trim',
      'speed_gear_pct',
      'altitude_hold_active',
      'headless_active',
      'gyro_calibrating',
      'stunt_flip_active',
      'protocol_type',
      'latency_ms',
      'packets_sent',
      'packets_received'
    ];

    const vNum = (v) => (v !== undefined && v !== null && !isNaN(v)) ? v : 'NaN';

    const rows = samples.map(s => [
      s.sortieNumber !== undefined ? s.sortieNumber : this.sortieCount,
      `"${s.sortieName || this.currentSortieName}"`,
      `"${s.timestampIso || ''}"`,
      `"${s.localTime || ''}"`,
      vNum(s.flightTime),
      `"${s.flightDuration || this.formatDuration(s.flightTime)}"`,
      vNum(s.totalFlightTime),
      `"${s.flightStatus || 'GROUND'}"`,
      `"${s.flightEvent || 'DATA_POINT'}"`,
      vNum(s.rollDeg),
      vNum(s.pitchDeg),
      vNum(s.yawDeg),
      vNum(s.gyroRollRate),
      vNum(s.gyroPitchRate),
      vNum(s.gyroYawRate),
      vNum(s.accelX),
      vNum(s.accelY),
      vNum(s.accelZ),
      vNum(s.totalG),
      vNum(s.posX),
      vNum(s.posY),
      vNum(s.posZ),
      vNum(s.velX),
      vNum(s.velY),
      vNum(s.velZ),
      vNum(s.groundSpeed),
      vNum(s.groundSpeedKmh),
      vNum(s.distanceToHome),
      vNum(s.totalDistanceTraveled),
      vNum(s.altitudePct),
      vNum(s.rawRoll),
      vNum(s.rawPitch),
      vNum(s.rawThrottle),
      vNum(s.rawYaw),
      vNum(s.rollTrim),
      vNum(s.pitchTrim),
      vNum(s.yawTrim),
      vNum(s.gearPct),
      s.altHold ? 1 : 0,
      s.headless ? 1 : 0,
      s.gyroCalibrating ? 1 : 0,
      s.stuntFlipping ? 1 : 0,
      `"${s.deviceType || 'GL-21B'}"`,
      vNum(s.latencyMs),
      vNum(s.packetsSent),
      vNum(s.packetsRecv)
    ].join(','));

    const csvContent = [...headers, columns.join(','), ...rows].join('\r\n');
    return csvContent;
  }

  /**
   * Triggers client-side browser file download for the CSV (fully compatible with Microsoft Excel & Google Sheets)
   * @param {string} mode - 'current' (default) or 'all'
   */
  downloadCSV(mode = 'current') {
    const csvContent = this.exportCSV(mode);
    if (!csvContent) {
      alert("Cannot export CSV log: No live flight telemetry data recorded for this sortie yet. Please arm the drone or run a mission flight simulation first.");
      return;
    }

    // Prepend UTF-8 BOM (\uFEFF) so Excel opens UTF-8 characters and columns accurately
    const blob = new Blob(['\uFEFF' + csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    
    const now = new Date();
    const pad = (n, l = 2) => String(n).padStart(l, '0');
    const tsStr = `${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
    
    const fname = mode === 'current'
      ? `drone_flight_sortie_${this.sortieCount}_${tsStr}.csv`
      : `drone_flight_all_sorties_${tsStr}.csv`;

    link.setAttribute('href', url);
    link.setAttribute('download', fname);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  }

  /**
   * Triggers client-side browser file download for JSON dataset
   */
  downloadJSON() {
    const data = {
      metadata: {
        title: "RC UFO Drone 6-DOF IMU & 3D Coordinates Dataset",
        referenceOrigin: "Home (0.00m, 0.00m, 0.00m)",
        exportedAt: new Date().toISOString(),
        totalSessionFlightTimeSec: this.totalSessionFlightTime,
        currentSortieFlightTimeSec: this.currentSortieFlightTime,
        totalDistanceTraveledM: this.totalDistanceTraveled,
        samplesCount: this.logs.length
      },
      currentCoordinates: {
        x_east_m: this.posX,
        y_north_m: this.posY,
        z_alt_m: this.posZ,
        distance_to_home_m: this.distanceToHome
      },
      flightTrajectoryTrail: this.pathTrail,
      samples: this.logs
    };
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    
    const now = new Date();
    const pad = (n, l = 2) => String(n).padStart(l, '0');
    const tsStr = `${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
    
    link.setAttribute('href', url);
    link.setAttribute('download', `drone_imu_coordinates_${tsStr}.json`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  }

  /**
   * Synchronize all UI metrics in modals and on the main dashboard
   */
  updateUI() {
    const s = this.currentIMU;

    // 1. Modal Top Stats Strip
    const elSortie = document.getElementById('imuSortieTime');
    if (elSortie) elSortie.textContent = this.formatDuration(this.currentSortieFlightTime);

    const elSession = document.getElementById('imuSessionTime');
    if (elSession) elSession.textContent = this.formatDurationLong(this.totalSessionFlightTime);

    const elTotalG = document.getElementById('imuTotalG');
    if (elTotalG) elTotalG.textContent = `${s.totalG.toFixed(3)} g`;

    const elCount = document.getElementById('imuSampleCount');
    if (elCount) elCount.textContent = this.logs.length.toLocaleString();

    const elDistHome = document.getElementById('imuDistHomeTop');
    if (elDistHome) elDistHome.textContent = `${s.distanceToHome.toFixed(2)} m`;

    const elSpeedTop = document.getElementById('imuSpeedTop');
    if (elSpeedTop) elSpeedTop.textContent = `${s.groundSpeed.toFixed(2)} m/s`;

    // 2. 3D Local Coordinates
    const elPosX = document.getElementById('imuPosX');
    if (elPosX) elPosX.textContent = `${s.posX >= 0 ? '+' : ''}${s.posX.toFixed(2)} m`;
    const elPosXCm = document.getElementById('imuPosXCm');
    if (elPosXCm) elPosXCm.textContent = `${(s.posX * 100).toFixed(1)} cm`;

    const elPosY = document.getElementById('imuPosY');
    if (elPosY) elPosY.textContent = `${s.posY >= 0 ? '+' : ''}${s.posY.toFixed(2)} m`;
    const elPosYCm = document.getElementById('imuPosYCm');
    if (elPosYCm) elPosYCm.textContent = `${(s.posY * 100).toFixed(1)} cm`;

    const elPosZ = document.getElementById('imuPosZ');
    if (elPosZ) elPosZ.textContent = `${s.posZ.toFixed(2)} m`;
    const elPosZCm = document.getElementById('imuPosZCm');
    if (elPosZCm) elPosZCm.textContent = `${(s.posZ * 100).toFixed(1)} cm`;

    const elDistHomeSub = document.getElementById('imuDistHomeSub');
    if (elDistHomeSub) elDistHomeSub.textContent = `${s.distanceToHome.toFixed(2)} m`;

    const elOdo = document.getElementById('imuTotalDistance');
    if (elOdo) elOdo.textContent = `${s.totalDistanceTraveled.toFixed(2)} m`;

    // 3. 6-DOF IMU (Gyro Rates & Accelerations)
    const elGyroRoll = document.getElementById('imuGyroRoll');
    if (elGyroRoll) elGyroRoll.textContent = `${s.gyroRollRate >= 0 ? '+' : ''}${s.gyroRollRate.toFixed(1)} °/s`;
    const elGyroPitch = document.getElementById('imuGyroPitch');
    if (elGyroPitch) elGyroPitch.textContent = `${s.gyroPitchRate >= 0 ? '+' : ''}${s.gyroPitchRate.toFixed(1)} °/s`;
    const elGyroYaw = document.getElementById('imuGyroYaw');
    if (elGyroYaw) elGyroYaw.textContent = `${s.gyroYawRate >= 0 ? '+' : ''}${s.gyroYawRate.toFixed(1)} °/s`;

    const elAccelX = document.getElementById('imuAccelX');
    if (elAccelX) elAccelX.textContent = `${s.accelX >= 0 ? '+' : ''}${s.accelX.toFixed(3)} g`;
    const elAccelY = document.getElementById('imuAccelY');
    if (elAccelY) elAccelY.textContent = `${s.accelY >= 0 ? '+' : ''}${s.accelY.toFixed(3)} g`;
    const elAccelZ = document.getElementById('imuAccelZ');
    if (elAccelZ) elAccelZ.textContent = `${s.accelZ.toFixed(3)} g`;
    const elTotalGCard = document.getElementById('imuTotalGCard');
    if (elTotalGCard) elTotalGCard.textContent = `${s.totalG.toFixed(3)} g`;

    // 4. Odometry & Speed
    const elGroundSpeed = document.getElementById('imuGroundSpeed');
    if (elGroundSpeed) elGroundSpeed.textContent = `${s.groundSpeed.toFixed(2)} m/s`;
    const elGroundSpeedKmh = document.getElementById('imuGroundSpeedKmh');
    if (elGroundSpeedKmh) elGroundSpeedKmh.textContent = `${(s.groundSpeed * 3.6).toFixed(1)} km/h`;

    const elClimbRate = document.getElementById('imuClimbRate');
    if (elClimbRate) elClimbRate.textContent = `${s.velZ >= 0 ? '+' : ''}${s.velZ.toFixed(2)} m/s`;

    const elAltMeters = document.getElementById('imuAltMeters');
    if (elAltMeters) elAltMeters.textContent = `${s.altitudeMeters.toFixed(2)} m`;
    const elAltPct = document.getElementById('imuAltPct');
    if (elAltPct) elAltPct.textContent = `${s.altitudePct}%`;

    // 5. Hardware Channels
    const elRawRoll = document.getElementById('imuRawRoll');
    if (elRawRoll) elRawRoll.textContent = `${s.rawRoll}`;
    const elRawPitch = document.getElementById('imuRawPitch');
    if (elRawPitch) elRawPitch.textContent = `${s.rawPitch}`;
    const elRawThr = document.getElementById('imuRawThr');
    if (elRawThr) elRawThr.textContent = `${s.rawThrottle}`;
    const elRawYaw = document.getElementById('imuRawYaw');
    if (elRawYaw) elRawYaw.textContent = `${s.rawYaw}`;

    const elGear = document.getElementById('imuGear');
    if (elGear) elGear.textContent = `${s.gearPct}%`;
    const elAltHold = document.getElementById('imuAltHold');
    if (elAltHold) elAltHold.textContent = s.altHold ? 'LOCK ON' : 'OFF';

    // 6. Main Dashboard Mini IMU Deck
    const mainPosX = document.getElementById('mainImuPosX');
    if (mainPosX) mainPosX.textContent = `${s.posX >= 0 ? '+' : ''}${s.posX.toFixed(2)}m`;
    const mainPosY = document.getElementById('mainImuPosY');
    if (mainPosY) mainPosY.textContent = `${s.posY >= 0 ? '+' : ''}${s.posY.toFixed(2)}m`;
    const mainPosZ = document.getElementById('mainImuPosZ');
    if (mainPosZ) mainPosZ.textContent = `${s.posZ.toFixed(2)}m`;
    const mainDist = document.getElementById('mainImuDist');
    if (mainDist) mainDist.textContent = `${s.distanceToHome.toFixed(2)}m`;
    const mainSpeed = document.getElementById('mainImuSpeed');
    if (mainSpeed) mainSpeed.textContent = `${s.groundSpeed.toFixed(2)} m/s`;
    const mainGLoad = document.getElementById('mainImuGLoad');
    if (mainGLoad) mainGLoad.textContent = `${s.totalG.toFixed(2)}g`;

    const mainGyroW = document.getElementById('mainImuGyroRates');
    if (mainGyroW) mainGyroW.textContent = `${s.gyroRollRate.toFixed(0)} / ${s.gyroPitchRate.toFixed(0)} / ${s.gyroYawRate.toFixed(0)} °/s`;
    const mainAccelG = document.getElementById('mainImuAccels');
    if (mainAccelG) mainAccelG.textContent = `${s.accelX.toFixed(2)} / ${s.accelY.toFixed(2)} / ${s.accelZ.toFixed(2)} g`;

    // 7. Render Active Canvas Views
    if (this.currentTab === 'tab-radar' || document.getElementById('mission-modal')?.classList.contains('hide') === false) {
      this.renderFlightRadar('imuRadarCanvas');
    }
    if (this.currentTab === 'tab-waveforms') {
      this.renderOscilloscope('imuWaveformCanvas');
    }
  }

  /**
   * Render real-time logs table with search filtering
   */
  /**
   * Render real-time logs table with search filtering
   */
  renderLogsTable() {
    const tbody = document.getElementById('imuLogsTableBody');
    if (!tbody) return;

    let displayLogs = this.logs;
    if (this.logSearchTerm) {
      displayLogs = this.logs.filter(l => 
        (l.timestampIso && l.timestampIso.toLowerCase().includes(this.logSearchTerm)) ||
        (l.localTime && l.localTime.toLowerCase().includes(this.logSearchTerm)) ||
        (l.flightEvent && l.flightEvent.toLowerCase().includes(this.logSearchTerm)) ||
        String(l.posX).includes(this.logSearchTerm) ||
        String(l.posY).includes(this.logSearchTerm) ||
        String(l.deviceType).toLowerCase().includes(this.logSearchTerm)
      );
    }

    const latestLogs = displayLogs.slice(-80).reverse(); // Last 80 points in reverse order

    if (latestLogs.length === 0) {
      tbody.innerHTML = `<tr><td colspan="13" class="table-empty-row">No flight logs recorded in buffer yet. Fly drone or move sticks to record telemetry.</td></tr>`;
      return;
    }

    const getEventBadge = (evt) => {
      if (evt === 'TAKEOFF') return '<span class="badge-tag bg-emerald" style="background:rgba(16,185,129,0.2);color:#34d399;font-weight:700;">TAKEOFF</span>';
      if (evt === 'LANDED') return '<span class="badge-tag bg-red" style="background:rgba(239,68,68,0.2);color:#f87171;font-weight:700;">LANDED</span>';
      if (evt && (evt.startsWith('STEP_') || evt.startsWith('MISSION_'))) return `<span class="badge-tag" style="background:rgba(217,70,239,0.2);color:#e879f9;font-weight:700;">${evt}</span>`;
      if (evt === 'STICK_INPUT') return '<span class="badge-tag" style="background:rgba(0,229,255,0.15);color:#00e5ff;">STICK_INPUT</span>';
      if (evt === 'ATTITUDE_CHANGE') return '<span class="badge-tag" style="background:rgba(245,158,11,0.15);color:#fbbf24;">ATTITUDE</span>';
      if (evt === 'POSITION_UPDATE') return '<span class="badge-tag" style="background:rgba(139,92,246,0.15);color:#a78bfa;">POSITION</span>';
      if (evt === 'FLIGHT_HEARTBEAT' || evt === 'IDLE_HEARTBEAT') return '<span class="badge-tag" style="background:rgba(255,255,255,0.06);color:#888;">HEARTBEAT</span>';
      return `<span class="badge-tag">${evt || 'DATA'}</span>`;
    };

    tbody.innerHTML = latestLogs.map((l) => {
      const timeStr = l.localTime || (l.timestampIso ? l.timestampIso.split('T')[1].replace('Z', '') : '');
      const durationStr = l.flightDuration || this.formatDuration(l.flightTime);
      const isFlying = l.flightStatus === 'AIRBORNE';
      const statusBadge = isFlying 
        ? '<span class="badge-tag" style="background:rgba(16,185,129,0.15);color:#10b981;font-weight:600;">FLYING</span>'
        : '<span class="badge-tag" style="background:rgba(255,255,255,0.05);color:#777;">GROUND</span>';

      return `
        <tr>
          <td class="mono text-muted">${timeStr}</td>
          <td class="mono font-bold ${isFlying ? 'text-cyan' : ''}">${durationStr}</td>
          <td class="mono">${getEventBadge(l.flightEvent)}</td>
          <td class="mono text-cyan">${l.posX >= 0 ? '+' : ''}${l.posX.toFixed(2)}m</td>
          <td class="mono text-green">${l.posY >= 0 ? '+' : ''}${l.posY.toFixed(2)}m</td>
          <td class="mono text-amber">${l.posZ.toFixed(2)}m</td>
          <td class="mono">${l.distanceToHome.toFixed(2)}m</td>
          <td class="mono">${l.rollDeg.toFixed(1)}° / ${l.pitchDeg.toFixed(1)}° / ${l.yawDeg.toFixed(0)}°</td>
          <td class="mono">${l.gyroRollRate.toFixed(1)} / ${l.gyroPitchRate.toFixed(1)}</td>
          <td class="mono">${l.accelX.toFixed(2)} / ${l.accelY.toFixed(2)} / ${l.accelZ.toFixed(2)}</td>
          <td class="mono font-bold text-amber">${l.totalG.toFixed(3)}g</td>
          <td class="mono text-muted">[${l.rawRoll}, ${l.rawPitch}, ${l.rawThrottle}, ${l.rawYaw}]</td>
          <td class="mono">${statusBadge}</td>
        </tr>
      `;
    }).join('');

    const badgeCount = document.getElementById('imuTableCountBadge');
    if (badgeCount) {
      badgeCount.textContent = `${this.logs.length} Logged (Sortie #${this.sortieCount})`;
    }

    const badgeSortie = document.getElementById('imuSortieBadge');
    if (badgeSortie) {
      badgeSortie.textContent = `Sortie #${this.sortieCount}`;
      badgeSortie.title = `Current Sortie: ${this.currentSortieName}`;
    }
  }

  /**
   * Render real-time multi-channel waveform on the oscilloscope canvas
   */
  renderOscilloscope(canvasId) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    
    ctx.clearRect(0, 0, w, h);

    // Grid Background
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
    ctx.lineWidth = 1;
    const gridCols = 8;
    const gridRows = 4;
    for (let i = 1; i < gridCols; i++) {
      const x = (w / gridCols) * i;
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    for (let j = 1; j < gridRows; j++) {
      const y = (h / gridRows) * j;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    // Center Baseline (Zero / 1g line)
    const midY = h / 2;
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(0, midY);
    ctx.lineTo(w, midY);
    ctx.stroke();
    ctx.setLineDash([]);

    const points = this.history.roll.length;
    if (points < 2) return;

    // Helper to draw a single channel wave
    const drawChannel = (dataArray, color, scale, offsetY = midY) => {
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      for (let i = 0; i < points; i++) {
        const x = (i / (this.historyLen - 1)) * w;
        const val = dataArray[i] || 0;
        const y = offsetY - (val * scale);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    };

    // 1. Roll Waveform (Blue) - scale: 1 deg = 1.5px
    if (this.activeWaveChannels.roll) {
      drawChannel(this.history.roll, '#0070f3', 1.5, midY);
    }
    // 2. Pitch Waveform (Amber) - scale: 1 deg = 1.5px
    if (this.activeWaveChannels.pitch) {
      drawChannel(this.history.pitch, '#f59e0b', 1.5, midY);
    }
    // 3. Accel Z (Emerald) - (Az - 1.0) * 30px
    if (this.activeWaveChannels.accelZ) {
      drawChannel(this.history.accelZ.map(v => v - 1.0), '#22c55e', 30.0, midY);
    }
    // 4. Gyro Roll Rate (Purple) - scale: 1 deg/s = 0.5px
    if (this.activeWaveChannels.gyroRoll) {
      drawChannel(this.history.gyroRoll, '#a855f7', 0.5, midY);
    }
  }

  /**
   * Render Top-Down 2D Flight Trajectory Radar with (0,0,0) Origin Marker
   */
  renderFlightRadar(canvasId) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    const cx = w / 2;
    const cy = h / 2;

    ctx.clearRect(0, 0, w, h);

    // Radar Range Rings (Scale: 1 meter = 45 pixels, dynamic range scaling)
    const maxCoord = Math.max(1.5, Math.abs(this.posX), Math.abs(this.posY));
    const scalePxPerMeter = Math.min(cx, cy) / (maxCoord * 1.3);

    // Range rings
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    ctx.lineWidth = 1;
    [0.5, 1.0, 2.0, 3.0, 5.0].forEach(r => {
      const radiusPx = r * scalePxPerMeter;
      if (radiusPx < Math.min(cx, cy) * 1.4) {
        ctx.beginPath();
        ctx.arc(cx, cy, radiusPx, 0, Math.PI * 2);
        ctx.stroke();
        ctx.fillStyle = 'rgba(255, 255, 255, 0.25)';
        ctx.font = '9px "JetBrains Mono", monospace';
        ctx.fillText(`${r}m`, cx + radiusPx + 3, cy - 3);
      }
    });

    // Crosshair axes
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
    ctx.beginPath();
    ctx.moveTo(cx, 10);
    ctx.lineTo(cx, h - 10);
    ctx.moveTo(10, cy);
    ctx.lineTo(w - 10, cy);
    ctx.stroke();

    // Axis labels (N / S / E / W)
    ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.fillText('+Y (NORTH)', cx, 14);
    ctx.fillText('+X (EAST)', w - 30, cy - 4);

    // Origin (0,0,0) HOME Marker
    ctx.fillStyle = '#f59e0b';
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = 'rgba(245, 158, 11, 0.9)';
    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.textAlign = 'left';
    ctx.fillText('HOME (0,0,0)', cx + 7, cy + 3.5);

    // Draw Flight Trajectory Path
    if (this.pathTrail && this.pathTrail.length > 1) {
      ctx.strokeStyle = 'rgba(0, 112, 243, 0.75)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i < this.pathTrail.length; i++) {
        const pt = this.pathTrail[i];
        const px = cx + (pt.x * scalePxPerMeter);
        const py = cy - (pt.y * scalePxPerMeter); // Canvas Y is inverted
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
    }

    // Current Drone Position Marker & Heading Triangle
    const dronePx = cx + (this.posX * scalePxPerMeter);
    const dronePy = cy - (this.posY * scalePxPerMeter);

    // Glow pulse around current location
    ctx.fillStyle = 'rgba(0, 112, 243, 0.25)';
    ctx.beginPath();
    ctx.arc(dronePx, dronePy, 10, 0, Math.PI * 2);
    ctx.fill();

    // Drone Triangle heading indicator
    ctx.save();
    ctx.translate(dronePx, dronePy);
    ctx.rotate((this.prevYaw * Math.PI) / 180);

    ctx.fillStyle = '#0070f3';
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(0, -9);
    ctx.lineTo(6, 6);
    ctx.lineTo(0, 3);
    ctx.lineTo(-6, 6);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();

    // Coordinates Tag beside Drone
    ctx.fillStyle = '#ededed';
    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.textAlign = 'left';
    ctx.fillText(`[X:${this.posX.toFixed(2)}m, Y:${this.posY.toFixed(2)}m, Z:${this.posZ.toFixed(2)}m]`, dronePx + 12, dronePy - 4);
  }
}

// Attach globally
window.DroneIMULogger = DroneIMULogger;
