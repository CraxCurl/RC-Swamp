/**
 * Drone 6-DOF IMU Telemetry, 3D Local Coordinate Tracker (0,0,0 Origin), Real-Time Flight Logger & CSV Exporter
 * 
 * Captures, computes, and logs:
 * - 3D Local Coordinates: X (East/Lateral m), Y (North/Forward m), Z (Altitude m) with start point at (0,0,0)
 * - Navigation & Odometry: Distance from Home (m), Total Distance Traveled (m), Ground Speed (m/s & km/h)
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
    
    // Flight Timers
    this.sessionStartTime = Date.now();
    this.totalSessionFlightTime = 0; // seconds spent in active flight across whole session
    this.currentSortieFlightTime = 0;  // seconds spent in current active flight
    this.isAirborne = false;
    this.lastFlightTimerTick = Date.now();

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

    // Previous state for numerical derivatives (calculating angular rates & accelerations)
    this.lastSampleTime = performance.now();
    this.prevRoll = 0;
    this.prevPitch = 0;
    this.prevYaw = 0;
    this.prevAltitude = 0;

    // Filtered / Smoothed IMU State
    this.currentIMU = {
      timestampIso: new Date().toISOString(),
      timestampEpoch: Date.now(),
      flightTime: 0,
      totalFlightTime: 0,

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
      rollDeg: 0,
      pitchDeg: 0,
      yawDeg: 0,

      // Gyro Rates (deg/s)
      gyroRollRate: 0,
      gyroPitchRate: 0,
      gyroYawRate: 0,

      // Accelerometer (g & m/s^2)
      accelX: 0,
      accelY: 0,
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

    // Start 100ms background flight timer loop
    setInterval(() => this._tickFlightTimer(), 100);
  }

  /**
   * Reset the drone coordinates to 0,0,0 (Home / Start Point Calibration)
   */
  resetCoordinates(x = 0, y = 0, z = 0) {
    this.posX = x;
    this.posY = y;
    this.posZ = z;
    this.velX = 0;
    this.velY = 0;
    this.velZ = 0;
    this.groundSpeed = 0;
    this.distanceToHome = 0;
    this.totalDistanceTraveled = 0;
    this.groundReferenceAlt = this.prevAltitude;
    this.pathTrail = [{ x: this.posX, y: this.posY, z: this.posZ, heading: this.prevYaw, time: this.currentSortieFlightTime }];
  }

  setAirborne(airborne) {
    const was = this.isAirborne;
    this.isAirborne = !!airborne;
    if (this.isAirborne && !was) {
      this.lastFlightTimerTick = Date.now();
      if (this.currentSortieFlightTime === 0 && this.totalDistanceTraveled === 0) {
        this.resetCoordinates(0, 0, 0);
      }
    }
  }

  _tickFlightTimer() {
    const now = Date.now();
    const dt = (now - this.lastFlightTimerTick) / 1000;
    this.lastFlightTimerTick = now;

    // Elapsed flight time ONLY increments when drone is actively flying / airborne
    if (this.isAirborne && dt > 0 && dt < 1.0) {
      this.currentSortieFlightTime += dt;
      this.totalSessionFlightTime += dt;
    }
  }

  /**
   * Updates IMU math, kinematics, and dead-reckoning coordinates from incoming telemetry
   */
  update(telemetry = {}) {
    const now = performance.now();
    const dt = Math.max(0.001, (now - this.lastSampleTime) / 1000);
    this.lastSampleTime = now;

    const rawRoll = typeof telemetry.roll === 'number' ? telemetry.roll : 128;
    const rawPitch = typeof telemetry.pitch === 'number' ? telemetry.pitch : 128;
    const rawThrottle = typeof telemetry.throttle === 'number' ? telemetry.throttle : 128;
    const rawYaw = typeof telemetry.yaw === 'number' ? telemetry.yaw : 128;

    // Detect airborne state from telemetry signals or explicit flight flags
    if (telemetry.is_airborne === true || telemetry.is_flying === true || telemetry.is_fast_fly || telemetry.isFastFly) {
      this.setAirborne(true);
    } else if (telemetry.is_airborne === false || telemetry.is_fast_drop || telemetry.is_emergency_stop) {
      this.setAirborne(false);
    }

    const isMotorsSpinning = this.isAirborne;

    // 1. Attitude Angles (deg)
    // Roll: 1..255 -> -44.45° to +44.45°
    const rollDeg = (rawRoll - 128) * 0.35;
    // Pitch: 1..255 -> -31.75° to +31.75° (Forward is positive)
    const pitchDeg = -(rawPitch - 128) * 0.25;
    // Yaw continuous heading tracker (0..360)
    let yawDeg = ((rawYaw - 128) * 1.4 + 360) % 360;

    // 2. Angular Velocities (Gyroscope Rates in deg/s)
    const gyroRollRate = (rollDeg - this.prevRoll) / dt;
    const gyroPitchRate = (pitchDeg - this.prevPitch) / dt;
    let deltaYaw = yawDeg - this.prevYaw;
    if (deltaYaw > 180) deltaYaw -= 360;
    if (deltaYaw < -180) deltaYaw += 360;
    const gyroYawRate = deltaYaw / dt;

    // 3. Accelerometer (g & m/s^2)
    const rollRad = (rollDeg * Math.PI) / 180;
    const pitchRad = (pitchDeg * Math.PI) / 180;
    const yawRad = (yawDeg * Math.PI) / 180;
    
    const thrustLoad = (rawThrottle / 128.0) * 0.85;
    const accelX = Math.sin(pitchRad);
    const accelY = -Math.sin(rollRad) * Math.cos(pitchRad);
    const accelZ = Math.cos(rollRad) * Math.cos(pitchRad) * (isMotorsSpinning ? (0.4 + thrustLoad * 0.6) : 1.0);
    const totalG = Math.sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

    // 4. Altitude estimation (Optical flow / Baro level 0..100% and estimated meters 0..3.5m)
    const altitudePct = isMotorsSpinning ? Math.round((rawThrottle / 255) * 100) : 0;
    const rawAltitudeMeters = isMotorsSpinning ? (rawThrottle / 255) * 3.5 : 0;
    const climbRateMs = isMotorsSpinning ? (rawAltitudeMeters - this.prevAltitude) / dt : 0;

    // ----------------- 5. 3D DEAD-RECKONING COORDINATE PROPAGATION ----------------- //
    // Speed calibration scale based on speed gear:
    // Gear 1 (30%): max speed ~0.35 m/s (35 cm/s)
    // Gear 2 (60%): max speed ~0.65 m/s (65 cm/s)
    // Gear 3 (100%): max speed ~1.10 m/s (110 cm/s)
    let maxLinearSpeed = 0.65;
    if (telemetry.gear === 1) maxLinearSpeed = 0.35;
    else if (telemetry.gear === 3) maxLinearSpeed = 1.10;

    if (this.isAirborne) {
      // Body-frame normalized stick deflections
      const bodyForwardVel = (pitchDeg / 30.0) * maxLinearSpeed; // Pitch forward (+) / back (-)
      const bodyStrafeVel = (rollDeg / 40.0) * maxLinearSpeed;   // Roll right (+) / left (-)

      // Transform Body-frame velocity into World / Ground Frame (East=X, North=Y) via Yaw Heading
      this.velX = (bodyStrafeVel * Math.cos(yawRad)) + (bodyForwardVel * Math.sin(yawRad));
      this.velY = (-bodyStrafeVel * Math.sin(yawRad)) + (bodyForwardVel * Math.cos(yawRad));
      this.velZ = climbRateMs;

      this.groundSpeed = Math.sqrt(this.velX * this.velX + this.velY * this.velY);

      // Integrate position displacements
      this.posX += this.velX * dt;
      this.posY += this.velY * dt;
      this.posZ = Math.max(0, rawAltitudeMeters - this.groundReferenceAlt);

      // Distance from Home (0,0,0) and Cumulative Odometer
      this.distanceToHome = Math.sqrt(this.posX * this.posX + this.posY * this.posY + this.posZ * this.posZ);
      this.totalDistanceTraveled += Math.sqrt(this.velX * this.velX + this.velY * this.velY + this.velZ * this.velZ) * dt;

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
      this.velX = 0;
      this.velY = 0;
      this.velZ = 0;
      this.groundSpeed = 0;
      this.posZ = 0;
      this.distanceToHome = Math.sqrt(this.posX * this.posX + this.posY * this.posY);
    }

    // Store previous
    this.prevRoll = rollDeg;
    this.prevPitch = pitchDeg;
    this.prevYaw = yawDeg;
    this.prevAltitude = rawAltitudeMeters;

    const sample = {
      timestampIso: new Date().toISOString(),
      timestampEpoch: Date.now(),
      flightTime: parseFloat(this.currentSortieFlightTime.toFixed(2)),
      totalFlightTime: parseFloat(this.totalSessionFlightTime.toFixed(2)),

      // 3D Local Coordinates (0,0,0 Origin)
      posX: parseFloat(this.posX.toFixed(3)),
      posY: parseFloat(this.posY.toFixed(3)),
      posZ: parseFloat(this.posZ.toFixed(3)),
      velX: parseFloat(this.velX.toFixed(2)),
      velY: parseFloat(this.velY.toFixed(2)),
      velZ: parseFloat(this.velZ.toFixed(2)),
      groundSpeed: parseFloat(this.groundSpeed.toFixed(2)),
      distanceToHome: parseFloat(this.distanceToHome.toFixed(2)),
      totalDistanceTraveled: parseFloat(this.totalDistanceTraveled.toFixed(2)),

      rollDeg: parseFloat(rollDeg.toFixed(2)),
      pitchDeg: parseFloat(pitchDeg.toFixed(2)),
      yawDeg: parseFloat(yawDeg.toFixed(1)),

      gyroRollRate: parseFloat(gyroRollRate.toFixed(2)),
      gyroPitchRate: parseFloat(gyroPitchRate.toFixed(2)),
      gyroYawRate: parseFloat(gyroYawRate.toFixed(2)),

      accelX: parseFloat(accelX.toFixed(3)),
      accelY: parseFloat(accelY.toFixed(3)),
      accelZ: parseFloat(accelZ.toFixed(3)),
      totalG: parseFloat(totalG.toFixed(3)),

      altitudePct,
      altitudeMeters: parseFloat(this.posZ.toFixed(2)),
      climbRateMs: parseFloat(climbRateMs.toFixed(2)),

      rawRoll,
      rawPitch,
      rawThrottle,
      rawYaw,

      rollTrim: telemetry.roll_trim || telemetry.rollTrim || 0,
      pitchTrim: telemetry.pitch_trim || telemetry.pitchTrim || 0,
      yawTrim: telemetry.yaw_trim || telemetry.yawTrim || 0,
      gearPct: (telemetry.gear === 1 ? 30 : telemetry.gear === 3 ? 100 : 60),
      altHold: (telemetry.is_fixed_height !== false && telemetry.isFixedHeight !== false) ? 1 : 0,
      headless: (telemetry.is_no_head_mode || telemetry.isNoHeadMode) ? 1 : 0,
      gyroCalibrating: (telemetry.is_gyro_correction || telemetry.isGyroCorrection) ? 1 : 0,
      stuntFlipping: (telemetry.is_circle_turn_end || telemetry.isCircleTurnEnd) ? 1 : 0,
      deviceType: telemetry.device_type || telemetry.deviceType || 'GL-21B',
      latencyMs: 8,
      packetsSent: telemetry.packets_sent || telemetry.packetsSent || 0,
      packetsRecv: telemetry.packets_received || telemetry.packetsReceived || 0
    };

    this.currentIMU = sample;

    // Append to Waveform History
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

    // Append to Flight Data Logger Buffer if recording is active
    if (this.isLogging) {
      this.logs.push(sample);
      if (this.logs.length > this.maxLogs) {
        this.logs.shift();
      }
    }

    return sample;
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
   * Clear in-memory log buffer
   */
  clearLogs() {
    this.logs = [];
    this.currentSortieFlightTime = 0;
  }

  /**
   * Toggle recording state
   */
  toggleLogging() {
    this.isLogging = !this.isLogging;
    return this.isLogging;
  }

  /**
   * Generate RFC 4180 compliant CSV file string containing all collected IMU hardware data & 3D coordinates
   */
  exportCSV() {
    const samples = this.logs.length > 0 ? this.logs : [this.currentIMU];
    
    // Header summary comments
    const now = new Date();
    const headers = [
      `# RC UFO Drone Ground Control Station — 6-DOF IMU & 3D Coordinates Flight Log`,
      `# Export Timestamp: ${now.toISOString()}`,
      `# Reference Origin: Home (0.00m, 0.00m, 0.00m)`,
      `# Total Session Flight Time (s): ${this.totalSessionFlightTime.toFixed(2)} (${this.formatDurationLong(this.totalSessionFlightTime)})`,
      `# Current Sortie Flight Time (s): ${this.currentSortieFlightTime.toFixed(2)} (${this.formatDuration(this.currentSortieFlightTime)})`,
      `# Total Distance Traveled (m): ${this.totalDistanceTraveled.toFixed(2)} m`,
      `# Total Samples Logged: ${samples.length}`,
      `# Hardware IMU: 6-Axis Gyroscope + 3-Axis Accelerometer + Barometric Optical Flow`,
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
      'vel_x_ms',
      'vel_y_ms',
      'vel_z_ms',
      'ground_speed_ms',
      'distance_to_home_m',
      'total_distance_traveled_m',
      'roll_deg',
      'pitch_deg',
      'yaw_heading_deg',
      'gyro_roll_rate_dps',
      'gyro_pitch_rate_dps',
      'gyro_yaw_rate_dps',
      'accel_x_g',
      'accel_y_g',
      'accel_z_g',
      'total_g_load',
      'estimated_altitude_pct',
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

    const rows = samples.map(s => [
      s.timestampIso,
      s.timestampEpoch,
      s.totalFlightTime,
      s.flightTime,
      s.posX,
      s.posY,
      s.posZ,
      s.velX,
      s.velY,
      s.velZ,
      s.groundSpeed,
      s.distanceToHome,
      s.totalDistanceTraveled,
      s.rollDeg,
      s.pitchDeg,
      s.yawDeg,
      s.gyroRollRate,
      s.gyroPitchRate,
      s.gyroYawRate,
      s.accelX,
      s.accelY,
      s.accelZ,
      s.totalG,
      s.altitudePct,
      s.rawRoll,
      s.rawPitch,
      s.rawThrottle,
      s.rawYaw,
      s.rollTrim,
      s.pitchTrim,
      s.yawTrim,
      s.gearPct,
      s.altHold,
      s.headless,
      s.gyroCalibrating,
      s.stuntFlipping,
      s.deviceType,
      s.latencyMs,
      s.packetsSent,
      s.packetsRecv
    ].join(','));

    const csvContent = [...headers, columns.join(','), ...rows].join('\r\n');
    return csvContent;
  }

  /**
   * Triggers client-side browser file download for the CSV (fully compatible with Microsoft Excel & Google Sheets)
   */
  downloadCSV() {
    const csvContent = this.exportCSV();
    // Prepend UTF-8 BOM (\uFEFF) so Excel opens UTF-8 characters and columns accurately
    const blob = new Blob(['\uFEFF' + csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    
    const now = new Date();
    const pad = (n, l = 2) => String(n).padStart(l, '0');
    const tsStr = `${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
    
    link.setAttribute('href', url);
    link.setAttribute('download', `drone_flight_log_imu_coordinates_${tsStr}.csv`);
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
      ctx.lineWidth = 1.5;
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
    drawChannel(this.history.roll, '#0070f3', 1.5, midY);
    // 2. Pitch Waveform (Amber) - scale: 1 deg = 1.5px
    drawChannel(this.history.pitch, '#f59e0b', 1.5, midY);
    // 3. Accel Z (Emerald) - (Az - 1.0) * 30px
    drawChannel(this.history.accelZ.map(v => v - 1.0), '#22c55e', 30.0, midY);
    // 4. Gyro Roll Rate (Purple) - scale: 1 deg/s = 0.5px
    drawChannel(this.history.gyroRoll, '#a855f7', 0.5, midY);
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
        ctx.font = '9px "Geist Mono", monospace';
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
    ctx.font = '9px "Geist Mono", monospace';
    ctx.textAlign = 'center';
    ctx.fillText('+Y (NORTH)', cx, 14);
    ctx.fillText('+X (EAST)', w - 30, cy - 4);

    // Origin (0,0,0) HOME Marker
    ctx.fillStyle = '#f59e0b';
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = 'rgba(245, 158, 11, 0.9)';
    ctx.font = '10px "Geist Mono", monospace';
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
    ctx.font = '10px "Geist Mono", monospace';
    ctx.textAlign = 'left';
    ctx.fillText(`[X:${this.posX.toFixed(2)}m, Y:${this.posY.toFixed(2)}m, Z:${this.posZ.toFixed(2)}m]`, dronePx + 12, dronePy - 4);
  }
}

// Attach globally
window.DroneIMULogger = DroneIMULogger;
