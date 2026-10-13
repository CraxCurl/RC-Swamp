/**
 * RC UFO Drone — Autonomous Mission Runner & Custom Programmed Instruction Engine
 * Supports Visual Step Sequence Tracking, Preset Scripts, Quick Chips, Smooth Ramp Motion, and Radar
 */

class MissionRunner {
  constructor(app) {
    this.app = app;
    this.isRunning = false;
    this.isPaused = false;
    this.currentStepIndex = 0;
    this.parsedSteps = [];
    this.stepTimer = null;
    this.rampInterval = null;
    this.stepStartTime = 0;
    this.activeStepLabel = '';

    // Flight Calibration
    this.calibration = {
      speedCmPerSec: 65.0,     // Linear travel speed in Gear 3 (100% Speed)
      stickDeflection: 95,     // 100% deflection magnitude for crisp rotation
      yawDegPerSec: 130.0,     // Yaw rotation rate (180° = 1.4s)
      takeoffDurationSec: 3.0, // 3.0s climb wait for firmware altitude hold to lock
      landDurationSec: 2.5,    // Touchdown duration
      flipDurationSec: 2.2,    // Stunt flip time & hover stabilization
      minStepDurationSec: 0.4,
      settleDelaySec: 0.2      // Brief stabilization between moves
    };

    this.presets = {
      takeoff_50cm_land: `# Preset 1: Auto Takeoff -> Hover -> Land
TAKEOFF
HOVER 2.5
LAND`,
      takeoff_1m_flip_land: `# Preset 2: Takeoff to 1m -> 360° Stunt Flip -> Land
TAKEOFF
UP 50
HOVER 1.5
FLIP FORWARD
HOVER 2.0
LAND`,
      square_patrol: `# Preset 3: 1m x 1m Square Box Patrol Pattern
TAKEOFF
FORWARD 100
RIGHT 100
BACKWARD 100
LEFT 100
HOVER 2.0
LAND`,
      panoramic_scan: `# Preset 4: Ascend 1.5m -> 360° Panoramic Scan -> Descend -> Land
TAKEOFF
UP 75
HOVER 1.5
YAW 180
HOVER 1.0
YAW 180
HOVER 1.5
DOWN 50
LAND`
    };

    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', () => this.initDOM());
    } else {
      setTimeout(() => this.initDOM(), 50);
    }
  }

  initDOM() {
    this.modal = document.getElementById('mission-modal');
    this.scriptEditor = document.getElementById('mission-script-editor') || document.getElementById('mission-script-input');
    this.presetSelect = document.getElementById('mission-preset-select');
    this.btnRun = document.getElementById('btn-run-mission');
    this.btnPause = document.getElementById('btn-pause-mission');
    this.btnAbort = document.getElementById('btn-abort-mission');
    this.btnSave = document.getElementById('btn-save-mission');
    this.btnExportMissionCsv = document.getElementById('btn-export-mission-csv');
    this.btnDownloadScript = document.getElementById('btn-download-script');
    this.btnUploadScript = document.getElementById('btn-upload-script');
    this.scriptFileInput = document.getElementById('mission-file-input');
    this.statusPill = document.getElementById('mission-status-pill');
    this.progressStepsContainer = document.getElementById('mission-steps-progress');
    this.logContainer = document.getElementById('mission-log');

    if (this.btnExportMissionCsv) {
      this.btnExportMissionCsv.addEventListener('click', () => {
        if (window.droneImu) {
          window.droneImu.downloadCSV('current');
          this.log("Exported active flight mission telemetry log (.csv)", "success");
        } else {
          this.log("IMU telemetry logger offline", "error");
        }
      });
    }

    // Preset dropdown listener
    if (this.presetSelect) {
      this.presetSelect.addEventListener('change', (e) => {
        const val = e.target.value;
        if (this.presets[val]) {
          if (this.scriptEditor) {
            this.scriptEditor.value = this.presets[val];
            this.parseScript();
            this.log(`Loaded preset mission: ${val}`, "info");
          }
        }
      });
    }

    if (this.btnRun) {
      this.btnRun.addEventListener('click', () => {
        if (this.isRunning) {
          this.abortMission("Mission stopped by user");
        } else {
          this.startMission();
        }
      });
    }

    if (this.btnPause) {
      this.btnPause.addEventListener('click', () => {
        if (!this.isRunning) return;
        this.isPaused = !this.isPaused;
        this.btnPause.textContent = this.isPaused ? 'Resume Mission' : 'Pause Mission';
        this.log(this.isPaused ? "Mission execution paused." : "Mission execution resumed.", "warn");
      });
    }

    if (this.btnAbort) {
      this.btnAbort.addEventListener('click', () => {
        this.abortMission("Emergency Abort Triggered");
      });
    }

    if (this.btnSave) {
      this.btnSave.addEventListener('click', () => {
        if (this.scriptEditor) {
          localStorage.setItem('drone_custom_script', this.scriptEditor.value);
          this.log("Saved custom mission script to browser storage.", "success");
        }
      });
    }

    if (this.btnDownloadScript) {
      this.btnDownloadScript.addEventListener('click', () => {
        if (!this.scriptEditor) return;
        const text = this.scriptEditor.value;
        const blob = new Blob([text], { type: 'text/plain;charset=utf-8' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `drone_mission_script_${Date.now()}.txt`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
      });
    }

    if (this.btnUploadScript && this.scriptFileInput) {
      this.btnUploadScript.addEventListener('click', () => {
        this.scriptFileInput.click();
      });

      this.scriptFileInput.addEventListener('change', (e) => {
        const file = e.target.files[0];
        if (!file) return;
        const reader = new FileReader();
        reader.onload = (evt) => {
          if (this.scriptEditor) {
            this.scriptEditor.value = evt.target.result;
            this.parseScript();
            this.log(`Loaded script from file '${file.name}'`, "success");
          }
        };
        reader.readAsText(file);
      });
    }

    // Quick Command Chip Inserters
    document.querySelectorAll('.cmd-chip').forEach((chip) => {
      chip.addEventListener('click', () => {
        const cmdToAdd = chip.getAttribute('data-cmd');
        if (!cmdToAdd || !this.scriptEditor) return;
        const currentVal = this.scriptEditor.value.trim();
        this.scriptEditor.value = currentVal ? `${currentVal}\n${cmdToAdd}` : cmdToAdd;
        this.parseScript();
        this.log(`Inserted instruction: '${cmdToAdd}'`, "info");
      });
    });

    // Default script load
    const saved = localStorage.getItem('drone_custom_script');
    if (this.scriptEditor) {
      this.scriptEditor.value = saved || this.presets.takeoff_50cm_land;
      this.scriptEditor.addEventListener('input', () => this.parseScript());
      this.parseScript();
    }
  }

  log(msg, type = "info") {
    if (!this.logContainer) return;
    const item = document.createElement('div');
    item.className = `mission-log-line log-${type}`;
    const timestamp = new Date().toLocaleTimeString();
    item.textContent = `[${timestamp}] ${msg}`;
    this.logContainer.appendChild(item);
    this.logContainer.scrollTop = this.logContainer.scrollHeight;
  }

  parseScript() {
    if (!this.scriptEditor) return [];
    const text = this.scriptEditor.value;
    const lines = text.split('\n');
    const steps = [];

    lines.forEach((line, index) => {
      let trimmed = line.trim();
      if (trimmed.startsWith('#') || trimmed.startsWith('//')) return;
      if (trimmed.length === 0) return;

      const parts = trimmed.split(/\s+/);
      const cmd = parts[0].toUpperCase();
      const arg1 = parts[1];
      const arg2 = parts[2];

      let duration = 0;
      let description = '';
      let isValid = true;

      switch (cmd) {
        case 'TAKEOFF':
          duration = this.calibration.takeoffDurationSec;
          description = `Auto-Takeoff & ascend to hover lock (${duration}s stabilization)`;
          break;
        case 'LAND':
          duration = this.calibration.landDurationSec;
          description = `Auto-Land smoothly to ground (${duration}s)`;
          break;
        case 'HOVER':
        case 'STAY':
        case 'WAIT':
          duration = parseFloat(arg1) || 2.5;
          description = `Stabilize position & hold hover for ${duration}s`;
          break;
        case 'FORWARD':
          const fwdCm = parseFloat(arg1) || 50;
          duration = Math.max(1.0, (fwdCm / this.calibration.speedCmPerSec) + 0.3);
          description = `Fly Forward ${fwdCm} cm (${duration.toFixed(1)}s)`;
          break;
        case 'BACKWARD':
          const bwdCm = parseFloat(arg1) || 50;
          duration = Math.max(1.0, (bwdCm / this.calibration.speedCmPerSec) + 0.3);
          description = `Fly Backward ${bwdCm} cm (${duration.toFixed(1)}s)`;
          break;
        case 'LEFT':
          const leftCm = parseFloat(arg1) || 50;
          duration = Math.max(1.0, (leftCm / this.calibration.speedCmPerSec) + 0.3);
          description = `Move Left ${leftCm} cm (${duration.toFixed(1)}s)`;
          break;
        case 'RIGHT':
          const rightCm = parseFloat(arg1) || 50;
          duration = Math.max(1.0, (rightCm / this.calibration.speedCmPerSec) + 0.3);
          description = `Move Right ${rightCm} cm (${duration.toFixed(1)}s)`;
          break;
        case 'UP':
          const upCm = parseFloat(arg1) || 50;
          duration = Math.max(1.0, (upCm / (this.calibration.speedCmPerSec * 0.8)) + 0.3);
          description = `Ascend Altitude +${upCm} cm (${duration.toFixed(1)}s)`;
          break;
        case 'DOWN':
          const downCm = parseFloat(arg1) || 50;
          duration = Math.max(1.0, (downCm / (this.calibration.speedCmPerSec * 0.8)) + 0.3);
          description = `Descend Altitude -${downCm} cm (${duration.toFixed(1)}s)`;
          break;
        case 'YAW':
        case 'ROTATE':
          const deg = parseFloat(arg1) || 180;
          duration = Math.max(0.5, Math.abs(deg) / this.calibration.yawDegPerSec);
          description = `Rotate Yaw Heading ${deg >= 0 ? '+' : ''}${deg}° (${duration.toFixed(1)}s)`;
          break;
        case 'FLIP':
          duration = this.calibration.flipDurationSec;
          description = `Perform 360° Stunt Flip (${(arg1 || 'FORWARD').toUpperCase()})`;
          break;
        case 'CALIBRATE':
          duration = 1.5;
          description = `Gyroscope Sensor Recalibration (1.5s)`;
          break;
        default:
          isValid = false;
          description = `[ERROR: Unknown instruction '${cmd}']`;
      }

      steps.push({
        lineIndex: index + 1,
        raw: trimmed,
        cmd,
        arg1,
        arg2,
        duration: parseFloat(duration.toFixed(2)),
        description,
        isValid
      });
    });

    this.parsedSteps = steps;
    this.renderStepsProgress();
    return steps;
  }

  renderStepsProgress() {
    if (!this.progressStepsContainer) return;
    this.progressStepsContainer.innerHTML = '';

    if (this.parsedSteps.length === 0) {
      this.progressStepsContainer.innerHTML = '<div class="mission-empty">No valid commands in script. Insert commands or select a preset.</div>';
      return;
    }

    this.parsedSteps.forEach((step, idx) => {
      const el = document.createElement('div');
      const isCurrent = idx === this.currentStepIndex && this.isRunning;
      const isDone = idx < this.currentStepIndex && this.isRunning;
      el.className = `mission-step-item ${isCurrent ? 'active' : ''} ${isDone ? 'completed' : ''} ${!step.isValid ? 'invalid' : ''}`;
      el.innerHTML = `
        <div class="step-num">${isDone ? '✓' : '#' + (idx + 1)}</div>
        <div class="step-info">
          <div class="step-cmd">${step.raw}</div>
          <div class="step-desc">${step.description}</div>
        </div>
        <div class="step-duration">${step.duration > 0 ? step.duration.toFixed(1) + 's' : ''}</div>
      `;
      this.progressStepsContainer.appendChild(el);
    });
  }

  startMission() {
    this.parseScript();
    if (this.parsedSteps.length === 0 || this.parsedSteps.some(s => !s.isValid)) {
      this.log("Cannot start: Script contains syntax errors or is empty.", "error");
      return;
    }

    if (this.stepTimer) {
      clearTimeout(this.stepTimer);
      this.stepTimer = null;
    }
    if (this.rampInterval) {
      clearInterval(this.rampInterval);
      this.rampInterval = null;
    }

    this.isRunning = true;
    this.isPaused = false;
    this.currentStepIndex = 0;
    this.activeStepLabel = '';
    this.updateUIState();

    // Determine mission title from script comments or editor
    let missionTitle = 'Custom Flight Mission';
    if (this.scriptEditor) {
      const lines = this.scriptEditor.value.split('\n');
      for (const l of lines) {
        const trimmed = l.trim();
        if (trimmed.startsWith('#') || trimmed.startsWith('//')) {
          const clean = trimmed.replace(/^[#/\s]+/, '').trim();
          if (clean) {
            missionTitle = clean;
            break;
          }
        }
      }
    }

    if (window.droneImu) {
      window.droneImu.startNewSortie(`Simulation: ${missionTitle}`);
      window.droneImu.isMissionSortie = true;
    }

    this.log(`=== STARTING AUTONOMOUS MISSION: ${missionTitle} ===`, "success");
    this.executeCurrentStep();
  }

  executeCurrentStep() {
    if (!this.isRunning) return;

    if (this.isPaused) {
      setTimeout(() => this.executeCurrentStep(), 200);
      return;
    }

    if (this.currentStepIndex >= this.parsedSteps.length) {
      this.completeMission();
      return;
    }

    const step = this.parsedSteps[this.currentStepIndex];
    const stepNum = this.currentStepIndex + 1;
    const totalSteps = this.parsedSteps.length;
    let stepTag = `STEP_${stepNum}_${step.cmd}`;
    if (step.arg1) stepTag += `_${step.arg1}`;
    this.activeStepLabel = stepTag;

    this.renderStepsProgress();
    this.updateUIState();
    this.log(`Step #${stepNum}/${totalSteps}: ${step.description}`, "info");

    this.applyStepCommand(step);

    this.stepStartTime = Date.now();
    const durationMs = step.duration * 1000;

    if (this.stepTimer) {
      clearTimeout(this.stepTimer);
      this.stepTimer = null;
    }

    this.stepTimer = setTimeout(() => {
      this.settleNeutral(() => {
        if (!this.isRunning) return;
        this.currentStepIndex++;
        this.executeCurrentStep();
      });
    }, durationMs);
  }

  settleNeutral(callback) {
    if (this.rampInterval) {
      clearInterval(this.rampInterval);
      this.rampInterval = null;
    }
    if (window.droneDispatchFlightCommand) {
      window.droneDispatchFlightCommand({ action: 'stick', roll: 128, pitch: 128, throttle: 128, yaw: 128 });
    }
    if (window.droneImu) {
      window.droneImu.velX = 0;
      window.droneImu.velY = 0;
      window.droneImu.groundSpeed = 0;
    }
    setTimeout(callback, this.calibration.settleDelaySec * 1000);
  }

  /**
   * Smooth Kinematic Ramping: Controls stick deflection and integrates accurate 3D trajectory
   */
  smoothRampAxis(axisName, targetVal, totalDurationSec, targetAxis = null, targetDeltaM = 0) {
    if (this.rampInterval) {
      clearInterval(this.rampInterval);
      this.rampInterval = null;
    }

    const startVal = 128;
    const rampTimeMs = 150;
    const startTime = Date.now();
    const totalMs = totalDurationSec * 1000;

    const initialPos = (targetAxis && window.droneImu) ? (window.droneImu[targetAxis] || 0) : 0;
    let lastDisplacement = 0;

    this.rampInterval = setInterval(() => {
      if (!this.isRunning || this.isPaused) {
        clearInterval(this.rampInterval);
        this.rampInterval = null;
        return;
      }

      const elapsed = Date.now() - startTime;
      let currentVal = targetVal;

      if (elapsed < rampTimeMs) {
        // Ramp In
        const factor = elapsed / rampTimeMs;
        currentVal = Math.round(startVal + (targetVal - startVal) * factor);
      } else if (elapsed > totalMs - rampTimeMs) {
        // Ramp Out
        const remaining = Math.max(0, totalMs - elapsed);
        const factor = remaining / rampTimeMs;
        currentVal = Math.round(startVal + (targetVal - startVal) * factor);
      }

      // Smooth kinematic trajectory propagation
      if (targetAxis && window.droneImu && targetDeltaM !== 0) {
        const progress = Math.min(1.0, elapsed / totalMs);
        const smoothProgress = progress * progress * (3 - 2 * progress); // smoothstep
        const newPos = initialPos + (targetDeltaM * smoothProgress);
        const currentDelta = newPos - initialPos;
        const incrementalDist = Math.abs(currentDelta - lastDisplacement);
        lastDisplacement = currentDelta;

        if (targetAxis === 'yaw') {
          window.droneImu.curYawDeg = (initialPos + targetDeltaM * smoothProgress + 360) % 360;
        } else {
          window.droneImu[targetAxis] = parseFloat(newPos.toFixed(3));
          const stepSec = Math.max(0.1, totalMs / 1000);
          const instSpeed = parseFloat((((targetDeltaM * 6 * progress * (1 - progress)) / stepSec)).toFixed(2));
          if (targetAxis === 'posZ') {
            window.droneImu.velZ = instSpeed;
          } else if (targetAxis === 'posY') {
            window.droneImu.velY = instSpeed;
          } else if (targetAxis === 'posX') {
            window.droneImu.velX = instSpeed;
          }
          if (incrementalDist > 0) {
            window.droneImu.totalDistanceTraveled += incrementalDist;
          }
        }
      }

      const stickPayload = {
        action: 'stick',
        roll: axisName === 'roll' ? currentVal : 128,
        pitch: axisName === 'pitch' ? currentVal : 128,
        throttle: axisName === 'throttle' ? currentVal : 128,
        yaw: axisName === 'yaw' ? currentVal : 128
      };

      if (window.droneDispatchFlightCommand) {
        window.droneDispatchFlightCommand(stickPayload);
      }

      if (elapsed >= totalMs) {
        clearInterval(this.rampInterval);
        this.rampInterval = null;
        if (targetAxis && window.droneImu && targetDeltaM !== 0) {
          if (targetAxis === 'yaw') {
            window.droneImu.curYawDeg = (initialPos + targetDeltaM + 360) % 360;
          } else {
            window.droneImu[targetAxis] = parseFloat((initialPos + targetDeltaM).toFixed(3));
          }
        }
        if (window.droneDispatchFlightCommand) {
          window.droneDispatchFlightCommand({ action: 'stick', roll: 128, pitch: 128, throttle: 128, yaw: 128 });
        }
      }
    }, 25);
  }

  applyStepCommand(step) {
    const defl = this.calibration.stickDeflection;
    const stepNum = this.currentStepIndex + 1;
    let stepTag = `STEP_${stepNum}_${step.cmd}`;
    if (step.arg1) stepTag += `_${step.arg1}`;
    this.activeStepLabel = stepTag;

    // Log explicit simulation step to IMU Flight Logger immediately
    if (window.droneImu) {
      window.droneImu.logForcedEvent(stepTag, step.description);
    }

    switch (step.cmd) {
      case 'TAKEOFF':
        this.smoothRampAxis('throttle', 185, this.calibration.takeoffDurationSec, 'posZ', 0.75);
        if (window.droneDispatchFlightCommand) window.droneDispatchFlightCommand({ action: 'takeoff' });
        break;
      case 'LAND':
        const curZ = window.droneImu ? window.droneImu.posZ : 0.75;
        this.smoothRampAxis('throttle', 70, this.calibration.landDurationSec, 'posZ', -curZ);
        if (window.droneDispatchFlightCommand) window.droneDispatchFlightCommand({ action: 'land' });
        break;
      case 'HOVER':
      case 'STAY':
      case 'WAIT':
        if (window.droneDispatchFlightCommand) {
          window.droneDispatchFlightCommand({ action: 'stick', roll: 128, pitch: 128, throttle: 128, yaw: 128 });
        }
        break;
      case 'FORWARD':
        const fwdCm = parseFloat(step.arg1) || 50;
        this.smoothRampAxis('pitch', Math.min(255, 128 + defl), step.duration, 'posY', fwdCm / 100.0);
        break;
      case 'BACKWARD':
        const bwdCm = parseFloat(step.arg1) || 50;
        this.smoothRampAxis('pitch', Math.max(0, 128 - defl), step.duration, 'posY', -(bwdCm / 100.0));
        break;
      case 'LEFT':
        const leftCm = parseFloat(step.arg1) || 50;
        this.smoothRampAxis('roll', Math.max(0, 128 - defl), step.duration, 'posX', -(leftCm / 100.0));
        break;
      case 'RIGHT':
        const rightCm = parseFloat(step.arg1) || 50;
        this.smoothRampAxis('roll', Math.min(255, 128 + defl), step.duration, 'posX', rightCm / 100.0);
        break;
      case 'UP':
        const upCm = parseFloat(step.arg1) || 50;
        this.smoothRampAxis('throttle', Math.min(255, 128 + defl), step.duration, 'posZ', upCm / 100.0);
        break;
      case 'DOWN':
        const downCm = parseFloat(step.arg1) || 50;
        this.smoothRampAxis('throttle', Math.max(0, 128 - defl), step.duration, 'posZ', -(downCm / 100.0));
        break;
      case 'YAW':
      case 'ROTATE':
        const deg = parseFloat(step.arg1) || 90;
        const targetYaw = deg >= 0 ? Math.min(255, 128 + defl) : Math.max(0, 128 - defl);
        this.smoothRampAxis('yaw', targetYaw, step.duration, 'yaw', deg);
        break;
      case 'FLIP':
        if (window.droneDispatchFlightCommand) window.droneDispatchFlightCommand({ action: 'flip_360' });
        break;
      case 'CALIBRATE':
        if (window.droneDispatchFlightCommand) window.droneDispatchFlightCommand({ action: 'calibrate_gyro' });
        break;
    }
  }

  abortMission(reason = "Aborted") {
    if (this.stepTimer) {
      clearTimeout(this.stepTimer);
      this.stepTimer = null;
    }
    if (this.rampInterval) {
      clearInterval(this.rampInterval);
      this.rampInterval = null;
    }

    this.isRunning = false;
    this.isPaused = false;
    this.activeStepLabel = '';

    if (window.droneImu) {
      window.droneImu.isMissionSortie = false;
      window.droneImu.logForcedEvent('MISSION_ABORTED', reason);
    }

    if (window.droneDispatchFlightCommand) {
      window.droneDispatchFlightCommand({ action: 'stick', roll: 128, pitch: 128, throttle: 128, yaw: 128 });
      window.droneDispatchFlightCommand({ action: 'land' });
    }

    this.updateUIState();
    this.renderStepsProgress();
    this.log(`MISSION ABORTED: ${reason} (Landing initiated)`, "error");
  }

  completeMission() {
    this.isRunning = false;
    this.isPaused = false;
    this.activeStepLabel = '';
    if (this.rampInterval) {
      clearInterval(this.rampInterval);
      this.rampInterval = null;
    }
    if (this.stepTimer) {
      clearTimeout(this.stepTimer);
      this.stepTimer = null;
    }

    if (window.droneDispatchFlightCommand) {
      window.droneDispatchFlightCommand({ action: 'stick', roll: 128, pitch: 128, throttle: 128, yaw: 128 });
    }

    if (window.droneImu) {
      window.droneImu.velX = 0;
      window.droneImu.velY = 0;
      window.droneImu.velZ = 0;
      window.droneImu.groundSpeed = 0;
      window.droneImu.isMissionSortie = false;
      window.droneImu.logForcedEvent('MISSION_COMPLETED', 'Mission Completed Successfully');
    }

    this.updateUIState();
    this.renderStepsProgress();
    this.log("MISSION COMPLETED SUCCESSFULLY", "success");
    this.log("Flight log recorded! Click 'Download Flight Log (CSV)' to export.", "info");
  }

  updateUIState() {
    if (this.btnRun) {
      this.btnRun.textContent = this.isRunning ? 'STOP MISSION' : 'RUN MISSION SCRIPT';
      this.btnRun.className = this.isRunning ? 'v-btn v-btn-danger' : 'v-btn v-btn-primary';
    }
    if (this.statusPill) {
      this.statusPill.textContent = this.isRunning ? `STEP #${this.currentStepIndex + 1}/${this.parsedSteps.length}` : 'READY';
      this.statusPill.className = `v-tag ${this.isRunning ? 'v-tag-rec' : 'v-tag-mono'}`;
    }
  }
}

window.MissionRunner = MissionRunner;
