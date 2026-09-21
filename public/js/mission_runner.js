/**
 * RC UFO Drone — Autonomous Mission Runner
 * Tuned for 100% Speed with Extended Stabilization Delays & Pure Camera Feed
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
    this.obstacleAvoidanceEnabled = false; // Pure video feed mode, no CV processing overhead
    this.obstacleDetected = false;
    this.obstacleZone = 'CLEAR';
    this.obstacleScore = 0;
        // Flight Calibration: 100% Speed with Zero Stabilization Delays
    this.calibration = {
      speedCmPerSec: 65.0,     // Linear travel speed in Gear 3 (100% Speed)
      stickDeflection: 95,     // 100% deflection magnitude for crisp rotation
      yawDegPerSec: 130.0,     // Yaw rotation rate (180° = 1.4s)
      takeoffDurationSec: 3.0, // 3.0s climb wait for firmware altitude hold to lock
      landDurationSec: 2.5,    // Touchdown duration
      flipDurationSec: 2.2,    // Stunt flip time & hover stabilization
      minStepDurationSec: 0.4,
      settleDelaySec: 0.0      // 0.0s = Zero stabilization delay between steps
    };

    this.presets = {
      takeoff_50cm_land: `# Mission: Takeoff to 50cm -> Turn 180° -> Land
TAKEOFF
YAW 180
LAND`,
      takeoff_1m_flip_land: `# Mission: Takeoff to 1m -> 360° Stunt Flip -> Land
TAKEOFF
UP 50
FLIP FORWARD
LAND`
    };

    this.initDOM();
  }

  initDOM() {
    this.modal = document.getElementById('mission-modal');
    this.scriptEditor = document.getElementById('mission-script-editor');
    this.presetSelect = document.getElementById('mission-preset-select');
    this.btnRun = document.getElementById('btn-run-mission');
    this.btnAbort = document.getElementById('btn-abort-mission');
    this.btnSave = document.getElementById('btn-save-mission');
    this.statusPill = document.getElementById('mission-status-pill');
    this.progressStepsContainer = document.getElementById('mission-steps-progress');
    this.logContainer = document.getElementById('mission-log');
    this.obstacleIndicator = document.getElementById('mission-obstacle-radar');

    if (this.presetSelect) {
      this.presetSelect.addEventListener('change', (e) => {
        const val = e.target.value;
        if (this.presets[val]) {
          this.scriptEditor.value = this.presets[val];
          this.parseScript();
        }
      });
    }

    if (this.btnRun) {
      this.btnRun.addEventListener('click', () => {
        if (this.isRunning) {
          this.abortMission("User paused/stopped mission");
        } else {
          this.startMission();
        }
      });
    }

    if (this.btnAbort) {
      this.btnAbort.addEventListener('click', () => {
        this.abortMission("Emergency Abort Triggered");
      });
    }

    if (this.btnSave) {
      this.btnSave.addEventListener('click', () => {
        localStorage.setItem('drone_custom_script', this.scriptEditor.value);
        this.log("Saved custom script to local storage.", "info");
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
        this.log(`Added command '${cmdToAdd}' to flight script.`, "info");
      });
    });

    // Always default to the Takeoff to 50cm & Land preset
    if (this.scriptEditor && this.presets.takeoff_50cm_land) {
      this.scriptEditor.value = this.presets.takeoff_50cm_land;
    }

    if (this.scriptEditor) {
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
          const fwdCm = parseFloat(arg1) || 30;
          duration = Math.max(this.calibration.minStepDurationSec, fwdCm / this.calibration.speedCmPerSec);
          description = `Fly Forward ${fwdCm} cm (${duration.toFixed(1)}s @ 100% Speed)`;
          break;
        case 'BACKWARD':
          const bwdCm = parseFloat(arg1) || 30;
          duration = Math.max(this.calibration.minStepDurationSec, bwdCm / this.calibration.speedCmPerSec);
          description = `Fly Backward ${bwdCm} cm (${duration.toFixed(1)}s @ 100% Speed)`;
          break;
        case 'LEFT':
          const leftCm = parseFloat(arg1) || 30;
          duration = Math.max(this.calibration.minStepDurationSec, leftCm / this.calibration.speedCmPerSec);
          description = `Move Left ${leftCm} cm (${duration.toFixed(1)}s @ 100% Speed)`;
          break;
        case 'RIGHT':
          const rightCm = parseFloat(arg1) || 30;
          duration = Math.max(this.calibration.minStepDurationSec, rightCm / this.calibration.speedCmPerSec);
          description = `Move Right ${rightCm} cm (${duration.toFixed(1)}s @ 100% Speed)`;
          break;
        case 'UP':
          const upCm = parseFloat(arg1) || 30;
          duration = Math.max(this.calibration.minStepDurationSec, upCm / (this.calibration.speedCmPerSec * 0.8));
          description = `Ascend ${upCm} cm (${duration.toFixed(1)}s)`;
          break;
        case 'DOWN':
          const downCm = parseFloat(arg1) || 30;
          duration = Math.max(this.calibration.minStepDurationSec, downCm / (this.calibration.speedCmPerSec * 0.8));
          description = `Descend ${downCm} cm (${duration.toFixed(1)}s)`;
          break;
        case 'YAW':
        case 'ROTATE':
          const deg = parseFloat(arg1) || 180;
          duration = Math.max(0.5, Math.abs(deg) / this.calibration.yawDegPerSec);
          description = `Rotate Yaw Right ${deg}° (${duration.toFixed(1)}s @ 100% Speed)`;
          break;
        case 'FLIP':
          duration = this.calibration.flipDurationSec;
          description = `Stunt Flip (${(arg1 || 'FORWARD').toUpperCase()})`;
          break;
        case 'AVOID_OBSTACLES':
          duration = 0.1;
          this.obstacleAvoidanceEnabled = false;
          description = `Obstacle processing: BYPASS (Direct camera feed active)`;
          break;
        default:
          isValid = false;
          description = `[ERROR: Unknown command '${cmd}']`;
      }

      steps.push({
        lineIndex: index + 1,
        raw: trimmed,
        cmd,
        arg1,
        arg2,
        duration,
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
      this.progressStepsContainer.innerHTML = '<div class="mission-empty">No valid commands in script.</div>';
      return;
    }

    this.parsedSteps.forEach((step, idx) => {
      const el = document.createElement('div');
      el.className = `mission-step-item ${idx === this.currentStepIndex && this.isRunning ? 'active' : ''} ${!step.isValid ? 'invalid' : ''}`;
      el.innerHTML = `
        <div class="step-num">#${idx + 1}</div>
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

    // Force 100% Speed Rate (Gear 3)
    if (this.app) {
      this.app.droneState.gear = 3;
      if (this.app.updateFlightChannels) this.app.updateFlightChannels();
      const gearBadge = document.getElementById('gearBadge');
      if (gearBadge) gearBadge.textContent = '100%';
    }

    fetch('/api/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ gear: 3 })
    }).catch(() => {});

    this.isRunning = true;
    this.currentStepIndex = 0;
    this.updateUIState();
    this.log("=== STARTING AUTONOMOUS MISSION (100% SPEED + STABILIZATION DELAYS) ===", "success");

    this.executeCurrentStep();
  }

  executeCurrentStep() {
    if (!this.isRunning) return;
    if (this.currentStepIndex >= this.parsedSteps.length) {
      this.completeMission();
      return;
    }

    const step = this.parsedSteps[this.currentStepIndex];
    this.renderStepsProgress();
    this.log(`Executing Step #${this.currentStepIndex + 1}: ${step.description}`, "info");

    this.applyStepCommand(step);

    this.stepStartTime = Date.now();
    const durationMs = step.duration * 1000;

    this.stepTimer = setTimeout(() => {
      // Settle into neutral hover for 0.0s before moving to next step
      this.settleNeutral(() => {
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
    if (this.app) {
      this.app.droneState.roll = 128;
      this.app.droneState.pitch = 128;
      this.app.droneState.throttle = 128;
      this.app.droneState.yaw = 128;
      if (this.app.updateFlightChannels) this.app.updateFlightChannels();
    }
    setTimeout(callback, this.calibration.settleDelaySec * 1000);
  }

  /**
   * Smooth Ramping Function: Eases stick deflection in and out for silky smooth flight
   */
  smoothRampAxis(axisName, targetVal, totalDurationSec) {
    if (!this.app) return;
    if (this.rampInterval) clearInterval(this.rampInterval);

    const startVal = 128;
    const rampTimeMs = 150;
    const startTime = Date.now();
    const totalMs = totalDurationSec * 1000;

    this.rampInterval = setInterval(() => {
      if (!this.isRunning) {
        clearInterval(this.rampInterval);
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

      this.app.droneState[axisName] = currentVal;
      if (this.app.updateFlightChannels) this.app.updateFlightChannels();

      fetch('/api/control', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          roll: this.app.droneState.roll,
          pitch: this.app.droneState.pitch,
          throttle: this.app.droneState.throttle,
          yaw: this.app.droneState.yaw
        })
      }).catch(() => {});

      if (elapsed >= totalMs) {
        clearInterval(this.rampInterval);
        this.rampInterval = null;
        this.app.droneState[axisName] = 128;
      }
    }, 25);
  }

  applyStepCommand(step) {
    const app = this.app;
    if (!app) return;

    // Reset sticks to neutral hover
    app.droneState.roll = 128;
    app.droneState.pitch = 128;
    const defl = this.calibration.stickDeflection;

    switch (step.cmd) {
      case 'TAKEOFF':
        app.triggerTakeoff();
        break;
      case 'LAND':
        app.triggerLand();
        break;
      case 'HOVER':
      case 'STAY':
      case 'WAIT':
        app.droneState.roll = 128;
        app.droneState.pitch = 128;
        app.droneState.throttle = 128;
        app.droneState.yaw = 128;
        break;
      case 'FORWARD':
        this.smoothRampAxis('pitch', Math.min(255, 128 + defl), step.duration);
        break;
      case 'BACKWARD':
        this.smoothRampAxis('pitch', Math.max(0, 128 - defl), step.duration);
        break;
      case 'LEFT':
        this.smoothRampAxis('roll', Math.max(0, 128 - defl), step.duration);
        break;
      case 'RIGHT':
        this.smoothRampAxis('roll', Math.min(255, 128 + defl), step.duration);
        break;
      case 'UP':
        this.smoothRampAxis('throttle', Math.min(255, 128 + defl), step.duration);
        break;
      case 'DOWN':
        this.smoothRampAxis('throttle', Math.max(0, 128 - defl), step.duration);
        break;
      case 'YAW':
      case 'ROTATE':
        const deg = parseFloat(step.arg1) || 90;
        const targetYaw = deg >= 0 ? Math.min(255, 128 + defl) : Math.max(0, 128 - defl);
        this.smoothRampAxis('yaw', targetYaw, step.duration);
        break;
      case 'FLIP':
        const dir = (step.arg1 || 'FORWARD').toUpperCase();
        if (app.triggerFlip) {
          app.triggerFlip(dir);
        }
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
    
    if (this.app) {
      this.app.droneState.roll = 128;
      this.app.droneState.pitch = 128;
      this.app.droneState.throttle = 128;
      this.app.droneState.yaw = 128;
      if (this.app.updateFlightChannels) this.app.updateFlightChannels();
    }

    this.updateUIState();
    this.renderStepsProgress();
    this.log(`⚠️ MISSION ABORTED: ${reason}`, "error");
  }

  completeMission() {
    this.isRunning = false;
    if (this.rampInterval) {
      clearInterval(this.rampInterval);
      this.rampInterval = null;
    }
    if (this.app) {
      this.app.droneState.roll = 128;
      this.app.droneState.pitch = 128;
      this.app.droneState.throttle = 128;
      this.app.droneState.yaw = 128;
      if (this.app.updateFlightChannels) this.app.updateFlightChannels();
    }
    this.updateUIState();
    this.renderStepsProgress();
    this.log("✅ MISSION COMPLETED SUCCESSFULLY (100% SPEED + STABILIZED)", "success");
  }

  updateUIState() {
    if (this.btnRun) {
      this.btnRun.textContent = this.isRunning ? 'PAUSE / STOP' : 'RUN MISSION SCRIPT';
      this.btnRun.className = this.isRunning ? 'btn-mission-running' : 'btn-mission-run';
    }
    if (this.statusPill) {
      this.statusPill.textContent = this.isRunning ? `STEP #${this.currentStepIndex + 1} (100% SPEED)` : 'READY';
      this.statusPill.className = `status-pill ${this.isRunning ? 'status-active' : 'status-ready'}`;
    }
  }
}

window.MissionRunner = MissionRunner;
