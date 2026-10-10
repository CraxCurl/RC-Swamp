/**
 * Drone HUD & Artificial Horizon Instrument Renderer — Vercel / Geist Edition
 * Renders both the Cockpit FPV Overlay HUD and the 3D Circular Attitude Indicator Sphere.
 */

class DroneHUD {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    if (this.canvas) {
      this.ctx = this.canvas.getContext('2d');
    }
    this.roll = 0;   // Degrees (-45..45)
    this.pitch = 0;  // Degrees (-30..30)
    this.yaw = 0;    // Degrees (0..360)
    this.altitude = 0;
  }

  updateAttitude(rollVal, pitchVal, yawVal, throttleVal) {
    // Calibrated physics mapping (-28.5 deg to +28.5 deg based on stick deflection)
    this.roll = (rollVal - 128) * (28.5 / 127.0);
    this.pitch = (pitchVal - 128) * (28.5 / 127.0);
    this.yaw = ((yawVal - 128) * (180.0 / 127.0) + 360) % 360;
    this.altitude = (throttleVal / 255) * 100.0;
  }

  static renderAttitudeSphere(canvas, rollDeg, pitchDeg) {
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    const cx = w / 2;
    const cy = h / 2;
    const r = (w / 2) - 3;

    ctx.clearRect(0, 0, w, h);

    // 1. Clip to circular horizon sphere
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.clip();

    // 2. Rotate & Translate according to Roll and Pitch
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate((rollDeg * Math.PI) / 180);

    const pitchPx = pitchDeg * 2.2;

    // Vercel Sky Gradient (Deep Indigo-Black to Slate Blue)
    const skyGrad = ctx.createLinearGradient(0, pitchPx - r * 1.5, 0, pitchPx);
    skyGrad.addColorStop(0, '#030712');
    skyGrad.addColorStop(0.7, '#0f172a');
    skyGrad.addColorStop(1, '#1e293b');
    ctx.fillStyle = skyGrad;
    ctx.fillRect(-w, pitchPx - h * 2, w * 2, h * 2);

    // Vercel Ground Gradient (Dark Graphite to Pitch Black)
    const groundGrad = ctx.createLinearGradient(0, pitchPx, 0, pitchPx + r * 1.5);
    groundGrad.addColorStop(0, '#18181b');
    groundGrad.addColorStop(0.6, '#09090b');
    groundGrad.addColorStop(1, '#000000');
    ctx.fillStyle = groundGrad;
    ctx.fillRect(-w, pitchPx, w * 2, h * 2);

    // Horizon Line (Pure White / Electric Blue Level)
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    ctx.moveTo(-w, pitchPx);
    ctx.lineTo(w, pitchPx);
    ctx.stroke();

    // Pitch Ladder Lines (+10, -10, +20, -20)
    ctx.fillStyle = 'rgba(255, 255, 255, 0.95)';
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.65)';
    ctx.lineWidth = 1;
    ctx.font = '9px "Geist Mono", "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    const pitchBars = [
      { deg: 10, label: '+10°' },
      { deg: -10, label: '-10°' },
      { deg: 20, label: '+20°' },
      { deg: -20, label: '-20°' }
    ];

    pitchBars.forEach(({ deg, label }) => {
      const y = pitchPx - (deg * 2.2);
      ctx.beginPath();
      ctx.moveTo(-32, y);
      ctx.lineTo(-12, y);
      ctx.moveTo(12, y);
      ctx.lineTo(32, y);
      ctx.stroke();

      ctx.fillText(label, -44, y);
      ctx.fillText(label, 44, y);
    });

    ctx.restore(); // Restore roll/pitch transformation

    // 3. Fixed White/Yellow Aircraft Reticle Symbol (Center)
    ctx.strokeStyle = '#ffffff';
    ctx.fillStyle = '#0070f3';
    ctx.lineWidth = 2;

    // Center dot/circle
    ctx.beginPath();
    ctx.arc(cx, cy, 3.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Left wing
    ctx.beginPath();
    ctx.moveTo(cx - 28, cy);
    ctx.lineTo(cx - 8, cy);
    ctx.stroke();

    // Right wing
    ctx.beginPath();
    ctx.moveTo(cx + 8, cy);
    ctx.lineTo(cx + 28, cy);
    ctx.stroke();

    ctx.restore(); // Restore clip
  }

  /**
   * Renders real-time dynamic FPV cockpit camera view with horizon, 3D perspective grid,
   * compass heading ribbon, altitude/airspeed ladders, pitch indicator, and crosshair reticle.
   */
  static renderSyntheticFPV(canvas, state, imu, tick = 0) {
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    if (w <= 0 || h <= 0) return;

    const cx = w / 2;
    const cy = h / 2;

    const rollDeg = imu ? imu.rollDeg : ((state.roll - 128) * (28.5 / 127.0));
    const pitchDeg = imu ? imu.pitchDeg : ((state.pitch - 128) * (28.5 / 127.0));
    const yawDeg = imu ? imu.yawDeg : Math.round((state.yaw - 128) * (180.0 / 127.0));
    const altM = imu ? imu.posZ : ((state.throttle / 255) * 3.5);
    const speedMs = imu ? imu.groundSpeed : 0;
    const thrPct = Math.round((state.throttle / 255) * 100);

    ctx.save();
    ctx.clearRect(0, 0, w, h);

    // 1. Sky & Ground Horizon Background (Vibrant High-Contrast Tactical Palettes)
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate((-rollDeg * Math.PI) / 180);

    const pitchPx = pitchDeg * 4.0;

    // Tactical Midnight Blue FPV Sky Gradient
    const skyGrad = ctx.createLinearGradient(0, pitchPx - h * 1.5, 0, pitchPx);
    skyGrad.addColorStop(0, '#061937');
    skyGrad.addColorStop(0.5, '#0c2b52');
    skyGrad.addColorStop(0.85, '#164273');
    skyGrad.addColorStop(1, '#1d538c');
    ctx.fillStyle = skyGrad;
    ctx.fillRect(-w * 2, pitchPx - h * 3, w * 4, h * 3);

    // Tactical Topographic Dark Emerald Terrain Gradient
    const groundGrad = ctx.createLinearGradient(0, pitchPx, 0, pitchPx + h * 1.5);
    groundGrad.addColorStop(0, '#0e4a33');
    groundGrad.addColorStop(0.35, '#093826');
    groundGrad.addColorStop(0.7, '#052418');
    groundGrad.addColorStop(1, '#02120b');
    ctx.fillStyle = groundGrad;
    ctx.fillRect(-w * 2, pitchPx, w * 4, h * 3);

    // Bright Glowing Horizon Line
    ctx.strokeStyle = '#00f0ff';
    ctx.shadowColor = '#00f0ff';
    ctx.shadowBlur = 10;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(-w * 2, pitchPx);
    ctx.lineTo(w * 2, pitchPx);
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Perspective Terrain Grid Lines (Vibrant Cyan, 45% Opacity)
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.45)';
    ctx.lineWidth = 1.2;
    const gridSpeedOffset = (tick * 90 * (speedMs + 0.6)) % 40;
    for (let gy = 15; gy < h * 1.2; gy += 25) {
      const yLine = pitchPx + gy + (gridSpeedOffset * (gy / h));
      if (yLine > pitchPx) {
        ctx.beginPath();
        ctx.moveTo(-w * 2, yLine);
        ctx.lineTo(w * 2, yLine);
        ctx.stroke();
      }
    }

    // Radial Perspective Vanishing Lines
    for (let vx = -w * 1.5; vx <= w * 1.5; vx += 60) {
      ctx.beginPath();
      ctx.moveTo(0, pitchPx);
      ctx.lineTo(vx * 3.0, pitchPx + h * 2.5);
      ctx.stroke();
    }

    // Dynamic Pitch Ladder (+30, +20, +10, -10, -20, -30)
    ctx.fillStyle = '#00f0ff';
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.85)';
    ctx.lineWidth = 1.5;
    ctx.font = 'bold 10px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    const ladderSteps = [
      { deg: 30, text: '+30°' },
      { deg: 20, text: '+20°' },
      { deg: 10, text: '+10°' },
      { deg: -10, text: '-10°' },
      { deg: -20, text: '-20°' },
      { deg: -30, text: '-30°' }
    ];

    ladderSteps.forEach(({ deg, text }) => {
      const ly = pitchPx - (deg * 4.0);
      const isNegative = deg < 0;
      const barW = isNegative ? 40 : 54;

      ctx.beginPath();
      if (isNegative) {
        ctx.setLineDash([5, 5]);
      } else {
        ctx.setLineDash([]);
      }
      ctx.moveTo(-barW, ly);
      ctx.lineTo(-16, ly);
      ctx.moveTo(16, ly);
      ctx.lineTo(barW, ly);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.fillText(text, -barW - 16, ly);
      ctx.fillText(text, barW + 16, ly);
    });

    ctx.restore(); // Restore horizon rotation

    // 2. Fixed Center Aircraft Reticle & Crosshair
    ctx.save();
    ctx.strokeStyle = '#ffcc00';
    ctx.fillStyle = '#ffcc00';
    ctx.shadowColor = '#ffcc00';
    ctx.shadowBlur = 8;
    ctx.lineWidth = 2.2;

    // Center Crosshair Dot
    ctx.beginPath();
    ctx.arc(cx, cy, 3, 0, Math.PI * 2);
    ctx.fill();

    // Aircraft Target Wings
    ctx.beginPath();
    ctx.moveTo(cx - 36, cy);
    ctx.lineTo(cx - 12, cy);
    ctx.lineTo(cx - 12, cy + 6);
    ctx.moveTo(cx + 12, cy);
    ctx.lineTo(cx + 36, cy);
    ctx.lineTo(cx + 12, cy + 6);
    ctx.moveTo(cx, cy - 18);
    ctx.lineTo(cx, cy - 8);
    ctx.stroke();
    ctx.shadowBlur = 0;
    ctx.restore();

    // 3. Top Compass Ribbon (Heading 0..360°)
    ctx.save();
    ctx.fillStyle = 'rgba(5, 12, 22, 0.85)';
    ctx.fillRect(cx - 150, 6, 300, 26);
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.4)';
    ctx.strokeRect(cx - 150, 6, 300, 26);

    ctx.font = 'bold 10px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = '#00f0ff';

    const headingCardinals = { 0: 'N', 90: 'E', 180: 'S', 270: 'W' };
    for (let deg = -60; deg <= 60; deg += 15) {
      const hDeg = (Math.round(yawDeg) + deg + 360) % 360;
      const xPos = cx + (deg * 2.3);
      if (xPos >= cx - 140 && xPos <= cx + 140) {
        ctx.strokeStyle = deg % 30 === 0 ? '#00f0ff' : 'rgba(255, 255, 255, 0.5)';
        ctx.beginPath();
        ctx.moveTo(xPos, 22);
        ctx.lineTo(xPos, 30);
        ctx.stroke();

        const label = headingCardinals[hDeg] || (deg % 30 === 0 ? String(hDeg).padStart(3, '0') : '');
        if (label) {
          ctx.fillText(label, xPos, 14);
        }
      }
    }

    // Compass Pointer
    ctx.fillStyle = '#ffcc00';
    ctx.beginPath();
    ctx.moveTo(cx, 31);
    ctx.lineTo(cx - 5, 37);
    ctx.lineTo(cx + 5, 37);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    // 4. Left Airspeed Tape
    ctx.save();
    ctx.fillStyle = 'rgba(5, 12, 22, 0.85)';
    ctx.fillRect(10, cy - 70, 62, 140);
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.4)';
    ctx.strokeRect(10, cy - 70, 62, 140);

    ctx.fillStyle = '#00f0ff';
    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.textAlign = 'left';
    ctx.fillText('SPD m/s', 14, cy - 55);
    ctx.font = 'bold 16px "JetBrains Mono", monospace';
    ctx.fillText(speedMs.toFixed(1), 14, cy);
    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.fillText(`THR:${thrPct}%`, 14, cy + 52);
    ctx.restore();

    // 5. Right Altitude Tape
    ctx.save();
    ctx.fillStyle = 'rgba(5, 12, 22, 0.85)';
    ctx.fillRect(w - 72, cy - 70, 62, 140);
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.4)';
    ctx.strokeRect(w - 72, cy - 70, 62, 140);

    ctx.fillStyle = '#00f0ff';
    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.textAlign = 'left';
    ctx.fillText('ALT (Z)', w - 67, cy - 55);
    ctx.font = 'bold 16px "JetBrains Mono", monospace';
    ctx.fillText(altM.toFixed(1) + 'm', w - 67, cy);
    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.fillText(`${(altM * 3.28084).toFixed(0)} ft`, w - 67, cy + 52);
    ctx.restore();

    // 6. Analog CRT Scanlines & Camera Noise Overlay
    ctx.save();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.06)';
    for (let sy = 0; sy < h; sy += 3) {
      ctx.fillRect(0, sy, w, 1);
    }
    ctx.restore();

    // 7. Bottom FPV Stream Status Legend
    ctx.save();
    ctx.fillStyle = 'rgba(5, 12, 22, 0.9)';
    ctx.fillRect(cx - 180, h - 28, 360, 22);
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.3)';
    ctx.strokeRect(cx - 180, h - 28, 360, 22);

    ctx.font = 'bold 9.5px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = state.hasLiveVideo ? '#22c55e' : '#38bdf8';
    const statusMsg = state.hasLiveVideo 
      ? `● LIVE CAMERA FEED [${state.device_type || 'GL-21B'}] 30 FPS`
      : `📡 FPV STANDBY — AWAITING DRONE LINK (192.168.1.1:7070)`;
    ctx.fillText(statusMsg, cx, h - 17);
    ctx.restore();

    ctx.restore();
  }
}
