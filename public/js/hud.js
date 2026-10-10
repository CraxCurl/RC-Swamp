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

    // 1. Sky & Ground Horizon Background
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate((-rollDeg * Math.PI) / 180);

    const pitchPx = pitchDeg * 3.5;

    // Deep Tactical FPV Sky Gradient
    const skyGrad = ctx.createLinearGradient(0, pitchPx - h * 1.5, 0, pitchPx);
    skyGrad.addColorStop(0, '#020617');
    skyGrad.addColorStop(0.7, '#0f172a');
    skyGrad.addColorStop(1, '#1e293b');
    ctx.fillStyle = skyGrad;
    ctx.fillRect(-w * 1.5, pitchPx - h * 3, w * 3, h * 3);

    // Deep Tactical FPV Terrain Gradient
    const groundGrad = ctx.createLinearGradient(0, pitchPx, 0, pitchPx + h * 1.5);
    groundGrad.addColorStop(0, '#0a1f18');
    groundGrad.addColorStop(0.5, '#061712');
    groundGrad.addColorStop(1, '#020b08');
    ctx.fillStyle = groundGrad;
    ctx.fillRect(-w * 1.5, pitchPx, w * 3, h * 3);

    // Horizon Line (Neon Cyan Glow)
    ctx.strokeStyle = '#00ffcc';
    ctx.shadowColor = '#00ffcc';
    ctx.shadowBlur = 8;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-w * 1.5, pitchPx);
    ctx.lineTo(w * 1.5, pitchPx);
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Perspective Terrain Grid Lines
    ctx.strokeStyle = 'rgba(0, 255, 204, 0.18)';
    ctx.lineWidth = 1;
    const gridSpeedOffset = (tick * 80 * (speedMs + 0.5)) % 60;
    for (let gy = 20; gy < h; gy += 30) {
      const yLine = pitchPx + gy + (gridSpeedOffset * (gy / h));
      if (yLine > pitchPx) {
        ctx.beginPath();
        ctx.moveTo(-w * 1.5, yLine);
        ctx.lineTo(w * 1.5, yLine);
        ctx.stroke();
      }
    }

    // Radial Perspective Vanishing Lines
    for (let vx = -w; vx <= w; vx += 70) {
      ctx.beginPath();
      ctx.moveTo(0, pitchPx);
      ctx.lineTo(vx * 2.5, pitchPx + h * 2);
      ctx.stroke();
    }

    // Dynamic Pitch Ladder (+30, +20, +10, -10, -20, -30)
    ctx.fillStyle = 'rgba(0, 255, 204, 0.9)';
    ctx.strokeStyle = 'rgba(0, 255, 204, 0.65)';
    ctx.lineWidth = 1.2;
    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    const ladderSteps = [
      { deg: 30, text: '+30' },
      { deg: 20, text: '+20' },
      { deg: 10, text: '+10' },
      { deg: -10, text: '-10' },
      { deg: -20, text: '-20' },
      { deg: -30, text: '-30' }
    ];

    ladderSteps.forEach(({ deg, text }) => {
      const ly = pitchPx - (deg * 3.5);
      const isNegative = deg < 0;
      const barW = isNegative ? 36 : 48;

      ctx.beginPath();
      if (isNegative) {
        ctx.setLineDash([4, 4]);
      } else {
        ctx.setLineDash([]);
      }
      ctx.moveTo(-barW, ly);
      ctx.lineTo(-14, ly);
      ctx.moveTo(14, ly);
      ctx.lineTo(barW, ly);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.fillText(text, -barW - 14, ly);
      ctx.fillText(text, barW + 14, ly);
    });

    ctx.restore(); // Restore horizon rotation

    // 2. Fixed Center Aircraft Reticle & Crosshair
    ctx.save();
    ctx.strokeStyle = '#ffcc00';
    ctx.fillStyle = '#ffcc00';
    ctx.shadowColor = '#ffcc00';
    ctx.shadowBlur = 6;
    ctx.lineWidth = 2;

    // Crosshair dot
    ctx.beginPath();
    ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
    ctx.fill();

    // Wings
    ctx.beginPath();
    ctx.moveTo(cx - 30, cy);
    ctx.lineTo(cx - 10, cy);
    ctx.moveTo(cx + 10, cy);
    ctx.lineTo(cx + 30, cy);
    ctx.moveTo(cx, cy - 15);
    ctx.lineTo(cx, cy - 6);
    ctx.stroke();
    ctx.shadowBlur = 0;
    ctx.restore();

    // 3. Top Compass Ribbon (Heading 0..360°)
    ctx.save();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.65)';
    ctx.fillRect(cx - 140, 6, 280, 24);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
    ctx.strokeRect(cx - 140, 6, 280, 24);

    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = '#00ffcc';

    const headingCardinals = { 0: 'N', 90: 'E', 180: 'S', 270: 'W' };
    for (let deg = -60; deg <= 60; deg += 15) {
      const hDeg = (Math.round(yawDeg) + deg + 360) % 360;
      const xPos = cx + (deg * 2.2);
      if (xPos >= cx - 130 && xPos <= cx + 130) {
        ctx.strokeStyle = deg % 30 === 0 ? 'rgba(0, 255, 204, 0.8)' : 'rgba(255, 255, 255, 0.4)';
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

    // Compass Lubber Line
    ctx.fillStyle = '#ffcc00';
    ctx.beginPath();
    ctx.moveTo(cx, 30);
    ctx.lineTo(cx - 4, 35);
    ctx.lineTo(cx + 4, 35);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    // 4. Left Airspeed Tape
    ctx.save();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.65)';
    ctx.fillRect(10, cy - 65, 55, 130);
    ctx.strokeStyle = 'rgba(0, 255, 204, 0.4)';
    ctx.strokeRect(10, cy - 65, 55, 130);

    ctx.fillStyle = '#00ffcc';
    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.textAlign = 'left';
    ctx.fillText('SPD m/s', 14, cy - 52);
    ctx.font = 'bold 15px "JetBrains Mono", monospace';
    ctx.fillText(speedMs.toFixed(1), 14, cy);
    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.fillText(`THR:${thrPct}%`, 14, cy + 48);
    ctx.restore();

    // 5. Right Altitude Tape
    ctx.save();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.65)';
    ctx.fillRect(w - 65, cy - 65, 55, 130);
    ctx.strokeStyle = 'rgba(0, 255, 204, 0.4)';
    ctx.strokeRect(w - 65, cy - 65, 55, 130);

    ctx.fillStyle = '#00ffcc';
    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.textAlign = 'left';
    ctx.fillText('ALT (Z)', w - 60, cy - 52);
    ctx.font = 'bold 15px "JetBrains Mono", monospace';
    ctx.fillText(altM.toFixed(1) + 'm', w - 60, cy);
    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.fillText(`${(altM * 3.28084).toFixed(0)} ft`, w - 60, cy + 48);
    ctx.restore();

    // 6. Bottom Stream Status Legend
    ctx.save();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.fillRect(cx - 160, h - 26, 320, 20);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
    ctx.strokeRect(cx - 160, h - 26, 320, 20);

    ctx.font = '9.5px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = state.hasLiveVideo ? '#00ffcc' : '#38bdf8';
    const statusMsg = state.hasLiveVideo 
      ? `LIVE CAMERA FEED [${state.device_type || 'GL-21B'}] 30 FPS`
      : `SIMULATED FPV FEED ACTIVE [READY FOR DRONE LINK]`;
    ctx.fillText(statusMsg, cx, h - 16);
    ctx.restore();

    ctx.restore();
  }
}
