/**
 * Drone HUD & Artificial Horizon Canvas Renderer — Vercel / Geist Edition
 * Renders ultra-crisp, high-DPI pitch ladder, roll horizon, aircraft crosshair, and heading compass.
 */
class DroneHUD {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.roll = 0;   // Roll angle in degrees (-45..45)
    this.pitch = 0;  // Pitch angle in degrees (-30..30)
    this.yaw = 0;    // Heading in degrees (0..360)
    this.altitude = 0;
    this.dpr = window.devicePixelRatio || 1;
    
    this.resize();
    window.addEventListener('resize', () => this.resize());
  }

  resize() {
    if (!this.canvas || !this.canvas.parentElement) return;
    this.dpr = window.devicePixelRatio || 1;
    this.width = this.canvas.parentElement.clientWidth;
    this.height = this.canvas.parentElement.clientHeight;

    this.canvas.width = this.width * this.dpr;
    this.canvas.height = this.height * this.dpr;
    this.canvas.style.width = `${this.width}px`;
    this.canvas.style.height = `${this.height}px`;

    this.ctx.resetTransform();
    this.ctx.scale(this.dpr, this.dpr);
  }

  updateAttitude(rollVal, pitchVal, yawVal, throttleVal) {
    this.roll = (rollVal - 128) * 0.35;
    this.pitch = -(pitchVal - 128) * 0.25;
    this.yaw = ((yawVal - 128) * 1.4 + 360) % 360;
    this.altitude = (throttleVal / 255) * 100.0;
  }

  clear() {
    this.ctx.clearRect(0, 0, this.width, this.height);
  }

  render() {
    const ctx = this.ctx;
    const w = this.width;
    const h = this.height;
    const cx = w / 2;
    const cy = h / 2;

    ctx.clearRect(0, 0, w, h);

    // Dynamic Horizon Layer
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate((this.roll * Math.PI) / 180);

    const pitchPx = this.pitch * 5;

    // Sleek Artificial Horizon Line (Vercel Monochrome with subtle blue level indicator)
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.45)';
    ctx.lineWidth = 1.5;

    ctx.beginPath();
    ctx.moveTo(-130, pitchPx);
    ctx.lineTo(-40, pitchPx);
    ctx.moveTo(40, pitchPx);
    ctx.lineTo(130, pitchPx);
    ctx.stroke();

    // Center Level Notches
    ctx.beginPath();
    ctx.moveTo(-40, pitchPx);
    ctx.lineTo(-40, pitchPx + 5);
    ctx.moveTo(40, pitchPx);
    ctx.lineTo(40, pitchPx + 5);
    ctx.stroke();

    // Pitch Ladder Bars
    for (let deg = -30; deg <= 30; deg += 10) {
      if (deg === 0) continue;
      const y = pitchPx - deg * 5;
      const barLen = deg % 20 === 0 ? 40 : 26;

      ctx.beginPath();
      ctx.strokeStyle = deg > 0 ? 'rgba(0, 112, 243, 0.65)' : 'rgba(245, 158, 11, 0.65)';
      ctx.lineWidth = 1;

      // Left tick
      ctx.moveTo(-barLen, y);
      ctx.lineTo(-18, y);
      // Right tick
      ctx.moveTo(18, y);
      ctx.lineTo(barLen, y);
      ctx.stroke();

      // Pitch angle text
      ctx.fillStyle = 'rgba(255, 255, 255, 0.55)';
      ctx.font = '10px "Geist Mono", "JetBrains Mono", monospace';
      ctx.textAlign = 'right';
      ctx.fillText(Math.abs(deg).toString(), -barLen - 6, y + 3.5);
      ctx.textAlign = 'left';
      ctx.fillText(Math.abs(deg).toString(), barLen + 6, y + 3.5);
    }

    ctx.restore();

    // Stationary Center Aircraft Crosshair
    ctx.save();
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.85)';
    ctx.lineWidth = 1.5;

    // Precision Center Dot
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.arc(cx, cy, 2, 0, Math.PI * 2);
    ctx.fill();

    // Subtle aircraft reticle wings
    ctx.beginPath();
    ctx.moveTo(cx - 24, cy);
    ctx.lineTo(cx - 7, cy);
    ctx.moveTo(cx + 7, cy);
    ctx.lineTo(cx + 24, cy);
    ctx.moveTo(cx, cy - 7);
    ctx.lineTo(cx, cy - 2);
    ctx.stroke();
    ctx.restore();

    // Top Heading Compass Tape
    this.renderCompass(ctx, cx, 24);
  }

  renderCompass(ctx, cx, cy) {
    ctx.save();
    
    // Sleek dark glass capsule
    ctx.fillStyle = 'rgba(0, 0, 0, 0.75)';
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
    ctx.lineWidth = 1;
    
    const pillW = 120;
    const pillH = 24;
    ctx.beginPath();
    ctx.roundRect(cx - pillW / 2, cy - pillH / 2, pillW, pillH, 6);
    ctx.fill();
    ctx.stroke();

    // Heading readout text
    ctx.fillStyle = '#ededed';
    ctx.font = '10px "Geist Mono", "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    const currentYaw = Math.round(this.yaw);
    ctx.fillText(`${currentYaw}° HDG`, cx, cy);

    // Accent indicator needle
    ctx.fillStyle = '#0070f3';
    ctx.beginPath();
    ctx.moveTo(cx, cy + pillH / 2);
    ctx.lineTo(cx - 3, cy + pillH / 2 + 4);
    ctx.lineTo(cx + 3, cy + pillH / 2 + 4);
    ctx.closePath();
    ctx.fill();

    ctx.restore();
  }
}

window.DroneHUD = DroneHUD;
