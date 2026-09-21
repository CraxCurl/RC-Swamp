/**
 * Drone HUD & Artificial Horizon Canvas Renderer — Vercel Edition
 * Renders minimalist pitch ladder, roll arc, attitude crosshair, and heading compass.
 */
class DroneHUD {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.roll = 0;   // Roll angle in degrees (-45..45)
    this.pitch = 0;  // Pitch angle in degrees (-30..30)
    this.yaw = 0;    // Heading in degrees (0..360)
    this.altitude = 0;
    
    this.resize();
    window.addEventListener('resize', () => this.resize());
  }

  resize() {
    this.width = this.canvas.parentElement.clientWidth;
    this.height = this.canvas.parentElement.clientHeight;
    this.canvas.width = this.width;
    this.canvas.height = this.height;
  }

  updateAttitude(rollVal, pitchVal, yawVal, throttleVal) {
    this.roll = (rollVal - 128) * 0.35;
    this.pitch = -(pitchVal - 128) * 0.25;
    this.yaw = ((yawVal - 128) * 1.4 + 360) % 360;
    this.altitude = (throttleVal / 255) * 100.0;
  }

  render() {
    const ctx = this.ctx;
    const w = this.width;
    const h = this.height;
    const cx = w / 2;
    const cy = h / 2;

    ctx.clearRect(0, 0, w, h);

    ctx.save();
    // Translate to center and rotate by roll
    ctx.translate(cx, cy);
    ctx.rotate((this.roll * Math.PI) / 180);

    const pitchPx = this.pitch * 5;

    // --- Minimalist Artificial Horizon Line ---
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.6)';
    ctx.lineWidth = 1.5;

    ctx.beginPath();
    ctx.moveTo(-140, pitchPx);
    ctx.lineTo(-40, pitchPx);
    ctx.moveTo(40, pitchPx);
    ctx.lineTo(140, pitchPx);
    ctx.stroke();

    // Center Level Notch
    ctx.beginPath();
    ctx.moveTo(-40, pitchPx);
    ctx.lineTo(-40, pitchPx + 6);
    ctx.moveTo(40, pitchPx);
    ctx.lineTo(40, pitchPx + 6);
    ctx.stroke();

    // --- Pitch Ladder Bars ---
    for (let deg = -30; deg <= 30; deg += 10) {
      if (deg === 0) continue;
      const y = pitchPx - deg * 5;
      const barLen = deg % 20 === 0 ? 45 : 30;

      ctx.beginPath();
      ctx.strokeStyle = deg > 0 ? 'rgba(0, 112, 243, 0.7)' : 'rgba(245, 158, 11, 0.7)';
      ctx.lineWidth = 1;

      // Left tick
      ctx.moveTo(-barLen, y);
      ctx.lineTo(-20, y);
      // Right tick
      ctx.moveTo(20, y);
      ctx.lineTo(barLen, y);
      ctx.stroke();

      // Pitch angle text
      ctx.fillStyle = 'rgba(255, 255, 255, 0.6)';
      ctx.font = '9px "Geist Mono", monospace';
      ctx.fillText(Math.abs(deg).toString(), -barLen - 16, y + 3);
      ctx.fillText(Math.abs(deg).toString(), barLen + 6, y + 3);
    }

    ctx.restore();

    // --- Center Stationary Reticle / Crosshair ---
    ctx.save();
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
    ctx.lineWidth = 1.5;

    // Crosshair dot
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
    ctx.fill();

    // Stationary wings
    ctx.beginPath();
    ctx.moveTo(cx - 24, cy);
    ctx.lineTo(cx - 8, cy);
    ctx.moveTo(cx + 8, cy);
    ctx.lineTo(cx + 24, cy);
    ctx.moveTo(cx, cy - 8);
    ctx.lineTo(cx, cy - 2);
    ctx.stroke();
    ctx.restore();

    // --- Heading / Compass Tape at Top ---
    this.renderCompass(ctx, cx, 28);
  }

  renderCompass(ctx, cx, cy) {
    ctx.save();
    ctx.fillStyle = 'rgba(10, 10, 10, 0.8)';
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.roundRect(cx - 70, cy - 14, 140, 24, 6);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = '#ffffff';
    ctx.font = '10px "Geist Mono", monospace';
    ctx.textAlign = 'center';

    const currentYaw = Math.round(this.yaw);
    ctx.fillText(`${currentYaw}° HDG`, cx, cy + 3);

    // Center Pointer
    ctx.fillStyle = '#0070f3';
    ctx.beginPath();
    ctx.moveTo(cx, cy + 10);
    ctx.lineTo(cx - 3, cy + 14);
    ctx.lineTo(cx + 3, cy + 14);
    ctx.closePath();
    ctx.fill();

    ctx.restore();
  }
}
window.DroneHUD = DroneHUD;
