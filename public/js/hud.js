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
    this.roll = (rollVal - 128) * 0.35;
    this.pitch = -(pitchVal - 128) * 0.25;
    this.yaw = ((yawVal - 128) * 1.4 + 360) % 360;
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
}
