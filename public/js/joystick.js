/**
 * Touch and Mouse Virtual Joystick Controller — Rock-Solid Stabilization
 * Provides spring-back physics, clean center deadband, and normalized [-1.0, 1.0] output.
 */
class VirtualJoystick {
  constructor(wrapperElementId, nippleElementId, onChangeCallback, autoReturnY = true) {
    this.wrapper = document.getElementById(wrapperElementId);
    this.nipple = document.getElementById(nippleElementId);
    this.onChange = onChangeCallback;
    this.autoReturnY = autoReturnY;

    this.active = false;
    this.isDragging = false;
    this.touchId = null;
    this.x = 0; // Normalized -1.0 to +1.0 (Right is +1.0, Left is -1.0)
    this.y = 0; // Normalized -1.0 to +1.0 (Up is +1.0, Down is -1.0)

    this.radius = 48; // Max displacement radius in px
    this.initEvents();
  }

  initEvents() {
    // Touch Events
    this.wrapper.addEventListener('touchstart', (e) => this.onTouchStart(e), { passive: false });
    window.addEventListener('touchmove', (e) => this.onTouchMove(e), { passive: false });
    window.addEventListener('touchend', (e) => this.onTouchEnd(e), { passive: false });
    window.addEventListener('touchcancel', (e) => this.onTouchEnd(e), { passive: false });

    // Mouse Events
    this.wrapper.addEventListener('mousedown', (e) => this.onMouseDown(e));
    window.addEventListener('mousemove', (e) => this.onMouseMove(e));
    window.addEventListener('mouseup', (e) => this.onMouseUp(e));
  }

  getCenter() {
    const rect = this.wrapper.getBoundingClientRect();
    return {
      x: rect.left + rect.width / 2,
      y: rect.top + rect.height / 2,
    };
  }

  updateStickPosition(clientX, clientY) {
    const center = this.getCenter();
    let dx = clientX - center.x;
    let dy = clientY - center.y;

    const dist = Math.hypot(dx, dy);
    if (dist > this.radius) {
      const angle = Math.atan2(dy, dx);
      dx = Math.cos(angle) * this.radius;
      dy = Math.sin(angle) * this.radius;
    }

    // Deadband threshold in pixels (3px around true center)
    const deadband = 3;
    let nx = Math.abs(dx) < deadband ? 0 : dx / this.radius;
    let ny = Math.abs(dy) < deadband ? 0 : -dy / this.radius; // -dy makes UP = +1.0

    this.x = Math.max(-1.0, Math.min(1.0, nx));
    this.y = Math.max(-1.0, Math.min(1.0, ny));

    // Move nipple visually
    this.nipple.style.transform = `translate(${dx}px, ${dy}px)`;

    if (this.onChange) {
      this.onChange(this.x, this.y);
    }
  }

  resetPosition() {
    this.active = false;
    this.isDragging = false;
    this.x = 0;
    if (this.autoReturnY) {
      this.y = 0;
      this.nipple.style.transform = 'translate(0px, 0px)';
    } else {
      const currentDy = -this.y * this.radius;
      this.nipple.style.transform = `translate(0px, ${currentDy}px)`;
    }

    if (this.onChange) {
      this.onChange(this.x, this.y);
    }
  }

  onTouchStart(e) {
    e.preventDefault();
    if (this.active) return;
    const touch = e.changedTouches[0];
    this.touchId = touch.identifier;
    this.active = true;
    this.isDragging = true;
    this.updateStickPosition(touch.clientX, touch.clientY);
  }

  onTouchMove(e) {
    if (!this.active) return;
    for (let i = 0; i < e.changedTouches.length; i++) {
      const touch = e.changedTouches[i];
      if (touch.identifier === this.touchId) {
        e.preventDefault();
        this.updateStickPosition(touch.clientX, touch.clientY);
        break;
      }
    }
  }

  onTouchEnd(e) {
    if (!this.active) return;
    for (let i = 0; i < e.changedTouches.length; i++) {
      if (e.changedTouches[i].identifier === this.touchId) {
        this.touchId = null;
        this.resetPosition();
        break;
      }
    }
  }

  onMouseDown(e) {
    this.active = true;
    this.isDragging = true;
    this.updateStickPosition(e.clientX, e.clientY);
  }

  onMouseMove(e) {
    if (!this.active) return;
    this.updateStickPosition(e.clientX, e.clientY);
  }

  onMouseUp(e) {
    if (!this.active) return;
    this.resetPosition();
  }
}

window.VirtualJoystick = VirtualJoystick;
