// Test Suite: Node.js RC UFO Protocol Calculations & Checksum Verification
const assert = require('assert');

function computeGLChecksum(r, p, t, y, flags1, flags2) {
  return (flags1 ^ (((p ^ r) ^ t) ^ y)) ^ (flags2 & 0xFF);
}

function buildControlPacket(state) {
  const r = Math.max(1, Math.min(255, Math.round(state.roll)));
  const p = Math.max(1, Math.min(255, Math.round(state.pitch)));
  const t = Math.max(0, Math.min(255, Math.round(state.throttle)));
  const y = Math.max(1, Math.min(255, Math.round(state.yaw)));

  if (state.deviceType !== 10) {
    let flags1 = 0;
    if (state.isFastFly || state.isFastDrop) flags1 |= 0x01;
    if (state.isEmergencyStop) flags1 |= 0x02;
    if (state.isGyroCorrection) flags1 |= 0x04;
    if (state.isCircleTurnEnd) flags1 |= 0x08;
    if (state.isGestureMode) flags1 |= 0x40;

    let flags2 = 0;
    if (state.isNoHeadMode) flags2 |= 0x01;
    if (state.isFixedHeight) flags2 |= 0x02;

    const cs = (flags1 ^ (((p ^ r) ^ t) ^ y)) ^ (flags2 & 0xFF);

    const pkt = Buffer.alloc(21);
    pkt[0] = 0x03;
    pkt[1] = 0x66;
    pkt[2] = 0x14;
    pkt[3] = r;
    pkt[4] = p;
    pkt[5] = t;
    pkt[6] = y;
    pkt[7] = flags1;
    pkt[8] = flags2;
    pkt[19] = cs & 0xFF;
    pkt[20] = 0x99;
    return pkt;
  } else {
    let flags = 0;
    if (state.isFastFly) flags += 1;
    if (state.isFastDrop) flags += 2;
    if (state.isEmergencyStop) flags += 4;
    if (state.isCircleTurnEnd) flags += 8;
    if (state.isNoHeadMode) flags += 16;
    if (state.isGyroCorrection) flags += 128;

    const cs = (((r ^ p) ^ t) ^ y) ^ (flags & 0xFF);
    return Buffer.from([
      0x03, 0x66, r, p, t, y, flags & 0xFF, cs & 0xFF, 0x99
    ]);
  }
}

// 1. Verify Neutral Hover GL Packet
const neutralState = { roll: 128, pitch: 128, throttle: 128, yaw: 128, deviceType: 2 };
const glPkt = buildControlPacket(neutralState);
assert.strictEqual(glPkt.length, 21, "GL Packet must be 21 bytes");
assert.strictEqual(glPkt[0], 0x03);
assert.strictEqual(glPkt[1], 0x66);
assert.strictEqual(glPkt[2], 0x14);
assert.strictEqual(glPkt[19], 0x00, "Neutral hover checksum must be 0x00");
assert.strictEqual(glPkt[20], 0x99);

// 2. Verify Legacy Packet
const legacyState = { roll: 128, pitch: 128, throttle: 128, yaw: 128, deviceType: 10 };
const legPkt = buildControlPacket(legacyState);
assert.strictEqual(legPkt.length, 9, "Legacy Packet must be 9 bytes");
assert.strictEqual(legPkt[7], 0x00, "Legacy neutral checksum must be 0x00");

// 3. Verify Flags Calculation
const takeoffState = { roll: 128, pitch: 128, throttle: 128, yaw: 128, isFastFly: true, deviceType: 2 };
const takeoffPkt = buildControlPacket(takeoffState);
assert.strictEqual(takeoffPkt[7], 0x01, "Flags 1 should have bit 0 set");
assert.strictEqual(takeoffPkt[19], 0x01, "Checksum should be 0x01");

console.log("All Node.js protocol calculation assertions passed successfully!");
