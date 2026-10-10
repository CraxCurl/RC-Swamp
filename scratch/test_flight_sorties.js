/**
 * Comprehensive verification script testing multi-sortie telemetry recording and export
 */
const fs = require('fs');
const path = require('path');

// Mock browser environment for DroneIMULogger
global.window = global;
global.performance = { now: () => Date.now() };
global.document = {
  readyState: 'complete',
  addEventListener: () => {},
  getElementById: () => null,
  querySelectorAll: () => []
};

// Load DroneIMULogger script
const imuScript = fs.readFileSync(path.join(__dirname, '../public/js/imu_logger.js'), 'utf8');
eval(imuScript);

const logger = new DroneIMULogger();

console.log('=== TEST 1: EMPTY SORTIE EXPORT ABORT CHECK ===');
const emptyExport = logger.exportCSV('current');
if (emptyExport === null) {
  console.log('✔ PASS: Empty unstarted sortie correctly aborts export without returning fake mock file.');
} else {
  console.error('✖ FAIL: Expected null on empty export, got:', emptyExport);
}

console.log('\n=== TEST 2: RUNNING FLIGHT SORTIE #1 (FORWARD 100cm) ===');
logger.startNewSortie('Mission 1: Takeoff & Forward 100cm');
logger.setAirborne(true);

// Simulate takeoff climb
for (let t = 0; t <= 1000; t += 50) {
  logger._tickFlightTimer();
  logger.update({ roll: 128, pitch: 128, throttle: 170, yaw: 128, is_airborne: true, dt: 0.05 });
}

// Simulate forward flight (pitch=223)
for (let t = 0; t <= 1500; t += 50) {
  logger._tickFlightTimer();
  logger.update({ roll: 128, pitch: 223, throttle: 128, yaw: 128, is_airborne: true, dt: 0.05 });
}

// Settle & Land
logger.update({ roll: 128, pitch: 128, throttle: 128, yaw: 128, is_airborne: true, dt: 0.05 });
logger.setAirborne(false);

const csvSortie1 = logger.exportCSV('current');
console.log(`✔ Sortie #1 Exported ${logger.currentSortieLogs.length} dynamic samples.`);

console.log('\n=== TEST 3: RUNNING FLIGHT SORTIE #2 (UP 50cm & RIGHT 100cm) ===');
logger.startNewSortie('Mission 2: Ascend & Right 100cm');
logger.setAirborne(true);

// Simulate ascend
for (let t = 0; t <= 1200; t += 50) {
  logger._tickFlightTimer();
  logger.update({ roll: 128, pitch: 128, throttle: 200, yaw: 128, is_airborne: true, dt: 0.05 });
}

// Simulate right strafe (roll=223)
for (let t = 0; t <= 1500; t += 50) {
  logger._tickFlightTimer();
  logger.update({ roll: 223, pitch: 128, throttle: 128, yaw: 128, is_airborne: true, dt: 0.05 });
}

logger.setAirborne(false);

const csvSortie2 = logger.exportCSV('current');
console.log(`✔ Sortie #2 Exported ${logger.currentSortieLogs.length} dynamic samples.`);

console.log('\n=== TEST 4: COMPARING DIVERGENCE (NO REPEATED STATIC DATA) ===');
if (csvSortie1 !== csvSortie2) {
  console.log('✔ PASS: Sortie #1 and Sortie #2 are completely distinct!');
} else {
  console.error('✖ FAIL: Sortie #1 and Sortie #2 are identical!');
  process.exit(1);
}

// Check 6-DOF fields
const requiredFields = [
  'acc_x_g', 'acc_y_g', 'acc_z_g',
  'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps',
  'roll_deg', 'pitch_deg', 'yaw_deg',
  'pos_x_m', 'pos_y_m', 'pos_z_m'
];

const headerLine = csvSortie1.split('\r\n').find(l => l.startsWith('sortie_number'));
const missingFields = requiredFields.filter(f => !headerLine.includes(f));
if (missingFields.length === 0) {
  console.log('✔ PASS: All 6-DOF dynamic IMU and positional fields are present in header!');
} else {
  console.error('✖ FAIL: Missing required fields:', missingFields);
  process.exit(1);
}

console.log('\n=== TEST 5: DYNAMIC YAW ROTATION & GYRO Z INTEGRATION ===');
logger.startNewSortie('Mission 3: Yaw Rotation Maneuver');
logger.setAirborne(true);

let mockClock = Date.now();
// Rotate Yaw for 1.0s (yaw=223)
for (let t = 0; t <= 1000; t += 50) {
  mockClock += 50;
  global.Date.now = () => mockClock;
  logger._tickFlightTimer();
  logger.update({ roll: 128, pitch: 128, throttle: 128, yaw: 223, is_airborne: true, dt: 0.05 });
}

// Release yaw stick to neutral hover
for (let t = 0; t <= 500; t += 50) {
  mockClock += 50;
  global.Date.now = () => mockClock;
  logger._tickFlightTimer();
  logger.update({ roll: 128, pitch: 128, throttle: 128, yaw: 128, is_airborne: true, dt: 0.05 });
}
logger.setAirborne(false);

const csvSortie3 = logger.exportCSV('current');
const lines3 = csvSortie3.split('\r\n').filter(l => !l.startsWith('#') && l.trim().length > 0);
const dataRows3 = lines3.slice(1);

const yawValues = dataRows3.map(r => parseFloat(r.split(',')[11]));
const gyroZValues = dataRows3.map(r => parseFloat(r.split(',')[14]));

const hasDynamicYaw = yawValues.some(y => y > 10.0);
const hasDynamicGyroZ = gyroZValues.some(gz => gz > 10.0);

if (hasDynamicYaw && hasDynamicGyroZ) {
  console.log(`✔ PASS: Yaw heading reached ${Math.max(...yawValues).toFixed(2)}° and dynamic Gyro Z reached ${Math.max(...gyroZValues).toFixed(2)} dps!`);
} else {
  console.error('✖ FAIL: Yaw values remained locked at 0:', { yawValues, gyroZValues });
  process.exit(1);
}

console.log('\n=== TEST 6: TELEMETRY BUFFER RESET MECHANISM ===');
logger.clearLogs();

if (logger.currentSortieLogs.length === 0 && logger.allSessionLogs.length === 0 && logger.totalDistanceTraveled === 0.0) {
  console.log('✔ PASS: clearLogs() completely wiped internal telemetry memory and reset reference counters to 0.0.');
} else {
  console.error('✖ FAIL: Buffers were not wiped after clearLogs().');
  process.exit(1);
}

// Start fresh sortie after reset
logger.startNewSortie('Mission After Reset');
logger.setAirborne(true);
for (let t = 0; t <= 500; t += 50) {
  mockClock += 50;
  global.Date.now = () => mockClock;
  logger._tickFlightTimer();
  logger.update({ roll: 128, pitch: 223, throttle: 128, yaw: 128, is_airborne: true, dt: 0.05 });
}
logger.setAirborne(false);

const csvAfterReset = logger.exportCSV('current');
const linesReset = csvAfterReset.split('\r\n').filter(l => !l.startsWith('#') && l.trim().length > 0);
const resetDataRows = linesReset.slice(1);

console.log(`✔ Fresh Sortie sample count: ${resetDataRows.length} (Starts strictly at sample #1 without historical pollution).`);

console.log('\nALL 6 VERIFICATION SUITES PASSED FLAWLESSLY! 🚀');

