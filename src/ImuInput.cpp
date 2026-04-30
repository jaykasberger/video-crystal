#include "ImuInput.h"

bool ImuInput::begin(TwoWire &wire, uint8_t i2cAddr) {
  _wire = &wire;

  // Probe BNO085 first (the original hardware). LDOs on the breakout can
  // delay ACK by a few hundred ms after power-up, so retry briefly before
  // falling through to the GY-85 detection.
  constexpr uint32_t BNO_PROBE_TIMEOUT_MS = 500;
  uint32_t deadline = millis() + BNO_PROBE_TIMEOUT_MS;
  bool bnoAcked = false;
  while (millis() < deadline) {
    wire.beginTransmission(i2cAddr);
    if (wire.endTransmission() == 0) { bnoAcked = true; break; }
    delay(25);
  }
  if (bnoAcked) {
    Serial.printf("ImuInput: BNO085 detected at 0x%02X\n", i2cAddr);
    if (bnoBegin(wire, i2cAddr)) {
      _backend = Backend::Bno085;
      return true;
    }
    Serial.println("ImuInput: BNO085 ACKed but begin_I2C failed; trying GY-85");
  }

  // Probe ITG3205 (the gyro is the most distinctive of the GY-85's three
  // chips — ADXL345's 0x53 occasionally collides with other breakouts).
  wire.beginTransmission(ITG3205_ADDR);
  if (wire.endTransmission() == 0) {
    Serial.println("ImuInput: GY-85 gyro detected at 0x68");
    if (gy85Begin(wire)) {
      _backend = Backend::Gy85;
      return true;
    }
    Serial.println("ImuInput: GY-85 init failed");
  }

  Serial.println("ImuInput: no supported IMU found");
  return false;
}

void ImuInput::update() {
  switch (_backend) {
    case Backend::Bno085: bnoUpdate();  break;
    case Backend::Gy85:   gy85Update(); break;
    case Backend::None:   break;
  }
}

// =============================================================
// BNO085 backend
// =============================================================

bool ImuInput::bnoBegin(TwoWire &wire, uint8_t addr) {
  // Probe already saw an ACK upstream; give the SHTP stack a moment to
  // come up before begin_I2C, which otherwise occasionally wedges.
  delay(100);
  for (int attempt = 1; attempt <= 3; attempt++) {
    if (_bno08x.begin_I2C(addr, &wire)) {
      bnoEnableReports();
      return true;
    }
    Serial.printf("ImuInput: begin_I2C attempt %d failed\n", attempt);
    delay(250);
  }
  return false;
}

void ImuInput::bnoUpdate() {
  if (_bno08x.wasReset()) {
    bnoEnableReports();
  }
  // Drain all pending events: with both rotation vector and linear
  // acceleration enabled at 50 Hz, the BNO085 produces ~100 events/s
  // and processing only one per loop iteration would back up the queue.
  while (_bno08x.getSensorEvent(&_sensorValue)) {
    switch (_sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        quaternionToEuler(_sensorValue.un.rotationVector.real,
                          _sensorValue.un.rotationVector.i,
                          _sensorValue.un.rotationVector.j,
                          _sensorValue.un.rotationVector.k,
                          _yaw, _pitch, _roll);
        _hasFix = true;
        break;
      case SH2_LINEAR_ACCELERATION:
        _accelX = _sensorValue.un.linearAcceleration.x;
        _accelY = _sensorValue.un.linearAcceleration.y;
        _accelZ = _sensorValue.un.linearAcceleration.z;
        _hasAccel = true;
        _accelEventCount++;
        break;
    }
  }
}

void ImuInput::bnoEnableReports() {
  if (!_bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
    Serial.println("ImuInput: could not enable rotation vector");
  }
  if (!_bno08x.enableReport(SH2_LINEAR_ACCELERATION, REPORT_INTERVAL_US)) {
    Serial.println("ImuInput: could not enable linear acceleration");
  }
}

// =============================================================
// GY-85 backend (ITG3205 + ADXL345; HMC5883L health-check only)
//
// Why no Madgwick: the screen is held vertical in this product, so the
// chip's Z axis points horizontally (out the back of the display). That
// puts the chip-frame Z-axis Euler yaw at near-singularity, smearing
// any device motion across all three Euler outputs. Replaced with a
// direct integration around whichever chip axis the gravity vector
// picks out at startup, which works regardless of how the GY-85 board
// happens to be oriented inside the enclosure.
// =============================================================

bool ImuInput::gy85WriteReg(uint8_t reg, uint8_t val) {
  _wire->beginTransmission(ITG3205_ADDR);
  _wire->write(reg);
  _wire->write(val);
  return _wire->endTransmission() == 0;
}

bool ImuInput::gy85ReadGyroDps(float &gx, float &gy, float &gz) {
  // ITG3205 register map: 0x1D=GYRO_XOUT_H, then six sequential bytes
  // for X/Y/Z high+low (big-endian, signed 16-bit). Auto-increment is
  // supported, so a single 6-byte burst returns all three axes.
  _wire->beginTransmission(ITG3205_ADDR);
  _wire->write(0x1D);
  if (_wire->endTransmission(false) != 0) return false;
  if (_wire->requestFrom((uint8_t)ITG3205_ADDR, (uint8_t)6) != 6) return false;
  int16_t rx = (int16_t)((_wire->read() << 8) | _wire->read());
  int16_t ry = (int16_t)((_wire->read() << 8) | _wire->read());
  int16_t rz = (int16_t)((_wire->read() << 8) | _wire->read());
  gx = rx / GYRO_LSB_PER_DPS;
  gy = ry / GYRO_LSB_PER_DPS;
  gz = rz / GYRO_LSB_PER_DPS;
  return true;
}

bool ImuInput::gy85Begin(TwoWire &wire) {
  // ADXL345 — Adafruit unified driver, default I²C 0x53. Returns m/s².
  if (!_adxl.begin()) {
    Serial.println("ImuInput: ADXL345 begin failed");
    return false;
  }
  _adxl.setRange(ADXL345_RANGE_4_G);
  _adxl.setDataRate(ADXL345_DATARATE_100_HZ);

  // HMC5883L — Adafruit unified driver, default I²C 0x1E. Returns µT.
  // NOTE: yaw will be hard-iron-biased without a per-device mag
  // calibration. The scrub in main.cpp uses delta-yaw, so this mostly
  // shows up as direction sensitivity, not as a broken control. If
  // absolute heading is ever needed, add a calibration step that
  // captures min/max mag readings while the device is rotated through
  // all axes, and subtract the midpoint as the hard-iron offset.
  if (!_hmc.begin()) {
    Serial.println("ImuInput: HMC5883L begin failed");
    return false;
  }

  // ITG3205:
  //   PWR_MGM (0x3E) ← 0x01: clock source = X-gyro (more stable than the
  //                          internal oscillator).
  //   DLPF_FS (0x16) ← (FS_SEL=3 << 3) | DLPF_CFG=3
  //                   = ±2000°/s full scale, 42 Hz LPF, 1 kHz internal.
  //   SMPLRT_DIV (0x15) ← 9: output rate = 1000 / (9+1) = 100 Hz,
  //                          matching FILTER_HZ.
  if (!gy85WriteReg(0x3E, 0x01)) {
    Serial.println("ImuInput: ITG3205 PWR_MGM write failed");
    return false;
  }
  gy85WriteReg(0x16, (3 << 3) | 3);
  gy85WriteReg(0x15, 9);
  delay(50);

  // Calibrate gyro bias AND detect the chip axis aligned with device-
  // vertical. The user is asked to hold the device in normal-use
  // orientation; the axis whose accel reads ~±g is the yaw-rotation axis,
  // and the axis cyclically next to it becomes the pitch-tilt axis.
  // Without an explicit axis pick we'd have to guess based on how the
  // GY-85 board is mounted inside the enclosure, which varies.
  Serial.println("ImuInput: GY-85 calibrating (hold device upright and still)...");
  const int   N    = 100;
  const int   WAIT_MS = 5;
  double gSum[3] = {0, 0, 0};
  double aSum[3] = {0, 0, 0};
  int got = 0;
  for (int i = 0; i < N; i++) {
    float gx, gy, gz;
    if (gy85ReadGyroDps(gx, gy, gz)) {
      gSum[0] += gx; gSum[1] += gy; gSum[2] += gz;
      sensors_event_t aevt;
      _adxl.getEvent(&aevt);
      aSum[0] += aevt.acceleration.x;
      aSum[1] += aevt.acceleration.y;
      aSum[2] += aevt.acceleration.z;
      got++;
    }
    delay(WAIT_MS);
  }
  if (got < N / 2) {
    Serial.println("ImuInput: gyro calibration read too few samples");
    return false;
  }
  for (int i = 0; i < 3; i++) _gyroBiasDps[i] = (float)(gSum[i] / got);
  Serial.printf("ImuInput: gyro bias (dps) = %.3f %.3f %.3f over %d samples\n",
                _gyroBiasDps[0], _gyroBiasDps[1], _gyroBiasDps[2], got);

  float aAvg[3] = {(float)(aSum[0]/got), (float)(aSum[1]/got), (float)(aSum[2]/got)};
  int idx = 0;
  if (fabsf(aAvg[1]) > fabsf(aAvg[idx])) idx = 1;
  if (fabsf(aAvg[2]) > fabsf(aAvg[idx])) idx = 2;
  _verticalAxisIdx  = idx;
  _verticalAxisSign = (aAvg[idx] >= 0.0f) ? 1.0f : -1.0f;
  _pitchAxisIdx     = (idx + 1) % 3;
  Serial.printf("ImuInput: GY-85 vertical=%c%c (g=%.2f m/s²), pitch axis=%c\n",
                _verticalAxisSign > 0 ? '+' : '-', "XYZ"[idx], aAvg[idx],
                "XYZ"[_pitchAxisIdx]);

  _yawIntegratedRad  = 0.0f;
  _lastSampleUs      = micros();
  _gy85FirstUpdateMs = 0;
  return true;
}

void ImuInput::gy85Update() {
  // Rate-limit to SAMPLE_HZ — capping I²C bandwidth so the SD/JPEG path
  // still gets per-frame time, not because the integration itself needs
  // a fixed step (it uses actual elapsed dt).
  uint32_t nowUs = micros();
  uint32_t elapsedUs = nowUs - _lastSampleUs;
  if (elapsedUs < SAMPLE_PERIOD_US) return;
  // If we fell badly behind (e.g. SD I/O blocked us), cap dt so a long
  // stall doesn't turn into a giant integrated yaw step on resume.
  if (elapsedUs > SAMPLE_PERIOD_US * 4) elapsedUs = SAMPLE_PERIOD_US;
  _lastSampleUs = nowUs;
  float dt = elapsedUs * 1e-6f;

  float gRaw[3];
  if (!gy85ReadGyroDps(gRaw[0], gRaw[1], gRaw[2])) return;
  for (int i = 0; i < 3; i++) gRaw[i] -= _gyroBiasDps[i];

  sensors_event_t aevt;
  _adxl.getEvent(&aevt);
  float a[3] = {aevt.acceleration.x, aevt.acceleration.y, aevt.acceleration.z};

  // Yaw = integrate gyro rate around the gravity-aligned chip axis. No
  // mag fusion: the HMC5883 on the GY-85 board is rotated relative to the
  // ITG/ADXL, and feeding it into a Euler-yaw filter in this orientation
  // is what was producing the cross-axis smearing we just removed. Drift
  // is acceptable because main.cpp's scrub is delta-yaw based.
  const float DEG2RAD = PI / 180.0f;
  _yawIntegratedRad += _verticalAxisSign * gRaw[_verticalAxisIdx] * DEG2RAD * dt;
  while (_yawIntegratedRad >  PI) _yawIntegratedRad -= 2.0f * PI;
  while (_yawIntegratedRad < -PI) _yawIntegratedRad += 2.0f * PI;
  _yaw = _yawIntegratedRad;

  // Pitch = signed gravity tilt of the pitch axis relative to vertical.
  // Static accuracy is excellent; during fast handling motion the linear
  // acceleration adds noise, but the hysteresis in main.cpp's video
  // selection absorbs that.
  float aVert  = _verticalAxisSign * a[_verticalAxisIdx];
  _pitch = atan2f(a[_pitchAxisIdx], aVert);

  // Roll: not used by main.cpp but populated for parity with the BNO085
  // path and for serial-debug visibility.
  int rollAxisIdx = 3 - _verticalAxisIdx - _pitchAxisIdx;
  _roll = atan2f(a[rollAxisIdx], aVert);

  _accelX = a[0];
  _accelY = a[1];
  _accelZ = a[2];
  _hasAccel = true;
  _accelEventCount++;

  // Short settling window: gyro bias was already calibrated in begin(),
  // so we only need to wait for the first valid accel-derived pitch.
  constexpr uint32_t SETTLE_MS = 200;
  if (_gy85FirstUpdateMs == 0) _gy85FirstUpdateMs = millis();
  if (!_hasFix && (millis() - _gy85FirstUpdateMs) > SETTLE_MS) _hasFix = true;
}

// =============================================================
// shared
// =============================================================

void ImuInput::quaternionToEuler(float qr, float qi, float qj, float qk,
                                 float &yaw, float &pitch, float &roll) {
  float sqr = qr * qr;
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;

  yaw   = atan2f(2.0f * (qi * qj + qk * qr), sqi - sqj - sqk + sqr);
  pitch = asinf(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  roll  = atan2f(2.0f * (qj * qk + qi * qr), -sqi - sqj + sqk + sqr);
}
