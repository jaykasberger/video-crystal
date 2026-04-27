#include "ImuInput.h"

bool ImuInput::begin(TwoWire &wire, uint8_t i2cAddr) {
  // BNO085 takes ~90ms typical to wake from power-up but module-level
  // LDOs can stretch that past 300ms. Wait until the sensor actually
  // ACKs a probe before invoking begin_I2C; otherwise the sh2 HAL can
  // get stuck in a state retries don't recover from.
  constexpr uint32_t PROBE_TIMEOUT_MS = 2000;
  uint32_t deadline = millis() + PROBE_TIMEOUT_MS;
  bool acked = false;
  while (millis() < deadline) {
    wire.beginTransmission(i2cAddr);
    if (wire.endTransmission() == 0) {
      acked = true;
      break;
    }
    delay(25);
  }
  if (!acked) {
    Serial.printf("ImuInput: BNO085 never ACKed 0x%02X within %ums\n",
                  i2cAddr, PROBE_TIMEOUT_MS);
    return false;
  }
  delay(100);

  for (int attempt = 1; attempt <= 3; attempt++) {
    if (_bno08x.begin_I2C(i2cAddr, &wire)) {
      enableReports();
      return true;
    }
    Serial.printf("ImuInput: begin_I2C attempt %d failed\n", attempt);
    delay(250);
  }
  return false;
}

void ImuInput::update() {
  if (_bno08x.wasReset()) {
    enableReports();
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

void ImuInput::enableReports() {
  if (!_bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
    Serial.println("ImuInput: could not enable rotation vector");
  }
  if (!_bno08x.enableReport(SH2_LINEAR_ACCELERATION, REPORT_INTERVAL_US)) {
    Serial.println("ImuInput: could not enable linear acceleration");
  }
}

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
