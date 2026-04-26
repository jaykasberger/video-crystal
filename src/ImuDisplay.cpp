#include "ImuDisplay.h"

ImuDisplay::ImuDisplay(LGFX &gfx) : _gfx(gfx) {}

bool ImuDisplay::begin(TwoWire &wire, uint8_t i2cAddr) {
  drawLayout();

  // BNO085 takes ~90ms typical to wake from power-up, but module-level
  // LDOs and clones can stretch that past 300ms. Attempting begin_I2C
  // before the sensor is ready can leave the sh2 HAL in a state that
  // later retries don't recover from — so wait until the sensor actually
  // ACKs a probe before touching the full init.
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
    Serial.printf("BNO085 never ACKed 0x%02X within %ums\n",
                  i2cAddr, PROBE_TIMEOUT_MS);
    return false;
  }
  Serial.printf("BNO085 ACKed at 0x%02X after %ums\n",
                i2cAddr, PROBE_TIMEOUT_MS - (deadline - millis()));

  // Small settling delay after first ACK — the sensor is reachable but
  // may still be finalizing its boot state.
  delay(100);

  for (int attempt = 1; attempt <= 3; attempt++) {
    if (_bno08x.begin_I2C(i2cAddr, &wire)) {
      Serial.printf("BNO085 begin_I2C OK on attempt %d\n", attempt);
      enableReports();
      return true;
    }
    Serial.printf("BNO085 begin_I2C attempt %d failed\n", attempt);
    delay(250);
  }
  return false;
}

void ImuDisplay::drawLayout() {
  _gfx.fillScreen(TFT_BLACK);
  _gfx.setTextDatum(textdatum_t::top_left);

  _gfx.setTextColor(TFT_CYAN, TFT_BLACK);
  _gfx.setTextSize(2);
  _gfx.drawString("BNO085", 8, 8);
  _gfx.drawString("deg", _gfx.width() - 44, 8);

  _gfx.setTextColor(TFT_WHITE, TFT_BLACK);
  _gfx.setTextSize(3);
  _gfx.drawString("Yaw",   8, ROW_Y_YAW);
  _gfx.drawString("Pitch", 8, ROW_Y_PITCH);
  _gfx.drawString("Roll",  8, ROW_Y_ROLL);

  _gfx.setTextColor(TFT_CYAN, TFT_BLACK);
  _gfx.setTextSize(2);
  _gfx.drawString("Mag", 8, ROW_Y_MX - 22);
  _gfx.drawString("uT",  _gfx.width() - 32, ROW_Y_MX - 22);

  _gfx.setTextColor(TFT_WHITE, TFT_BLACK);
  _gfx.setTextSize(2);
  _gfx.drawString("Mx", 8, ROW_Y_MX);
  _gfx.drawString("My", 8, ROW_Y_MY);
  _gfx.drawString("Mz", 8, ROW_Y_MZ);
}

void ImuDisplay::update() {
  if (_bno08x.wasReset()) {
    enableReports();
  }
  if (!_bno08x.getSensorEvent(&_sensorValue)) return;

  switch (_sensorValue.sensorId) {
    case SH2_ROTATION_VECTOR: {
      float yaw, pitch, roll;
      quaternionToEuler(_sensorValue.un.rotationVector.real,
                        _sensorValue.un.rotationVector.i,
                        _sensorValue.un.rotationVector.j,
                        _sensorValue.un.rotationVector.k,
                        yaw, pitch, roll);
      constexpr float RAD_TO_DEG_F = 57.2957795f;
      drawAngle(ROW_Y_YAW,   yaw   * RAD_TO_DEG_F);
      drawAngle(ROW_Y_PITCH, pitch * RAD_TO_DEG_F);
      drawAngle(ROW_Y_ROLL,  roll  * RAD_TO_DEG_F);
      break;
    }
    case SH2_MAGNETIC_FIELD_CALIBRATED: {
      drawMag(ROW_Y_MX, _sensorValue.un.magneticField.x);
      drawMag(ROW_Y_MY, _sensorValue.un.magneticField.y);
      drawMag(ROW_Y_MZ, _sensorValue.un.magneticField.z);
      break;
    }
  }
}

void ImuDisplay::enableReports() {
  if (!_bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
    Serial.println("Could not enable rotation vector");
  }
  if (!_bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, REPORT_INTERVAL_US)) {
    Serial.println("Could not enable magnetic field");
  }
}

void ImuDisplay::drawAngle(int y, float degrees) {
  _gfx.setTextColor(TFT_YELLOW, TFT_BLACK);
  _gfx.setTextSize(3);
  _gfx.setTextDatum(textdatum_t::top_right);
  char buf[16];
  snprintf(buf, sizeof(buf), "%7.1f", degrees);
  _gfx.drawString(buf, _gfx.width() - 8, y);
}

void ImuDisplay::drawMag(int y, float microtesla) {
  _gfx.setTextColor(TFT_MAGENTA, TFT_BLACK);
  _gfx.setTextSize(2);
  _gfx.setTextDatum(textdatum_t::top_right);
  char buf[16];
  snprintf(buf, sizeof(buf), "%7.1f", microtesla);
  _gfx.drawString(buf, _gfx.width() - 8, y);
}

// ZYX Tait-Bryan (yaw, pitch, roll) in radians.
void ImuDisplay::quaternionToEuler(float qr, float qi, float qj, float qk,
                                   float &yaw, float &pitch, float &roll) {
  float sqr = qr * qr;
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;

  yaw   = atan2f(2.0f * (qi * qj + qk * qr), sqi - sqj - sqk + sqr);
  pitch = asinf(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  roll  = atan2f(2.0f * (qj * qk + qi * qr), -sqi - sqj + sqk + sqr);
}
