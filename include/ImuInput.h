#pragma once

#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Thin wrapper around the BNO085 that exposes the latest orientation as
// yaw/pitch/roll (radians, ZYX Tait-Bryan). No display rendering — meant
// to drive game/UI logic in main.cpp.
class ImuInput {
 public:
  // Bring up the sensor on the given Wire/address. Wire must already be
  // started by the caller (powerOnTouchAndPanel does this in main).
  bool begin(TwoWire &wire = Wire, uint8_t i2cAddr = 0x4B);

  // Pull any pending sensor events. Call each loop().
  void update();

  // Latest orientation in radians.
  float yaw()   const { return _yaw; }
  float pitch() const { return _pitch; }
  float roll()  const { return _roll; }

  // True once we've received at least one rotation-vector event.
  bool hasFix() const { return _hasFix; }

 private:
  static constexpr long REPORT_INTERVAL_US = 20000;  // 50 Hz

  void enableReports();
  static void quaternionToEuler(float qr, float qi, float qj, float qk,
                                float &yaw, float &pitch, float &roll);

  Adafruit_BNO08x _bno08x{-1};
  sh2_SensorValue_t _sensorValue{};
  float _yaw{0};
  float _pitch{0};
  float _roll{0};
  bool _hasFix{false};
};
