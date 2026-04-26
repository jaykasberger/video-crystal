#pragma once

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "LGFX_CrowPanel24.h"

// One display mode: renders orientation (yaw/pitch/roll) and calibrated
// magnetometer (x/y/z) from a BNO085 connected over I²C. Owns the sensor
// so the main loop only has to call begin() once and update() per tick.
class ImuDisplay {
 public:
  explicit ImuDisplay(LGFX &gfx);

  // Initialize the BNO085 on `wire` at `i2cAddr`, enable the orientation
  // and magnetometer reports, and draw the static layout. Returns false
  // if the sensor can't be reached.
  bool begin(TwoWire &wire = Wire, uint8_t i2cAddr = 0x4B);

  // Redraw the static labels without touching the sensor. Call this when
  // switching back to this mode from a different mode that clobbered the
  // screen.
  void drawLayout();

  // Poll the sensor and redraw any updated values. Call each loop().
  void update();

 private:
  static constexpr long REPORT_INTERVAL_US = 20000;  // 50 Hz

  static constexpr int ROW_Y_YAW   =  40;
  static constexpr int ROW_Y_PITCH =  74;
  static constexpr int ROW_Y_ROLL  = 108;
  static constexpr int ROW_Y_MX    = 150;
  static constexpr int ROW_Y_MY    = 174;
  static constexpr int ROW_Y_MZ    = 198;

  LGFX &_gfx;
  Adafruit_BNO08x _bno08x{-1};  // -1: no hardware reset pin wired
  sh2_SensorValue_t _sensorValue{};

  void enableReports();
  void drawAngle(int y, float degrees);
  void drawMag(int y, float microtesla);

  static void quaternionToEuler(float qr, float qi, float qj, float qk,
                                float &yaw, float &pitch, float &roll);
};
