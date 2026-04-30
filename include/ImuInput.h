#pragma once

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>

// Orientation source for game/UI logic. Auto-detects which IMU board is
// wired to `wire` at begin() time and dispatches internally:
//
//   - BNO085 at i2cAddr (default 0x4B) → uses the chip's onboard fusion
//     (one rotation-vector report + one linear-accel report per tick).
//   - GY-85 (ITG3205 0x68 + ADXL345 0x53 + HMC5883L 0x1E) → reads gyro
//     and accel at 100 Hz, integrates yaw directly from the gyro axis
//     aligned with the device's vertical (auto-detected at startup) and
//     derives pitch from the accelerometer. The mag is health-checked at
//     begin but unused at runtime — fusing it pulls Euler yaw into a
//     near-singularity when the screen is held vertical.
//
// Public API and units (radians, ZYX Tait-Bryan; m/s² for accel) are the
// same regardless of which backend is active, so callers don't care.
class ImuInput {
 public:
  // Bring up whichever IMU is present on `wire`. Wire must already be
  // started by the caller (powerOnTouchAndPanel does this in main).
  // i2cAddr is the BNO085 probe address; the GY-85's three chip
  // addresses are fixed.
  bool begin(TwoWire &wire = Wire, uint8_t i2cAddr = 0x4B);

  // Pull any pending sensor events / step the AHRS filter. Call each loop().
  void update();

  float yaw()   const { return _yaw; }
  float pitch() const { return _pitch; }
  float roll()  const { return _roll; }

  bool hasFix() const { return _hasFix; }

  float accelX() const { return _accelX; }
  float accelY() const { return _accelY; }
  float accelZ() const { return _accelZ; }
  bool hasAccel() const { return _hasAccel; }

  uint32_t accelEventCount() const { return _accelEventCount; }

 private:
  enum class Backend { None, Bno085, Gy85 };

  // BNO085 path
  static constexpr long REPORT_INTERVAL_US = 20000;  // 50 Hz
  bool bnoBegin(TwoWire &wire, uint8_t addr);
  void bnoUpdate();
  void bnoEnableReports();

  // GY-85 path
  static constexpr uint8_t ITG3205_ADDR     = 0x68;
  static constexpr float   GYRO_LSB_PER_DPS = 14.375f;  // FS_SEL=3 (±2000°/s)
  static constexpr float   SAMPLE_HZ        = 100.0f;
  static constexpr uint32_t SAMPLE_PERIOD_US = (uint32_t)(1000000.0f / SAMPLE_HZ);
  bool gy85Begin(TwoWire &wire);
  void gy85Update();
  bool gy85ReadGyroDps(float &gx, float &gy, float &gz);
  bool gy85WriteReg(uint8_t reg, uint8_t val);

  static void quaternionToEuler(float qr, float qi, float qj, float qk,
                                float &yaw, float &pitch, float &roll);

  Backend  _backend{Backend::None};
  TwoWire *_wire{nullptr};

  // BNO085 state
  Adafruit_BNO08x   _bno08x{-1};  // -1: no hardware reset pin wired
  sh2_SensorValue_t _sensorValue{};

  // GY-85 state
  Adafruit_ADXL345_Unified _adxl{12345};
  Adafruit_HMC5883_Unified _hmc{12346};
  uint32_t _lastSampleUs{0};
  uint32_t _gy85FirstUpdateMs{0};
  float    _gyroBiasDps[3]{0.0f, 0.0f, 0.0f};

  // Auto-detected at startup from the gravity vector. _verticalAxisIdx is
  // the chip axis (0=X,1=Y,2=Z) that points along device-vertical when
  // held in normal use; _verticalAxisSign is +1 if that axis reads positive
  // gravity, -1 otherwise. The yaw integrator runs around this axis, and
  // pitch is derived from the gravity tilt of _pitchAxisIdx (the axis
  // immediately following vertical, cyclically).
  int   _verticalAxisIdx{2};
  float _verticalAxisSign{1.0f};
  int   _pitchAxisIdx{0};
  float _yawIntegratedRad{0.0f};

  // Output state
  float _yaw{0};
  float _pitch{0};
  float _roll{0};
  bool _hasFix{false};
  float _accelX{0};
  float _accelY{0};
  float _accelZ{0};
  bool _hasAccel{false};
  uint32_t _accelEventCount{0};
};
