#include <Arduino.h>
#include <Wire.h>
#include "LGFX_CrowPanel24.h"
#include "TreeDisplay.h"
#include "ImuInput.h"

static constexpr int PIN_BACKLIGHT  = 38;
static constexpr int PIN_I2C_SDA    = 15;
static constexpr int PIN_I2C_SCL    = 16;
static constexpr int PIN_TOUCH_RST  = 1;
static constexpr int PIN_TOUCH_INT  = 2;

// Backlight auto-off (same thresholds as the video mode). The IMU is the
// motion source even though the tree itself doesn't depend on it; keeping
// the dim-on-idle behavior makes the device safe to leave on a desk.
static constexpr float    STAY_AWAKE_RAD       =  2.0f * (PI / 180.0f);
static constexpr float    WAKE_RAD             = 10.0f * (PI / 180.0f);
static constexpr uint32_t BACKLIGHT_TIMEOUT_MS = 30000;

// How `age` is driven. Pick exactly one.
//
//   TREE_AGE_YAW_CTRL  — rotate the device around its vertical axis to
//                        scrub age. Yaw is unwrapped across the BNO085's
//                        ±π wrap (same idiom as the old video scrub) so
//                        continuous rotation accumulates. The accumulator
//                        is then clamped to ±YAW_RANGE_RAD, mapped 0..1.
//                        Unlike the video mode this clamps (no loop) —
//                        tree age is monotonic, so wrapping back to 0
//                        would snap the tree back to a seed.
//   TREE_AGE_AUTOGROW  — animate 0 → 1 once over GROW_DURATION_MS at boot,
//                        then hold. No physical interaction needed; good
//                        for a hands-off demo.
//   TREE_AGE_ROLL_CTRL — tilt the device sideways to scrub age. Holding
//                        level (roll == IMU_ROLL_OFFSET_RAD) is age 0.5;
//                        ±ROLL_RANGE_RAD past that maps to 0 / 1.
#define TREE_AGE_YAW_CTRL
// #define TREE_AGE_AUTOGROW
// #define TREE_AGE_ROLL_CTRL

// Half a turn from the boot orientation grows the tree from 0 → 1. Easy
// to reach without lifting the device; raise toward PI for a fuller spin.
static constexpr float    YAW_RANGE_RAD       = PI;
static constexpr uint32_t GROW_DURATION_MS    = 25000;
static constexpr float    ROLL_RANGE_RAD      = 60.0f * (PI / 180.0f);
static constexpr float    IMU_ROLL_OFFSET_RAD = PI / 2.0f;

// How far past the floor (going forward again) the user must yaw to
// re-arm the reseed latch. Just big enough that a hand-jitter at the
// floor doesn't bounce the latch open and immediately reseed twice;
// once the user has clearly moved off the floor, a future wind-back
// gets a fresh tree.
static constexpr float    YAW_RESEED_REARM_RAD = 15.0f * (PI / 180.0f);

// How far the device must tilt above/below horizontal to reach the
// extreme scene states (night ↔ bright high sky). 60° puts "well above
// the horizon" comfortably in reach without straying into the IMU's
// gimbal-lock zone. Holding the device aimed at the horizon (pitch 0)
// lands on daylight 0.5 = the canonical mid-morning blue.
static constexpr float    PITCH_DAYLIGHT_RANGE_RAD = 60.0f * (PI / 180.0f);

LGFX gfx;
TreeDisplay tree(gfx);
ImuInput imu;

// GT911 / touch-rail power-on sequence. Copied from Elecrow's factory example;
// the panel power supply is gated through this touch-controller rail on some
// units, so the display stays dark without it.
static void powerOnTouchAndPanel() {
  delay(200);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  delay(50);

  pinMode(PIN_TOUCH_RST, OUTPUT);
  pinMode(PIN_TOUCH_INT, OUTPUT);
  digitalWrite(PIN_TOUCH_RST, LOW);
  digitalWrite(PIN_TOUCH_INT, LOW);
  delay(20);
  digitalWrite(PIN_TOUCH_INT, HIGH);
  delay(100);
  pinMode(PIN_TOUCH_RST, INPUT);
}

static void showFatal(const char *msg) {
  gfx.setTextDatum(textdatum_t::top_left);
  gfx.setTextColor(TFT_RED, TFT_BLACK);
  gfx.setTextSize(2);
  gfx.drawString(msg, 8, 220);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_BACKLIGHT, OUTPUT);
  digitalWrite(PIN_BACKLIGHT, HIGH);

  powerOnTouchAndPanel();

  gfx.init();
  gfx.setRotation(0);

  if (!imu.begin(Wire, 0x4B)) {
    Serial.println("IMU init failed");
    showFatal("IMU init failed");
    while (true) delay(1000);
  }

  // Re-seed per boot from millis() if you want a different tree every
  // time the device wakes; hard-code a seed you like the look of to
  // pin the silhouette across reboots.
  if (!tree.begin(0xC0FFEEu)) {
    Serial.println("TreeDisplay: sprite allocation failed");
    showFatal("tree: alloc failed");
    while (true) delay(1000);
  }
}

void loop() {
  imu.update();

  // --- Backlight auto-off on inactivity --------------------------------
  // Identical structure to the video mode: track a reference pose that
  // updates whenever the device moves enough; if no motion crosses the
  // active threshold for BACKLIGHT_TIMEOUT_MS, drop the backlight pin.
  static bool     blInitialized   = false;
  static float    blRefPitch      = 0.0f;
  static float    blRefYaw        = 0.0f;
  static uint32_t blLastMotionMs  = 0;
  static bool     blOn            = true;
  if (imu.hasFix()) {
    if (!blInitialized) {
      blRefPitch     = imu.pitch();
      blRefYaw       = imu.yaw();
      blLastMotionMs = millis();
      blInitialized  = true;
    }
    float dPitch = imu.pitch() - blRefPitch;
    float dYaw   = imu.yaw()   - blRefYaw;
    if (dYaw >  PI) dYaw -= 2.0f * PI;
    if (dYaw < -PI) dYaw += 2.0f * PI;
    float threshold = blOn ? STAY_AWAKE_RAD : WAKE_RAD;
    if (fabsf(dPitch) > threshold || fabsf(dYaw) > threshold) {
      blRefPitch     = imu.pitch();
      blRefYaw       = imu.yaw();
      blLastMotionMs = millis();
    }
    bool shouldBeOn = (millis() - blLastMotionMs) < BACKLIGHT_TIMEOUT_MS;
    if (shouldBeOn != blOn) {
      digitalWrite(PIN_BACKLIGHT, shouldBeOn ? HIGH : LOW);
      blOn = shouldBeOn;
    }
  }

  // --- Age control -----------------------------------------------------
#if defined(TREE_AGE_YAW_CTRL)
  // Unwrap yaw across the BNO085's ±π wrap so the accumulator follows
  // real cumulative rotation (matches the old video-scrub idiom). Then
  // clamp to ±YAW_RANGE_RAD instead of folding — age 0→1 is monotonic,
  // so wrapping past the limit would snap the tree back to a seed and
  // restart growth, which is visually jarring.
  if (imu.hasFix()) {
    static bool  yawInited        = false;
    static float prevYaw          = 0.0f;
    static float unwrappedYaw     = 0.0f;
    static bool  reseededThisTrip = false; // suppress repeats until user moves off the floor
    float curYaw = imu.yaw();
    if (!yawInited) {
      prevYaw      = curYaw;
      unwrappedYaw = 0.0f;       // boot orientation = age 0.5 (mid-grown)
      yawInited    = true;
    } else {
      float dYaw = curYaw - prevYaw;
      if (dYaw >  PI) dYaw -= 2.0f * PI;
      if (dYaw < -PI) dYaw += 2.0f * PI;
      // Sign flip: clockwise device rotation grows the tree. The BNO085's
      // yaw axis increases counter-clockwise (right-hand rule about +Z),
      // so we subtract dYaw to invert the mapping.
      unwrappedYaw -= dYaw;
      prevYaw = curYaw;
    }
    if (unwrappedYaw < -YAW_RANGE_RAD) unwrappedYaw = -YAW_RANGE_RAD;
    if (unwrappedYaw >  YAW_RANGE_RAD) unwrappedYaw =  YAW_RANGE_RAD;

    // Reseed the moment the user winds back all the way to age 0. The
    // tree is invisible at this point (scale 0), so the substitution is
    // unnoticeable — the new silhouette shows up on the next forward yaw.
    // The latch fires once per visit to the floor; it re-arms only after
    // the user yaws forward past YAW_RESEED_REARM_RAD, so jitter at the
    // floor doesn't keep retriggering.
    if (unwrappedYaw <= -YAW_RANGE_RAD && !reseededThisTrip) {
      uint32_t newSeed = esp_random() | 1u;   // | 1u so the seed never lands on 0
      tree.reseed(newSeed);
      reseededThisTrip = true;
      Serial.printf("[tree] reseed at floor (seed=0x%08lx)\n",
                    (unsigned long)newSeed);
    }
    if (unwrappedYaw > -YAW_RANGE_RAD + YAW_RESEED_REARM_RAD) {
      reseededThisTrip = false;
    }

    float age = (unwrappedYaw + YAW_RANGE_RAD) / (2.0f * YAW_RANGE_RAD);
    tree.setAge(age);
  }
#elif defined(TREE_AGE_AUTOGROW)
  {
    uint32_t t = millis();
    float age = (float)t / (float)GROW_DURATION_MS;
    if (age > 1.0f) age = 1.0f;
    tree.setAge(age);
  }
#elif defined(TREE_AGE_ROLL_CTRL)
  if (imu.hasFix()) {
    float roll = imu.roll() - IMU_ROLL_OFFSET_RAD;
    float age  = (roll + ROLL_RANGE_RAD) / (2.0f * ROLL_RANGE_RAD);
    if (age < 0.0f) age = 0.0f;
    if (age > 1.0f) age = 1.0f;
    tree.setAge(age);
  }
#endif

  // --- Pitch-based time-of-day ----------------------------------------
  // Aim the device below the horizon → night sky over a near-silhouette
  // tree. At the horizon → mid-morning blue with the tree at full
  // daylight colors. Well above the horizon → very pale, bright high
  // sky. The mapping is linear in pitch with horizon at daylight 0.5.
  //
  // imu.pitch() increases as the device's top edge tilts toward the
  // user (same convention as the old pitch-band video selector); the
  // negation lets "aiming up" map to a brighter scene.
  if (imu.hasFix()) {
    float pitch    = -imu.pitch();
    float daylight = 0.5f + 0.5f * (pitch / PITCH_DAYLIGHT_RANGE_RAD);
    if (daylight < 0.0f) daylight = 0.0f;
    if (daylight > 1.0f) daylight = 1.0f;
    tree.setDaylight(daylight);
  }

  // Skip the redraw (and its DMA push) while the backlight is off — the
  // age setter above keeps the latest value live, so when motion turns
  // the panel back on the very next update() will catch up in one paint.
  if (blOn) {
    tree.update();
  }
}
