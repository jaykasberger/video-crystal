#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "LGFX_CrowPanel24.h"
#include "VideoPlayer.h"
#include "ImuInput.h"

static constexpr int PIN_BACKLIGHT  = 38;
static constexpr int PIN_I2C_SDA    = 15;
static constexpr int PIN_I2C_SCL    = 16;
static constexpr int PIN_TOUCH_RST  = 1;
static constexpr int PIN_TOUCH_INT  = 2;

static constexpr int PIN_SD_SCK   = 5;
static constexpr int PIN_SD_MOSI  = 6;
static constexpr int PIN_SD_MISO  = 4;
static constexpr int PIN_SD_CS    = 7;

// Three videos, selected by pitch:
//   pitch >  PITCH_HIGH_DEG : video 1
//   pitch in [LOW, HIGH]    : video 2
//   pitch <  PITCH_LOW_DEG  : video 3
// VIDEO_PATHS[0] is the "level/middle" video that gets loaded at startup.
static constexpr const char *VIDEO_PATHS[3] = {
  "/video-2.mjp",  // index 0: middle pose (level), shown first
  "/video-1.mjp",  // index 1: tilted up
  "/video-3.mjp",  // index 2: tilted down
};
static constexpr float PITCH_HIGH_DEG =   0.0f;
static constexpr float PITCH_LOW_DEG  = -25.0f;
static constexpr float PITCH_HYST_DEG =   3.0f;

// Backlight auto-off: if neither pitch nor yaw changes by more than
// MOTION_THRESHOLD_RAD for BACKLIGHT_TIMEOUT_MS, drive PIN_BACKLIGHT
// LOW. Any subsequent motion turns it back on.
static constexpr float    MOTION_THRESHOLD_RAD = 2.0f * (PI / 180.0f);  // ~2°
static constexpr uint32_t BACKLIGHT_TIMEOUT_MS = 30000;

// --- Pitch-based scrub (active) --------------------------------------
//
// Tilt the device around its X axis (rocking the top edge toward or
// away from you) to scrub through the video. Same shape of mapping as
// the original roll-based control: ±RANGE → frame 0…N-1, level → middle.
//
// If the wrong sensor axis is being used, swap imu.pitch() in loop()
// for imu.yaw() or imu.roll(). If direction is reversed, flip the sign
// of `angle` at the call site.
static constexpr float SCRUB_PITCH_RANGE_RAD = 90.0f * (PI / 180.0f);
static constexpr float IMU_PITCH_OFFSET_RAD  = 0.0f;

// --- Translation-based scrub (commented out for fallback) ------------
// Drove scrub from integrated lateral acceleration. Worked, but the
// BNO085's onboard fusion would eventually decide our motion was noise
// and stop reporting it (events kept flowing at 50 Hz with z=0 values),
// freezing the scrub. Switched to rotation-based input instead.
// static constexpr float    SCRUB_GAIN     = 60.0f;
// static constexpr float    ACCEL_DEADBAND = 0.10f;  // m/s²
// static constexpr float    REST_THRESHOLD = 0.15f;  // m/s²
// static constexpr uint32_t REST_SAMPLES   = 30;

// --- Roll-based scrub (kept commented out for fallback) --------------
// static constexpr float SCRUB_ROLL_RANGE_RAD = 60.0f * (PI / 180.0f);
// static constexpr float IMU_ROLL_OFFSET_RAD  = PI / 2.0f;

LGFX gfx;
VideoPlayer videoPlayer(gfx);
ImuInput imu;

// SD on SPI3 (HSPI on ESP32-S3 Arduino); LGFX owns SPI2 for the display.
SPIClass sdSpi(HSPI);

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

static void drawSDStatus(const char *msg, uint16_t color) {
  gfx.setTextDatum(textdatum_t::top_left);
  gfx.setTextColor(color, TFT_BLACK);
  gfx.setTextSize(2);
  gfx.fillRect(0, 220, gfx.width(), 18, TFT_BLACK);
  gfx.drawString(msg, 8, 220);
}

static void initSDCard() {
  sdSpi.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, sdSpi, 25000000)) {
    Serial.println("SD mount failed");
    drawSDStatus("SD: not found", TFT_RED);
    return;
  }
  const char *typeStr = "UNK";
  switch (SD.cardType()) {
    case CARD_NONE: typeStr = "none"; break;
    case CARD_MMC:  typeStr = "MMC";  break;
    case CARD_SD:   typeStr = "SD";   break;
    case CARD_SDHC: typeStr = "SDHC"; break;
  }
  uint64_t cardMB = SD.cardSize() / (1024ULL * 1024ULL);
  char buf[64];
  snprintf(buf, sizeof(buf), "SD: %s %lluMB", typeStr, cardMB);
  Serial.println(buf);
  drawSDStatus(buf, TFT_GREEN);

  // Dump root dir to serial as a smoke test.
  File root = SD.open("/");
  if (root) {
    Serial.println("SD / contents:");
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      Serial.printf("  %s%s  %u\n",
                    f.name(),
                    f.isDirectory() ? "/" : "",
                    (unsigned)f.size());
    }
    root.close();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_BACKLIGHT, OUTPUT);
  digitalWrite(PIN_BACKLIGHT, HIGH);

  powerOnTouchAndPanel();

  gfx.init();
  // setRotation(2), not (1): the LGFX panel config's offset_rotation=3
  // means the user-facing rotation 2 lands on hardware MADCTL rotation 1
  // (landscape, 320x240 native scan), which is what pushImageDMA needs.
  // drawString/fillRect work at any rotation because LGFX rotates their
  // coordinates in software, but raw image push needs hw to match buffer.
  gfx.setRotation(0);

  initSDCard();

  if (!imu.begin(Wire, 0x4B)) {
    Serial.println("IMU init failed");
    showFatal("IMU init failed");
    while (true) delay(1000);
  }

  // Start with the middle/level video; pitch selector in loop() will
  // switch among the three as the device tilts.
  if (!videoPlayer.begin(VIDEO_PATHS[0])) {
    Serial.printf("VideoPlayer: failed to open %s\n", VIDEO_PATHS[0]);
    showFatal("video file missing/invalid");
    while (true) delay(1000);
  }

  // Start in the middle so the first frame painted (before any IMU
  // reading arrives) matches the level-device position.
  videoPlayer.seekToFrame(videoPlayer.frameCount() / 2);
}

void loop() {
  imu.update();

  // Yaw scrub state — declared at function scope so both the video
  // selection block and the scrub block can touch it. The scrub block
  // unwraps yaw across the BNO085's ±π wrap, accumulating cumulative
  // rotation; the selection block resets it on a successful video
  // switch so each new video starts at its middle frame.
  static bool  yawInited     = false;
  static float prevYaw       = 0.0f;
  static float unwrappedYaw  = 0.0f;

  // --- Backlight auto-off on inactivity --------------------------------
  // Track a "reference" orientation that updates whenever the device
  // moves enough; the timestamp of that update is our last-motion time.
  // Comparing each sample to the *reference* (rather than the previous
  // sample) means slow drift still eventually trips the threshold.
  static bool   blInitialized = false;
  static float  blRefPitch    = 0.0f;
  static float  blRefYaw      = 0.0f;
  static uint32_t blLastMotionMs = 0;
  static bool   blOn          = true;
  if (imu.hasFix()) {
    if (!blInitialized) {
      blRefPitch = imu.pitch();
      blRefYaw   = imu.yaw();
      blLastMotionMs = millis();
      blInitialized = true;
    }

    float dPitch = imu.pitch() - blRefPitch;
    float dYaw   = imu.yaw()   - blRefYaw;
    // Yaw wraps at ±π — fold the difference into [-π, π].
    if (dYaw >  PI) dYaw -= 2.0f * PI;
    if (dYaw < -PI) dYaw += 2.0f * PI;

    if (fabsf(dPitch) > MOTION_THRESHOLD_RAD ||
        fabsf(dYaw)   > MOTION_THRESHOLD_RAD) {
      blRefPitch = imu.pitch();
      blRefYaw   = imu.yaw();
      blLastMotionMs = millis();
    }

    bool shouldBeOn = (millis() - blLastMotionMs) < BACKLIGHT_TIMEOUT_MS;
    if (shouldBeOn != blOn) {
      digitalWrite(PIN_BACKLIGHT, shouldBeOn ? HIGH : LOW);
      blOn = shouldBeOn;
    }
  }

  // --- Pitch-based video selection -------------------------------------
  // Selects which of the three videos is active. PITCH_HYST_DEG
  // prevents flicker at the boundaries: once you've entered a band you
  // have to overshoot the threshold by HYST degrees to leave it.
  // currentVideo holds an index into VIDEO_PATHS: 0=mid, 1=high, 2=low.
  static int currentVideo = 0;
  if (imu.hasFix()) {
    float pitchDeg = -imu.pitch() * (180.0f / PI);
    int target = currentVideo;
    switch (currentVideo) {
      case 1:  // currently "high"; need to drop past HIGH-HYST to leave
        if (pitchDeg < PITCH_HIGH_DEG - PITCH_HYST_DEG) {
          target = (pitchDeg < PITCH_LOW_DEG) ? 2 : 0;
        }
        break;
      case 2:  // currently "low"; need to rise past LOW+HYST to leave
        if (pitchDeg > PITCH_LOW_DEG + PITCH_HYST_DEG) {
          target = (pitchDeg > PITCH_HIGH_DEG) ? 1 : 0;
        }
        break;
      default:  // currently "mid"; cross full thresholds to leave
        if (pitchDeg > PITCH_HIGH_DEG)      target = 1;
        else if (pitchDeg < PITCH_LOW_DEG)  target = 2;
        break;
    }
    if (target != currentVideo) {
      if (videoPlayer.switchTo(VIDEO_PATHS[target])) {
        currentVideo = target;
        // Anchor the scrub origin at the current yaw so the new video
        // starts at its middle frame and dYaw = 0 on the next sample.
        unwrappedYaw = 0.0f;
        prevYaw      = imu.yaw();
      }
    }
  }

  // --- Yaw-based scrub (active, looping) -------------------------------
  // Continuous rotation accumulates into `unwrappedYaw` (handling the
  // BNO085's ±π wrap), so the user can keep turning in one direction
  // and the video loops past the start/end instead of clamping.
  if (imu.hasFix() && videoPlayer.frameCount() > 0) {
    float curYaw = imu.yaw();
    if (!yawInited) {
      prevYaw = curYaw;
      unwrappedYaw = 0.0f;
      yawInited = true;
    } else {
      float dYaw = curYaw - prevYaw;
      if (dYaw >  PI) dYaw -= 2.0f * PI;
      if (dYaw < -PI) dYaw += 2.0f * PI;
      unwrappedYaw += dYaw;
      prevYaw = curYaw;

      // Fold unwrappedYaw back into [-RANGE, +RANGE] every iteration so
      // it can't accumulate without bound over long sessions and erode
      // float precision. A fold of 2*RANGE corresponds to one full loop
      // of the video, so the visible frame is unchanged by the fold.
      const float TWO_RANGE = 2.0f * SCRUB_PITCH_RANGE_RAD;
      while (unwrappedYaw >  SCRUB_PITCH_RANGE_RAD) unwrappedYaw -= TWO_RANGE;
      while (unwrappedYaw < -SCRUB_PITCH_RANGE_RAD) unwrappedYaw += TWO_RANGE;
    }

    float t = (unwrappedYaw + SCRUB_PITCH_RANGE_RAD) /
              (2.0f * SCRUB_PITCH_RANGE_RAD);
    t = t - floorf(t);  // wrap to [0, 1)
    uint32_t target = (uint32_t)(t * (videoPlayer.frameCount() - 1));
    videoPlayer.seekToFrame(target);
  }

  // --- Translation-based scrub (commented out for fallback) ------------
  // See constants block above for why this was abandoned. The full body
  // (with ZUPT, NaN sanitization, and 1-Hz scrub log) lived here.

  // --- Roll-based scrub (kept commented out for fallback) --------------
  // if (imu.hasFix() && videoPlayer.frameCount() > 0) {
  //   float roll = imu.roll() - IMU_ROLL_OFFSET_RAD;
  //   float t = (roll + SCRUB_ROLL_RANGE_RAD) / (2.0f * SCRUB_ROLL_RANGE_RAD);
  //   if (t < 0.0f) t = 0.0f;
  //   if (t > 1.0f) t = 1.0f;
  //   uint32_t target = (uint32_t)(t * (videoPlayer.frameCount() - 1));
  //   videoPlayer.seekToFrame(target);
  // }

  // Skip the (expensive) SD read + JPEG decode + DMA push while the
  // backlight is off — there's nothing to see, so no point paying the
  // CPU/SD cost. seekToFrame() above keeps _requestedIdx current, so
  // when the backlight comes back on the next update() call will decode
  // and push the right frame.
  if (blOn) {
    videoPlayer.update();
  }
}
