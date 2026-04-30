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

// Three pitch-band slots populated from whatever .mjp files are on the SD
// card root, sorted alphabetically:
//   videoPaths[0] = level/middle pose (also the first file shown at boot)
//   videoPaths[1] = tilted up   (pitch >  PITCH_HIGH_DEG)
//   videoPaths[2] = tilted down (pitch <  PITCH_LOW_DEG)
// If fewer than three .mjp files are present, the missing slots fall back
// to videoPaths[0] so the pitch-switching logic always has a valid path
// to pass to VideoPlayer::switchTo().
static constexpr int  MAX_VIDEO_SLOTS  = 3;
static constexpr int  MAX_PATH_LEN     = 64;
static char videoPaths[MAX_VIDEO_SLOTS][MAX_PATH_LEN] = {{0}};
static int  videoCount = 0;
static constexpr float PITCH_HIGH_DEG =   20.0f;
static constexpr float PITCH_LOW_DEG  = -20.0f;
static constexpr float PITCH_HYST_DEG =   3.0f;

// Backlight auto-off: if neither pitch nor yaw changes by more than the
// active motion threshold for BACKLIGHT_TIMEOUT_MS, drive PIN_BACKLIGHT
// LOW. Subsequent motion turns it back on.
//
// Two thresholds, asymmetric on purpose:
//   - STAY_AWAKE_RAD (~2°): used while the backlight is already on, so
//     small deliberate adjustments keep the timer alive.
//   - WAKE_RAD (~10°): used while the backlight is off, so ambient table
//     vibration / handling bumps don't spuriously wake the device — only
//     a real pickup motion crosses this.
static constexpr float    STAY_AWAKE_RAD       =  2.0f * (PI / 180.0f);
static constexpr float    WAKE_RAD             = 10.0f * (PI / 180.0f);
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

static bool hasMjpExtension(const char *name) {
  size_t n = strlen(name);
  if (n < 5) return false;  // need at least "x.mjp"
  const char *e = name + n - 4;
  return e[0] == '.'
      && (e[1] == 'm' || e[1] == 'M')
      && (e[2] == 'j' || e[2] == 'J')
      && (e[3] == 'p' || e[3] == 'P');
}

// Scan SD root for .mjp files, sort alphabetically, copy up to
// MAX_VIDEO_SLOTS into videoPaths[]. Empty slots fall back to slot 0.
// Must be called after initSDCard() succeeds.
static void discoverVideos() {
  static constexpr int MAX_CANDIDATES = 16;
  char candidates[MAX_CANDIDATES][MAX_PATH_LEN];
  int n = 0;

  File root = SD.open("/");
  if (!root) {
    Serial.println("discoverVideos: SD root open failed");
    return;
  }
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    if (f.isDirectory()) { f.close(); continue; }
    const char *name = f.name();
    if (name[0] == '/') name++;            // some SD lib versions prefix "/"
    if (name[0] == '.') { f.close(); continue; }  // skip hidden / "._" files
    if (!hasMjpExtension(name)) { f.close(); continue; }
    if (n < MAX_CANDIDATES) {
      snprintf(candidates[n], MAX_PATH_LEN, "/%s", name);
      n++;
    } else {
      Serial.printf("discoverVideos: ignoring extra .mjp '%s' (cap %d)\n",
                    name, MAX_CANDIDATES);
    }
    f.close();
  }
  root.close();

  // Insertion sort, case-insensitive — humans expect "1-foo.mjp" < "2-bar.mjp"
  // regardless of how the FAT layer happens to capitalize the names.
  for (int i = 1; i < n; i++) {
    char tmp[MAX_PATH_LEN];
    strncpy(tmp, candidates[i], MAX_PATH_LEN);
    int j = i - 1;
    while (j >= 0 && strcasecmp(candidates[j], tmp) > 0) {
      strncpy(candidates[j + 1], candidates[j], MAX_PATH_LEN);
      j--;
    }
    strncpy(candidates[j + 1], tmp, MAX_PATH_LEN);
  }

  videoCount = (n < MAX_VIDEO_SLOTS) ? n : MAX_VIDEO_SLOTS;
  for (int i = 0; i < MAX_VIDEO_SLOTS; i++) {
    int src = (i < videoCount) ? i : 0;  // missing slots reuse the first file
    if (videoCount > 0) {
      strncpy(videoPaths[i], candidates[src], MAX_PATH_LEN);
    } else {
      videoPaths[i][0] = '\0';
    }
  }

  Serial.printf("discoverVideos: %d .mjp file(s) found\n", n);
  for (int i = 0; i < videoCount; i++) {
    Serial.printf("  slot[%d] = %s\n", i, videoPaths[i]);
  }
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
  discoverVideos();

  if (!imu.begin(Wire, 0x4B)) {
    Serial.println("IMU init failed");
    showFatal("IMU init failed");
    while (true) delay(1000);
  }

  if (videoCount == 0) {
    Serial.println("No .mjp files found on SD root");
    showFatal("no .mjp on SD");
    while (true) delay(1000);
  }

  // Start with the middle/level video; pitch selector in loop() will
  // switch among the three as the device tilts.
  if (!videoPlayer.begin(videoPaths[0])) {
    Serial.printf("VideoPlayer: failed to open %s\n", videoPaths[0]);
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

    float threshold = blOn ? STAY_AWAKE_RAD : WAKE_RAD;
    if (fabsf(dPitch) > threshold || fabsf(dYaw) > threshold) {
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
      // When fewer than 3 .mjp files are on the card, missing slots
      // resolve to the same path as slot 0. Skip the (expensive)
      // switchTo in that case — it would re-open and re-index the
      // same file, causing a visible hitch.
      bool samePath = strcmp(videoPaths[target], videoPaths[currentVideo]) == 0;
      if (samePath || videoPlayer.switchTo(videoPaths[target])) {
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
