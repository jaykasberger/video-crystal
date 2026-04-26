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

static constexpr const char *VIDEO_PATH = "/video.mjp";

// Roll angle (radians) that maps to first/last frame. Roll past the
// extents clamps. ±60° feels comfortable when holding the device.
static constexpr float SCRUB_ROLL_RANGE_RAD = 60.0f * (PI / 180.0f);

// The IMU is physically mounted rotated 90° clockwise in the roll axis,
// so the sensor reads ~+90° when the device itself is level. Subtract
// that constant so the *device's* roll, not the sensor's, drives the
// scrub. If scrubbing ends up reversed, flip the sign here.
static constexpr float IMU_ROLL_OFFSET_RAD = PI / 2.0f;

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

  if (!videoPlayer.begin(VIDEO_PATH)) {
    Serial.printf("VideoPlayer: failed to open %s\n", VIDEO_PATH);
    showFatal("video.mjp missing/invalid");
    while (true) delay(1000);
  }

  // Start in the middle so the first frame painted (before any IMU
  // reading arrives) matches the level-device position.
  videoPlayer.seekToFrame(videoPlayer.frameCount() / 2);
}

void loop() {
  imu.update();

  if (imu.hasFix() && videoPlayer.frameCount() > 0) {
    // Subtract the mounting offset so we work in device-frame roll, then
    // map [-RANGE, +RANGE] → [0, frameCount-1]. Level → middle frame.
    float roll = imu.roll() - IMU_ROLL_OFFSET_RAD;
    float t = (roll + SCRUB_ROLL_RANGE_RAD) / (2.0f * SCRUB_ROLL_RANGE_RAD);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    uint32_t target = (uint32_t)(t * (videoPlayer.frameCount() - 1));
    videoPlayer.seekToFrame(target);
  }

  videoPlayer.update();
}
