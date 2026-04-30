# video-crystal

Firmware for the **Elecrow CrowPanel Advance 2.4" HMI ESP32 AI Display**
(ESP32-S3-WROOM-1-N16R8, 16MB flash, 8MB PSRAM; 320×240 ST7789 panel with
FT5x06 capacitive touch). PlatformIO + Arduino framework, LovyanGFX for
rendering.

## Board gotchas

### LovyanGFX's touch driver silently clobbers `Wire`

**Root cause**: on this board the external I²C socket is wired to the same
pins (GPIO 15 SDA, GPIO 16 SCL) as the touch controller. If LovyanGFX is
configured with its `Touch_FT5x06` driver, `gfx.init()` claims I²C port 0
on those pins at 400 kHz, which is the same hardware peripheral the
Arduino `Wire` library uses by default. Any `Wire`-based peripheral
(IMU, RTC, etc.) plugged into the external I²C socket will be silently
unreachable — even an I²C scanner returns `(none)`, because LGFX has
reconfigured the peripheral out from under `Wire`.

**Current workaround**: the touch driver is intentionally removed from
[include/LGFX_CrowPanel24.h](include/LGFX_CrowPanel24.h). `Wire` owns the
I²C bus; the touch chip still sits on the bus (visible as `0x38` on a
scan) but is not polled. If touch input is ever wired back in, a
different sharing strategy is needed — e.g. route touch reads through
LGFX's bus API, or manage the touch chip directly via `Wire`.

### PSRAM is not reliable as a DMA source for sustained streaming

`heap_caps_malloc(MALLOC_CAP_SPIRAM)` / `ps_malloc` returns PSRAM, which
the SPI DMA engine *can* technically read from on this board's octal
PSRAM config — but cache coherency between the CPU and the DMA engine
isn't reliably maintained when you push frames back-to-back at video
rates. Symptom: video plays cleanly for a few loops, then the image
walks across the screen with horizontal+vertical wrap. There are no
errors on the serial console — the DMA just reads stale cache lines.

**Workaround**: allocate streaming buffers in internal SRAM with
`heap_caps_malloc(size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)`. Used
in [src/VideoPlayer.cpp](src/VideoPlayer.cpp) for the two ~150 kB
frame buffers. The ESP32-S3 has 512 kB internal SRAM, so two
full-screen RGB565 buffers fit comfortably; if you want more or larger
buffers, you'll need to either flush cache lines manually before each
DMA or drop back to single-buffered non-DMA `pushImage`.

### Touch pin constants in `main.cpp` are stale

[src/main.cpp:9-10](src/main.cpp#L9-L10) defines `PIN_TOUCH_RST = 1` and
`PIN_TOUCH_INT = 2`. Per the Elecrow schematic, touch INT is actually
**GPIO 47**. The Elecrow factory `powerOnTouchAndPanel()` sequence we
inherited toggles these constants but on wrong pins; it happens not to
matter because the display lights up regardless and we're not using
touch. Fix the constants if touch is re-enabled.

## Hardware pinout (from Elecrow wiki)

| Function | GPIO |
|---|---|
| I²C SDA (touch + external socket) | 15 |
| I²C SCL (touch + external socket) | 16 |
| Touch FT5x06 INT | 47 |
| Display ST7789 SCLK | 42 |
| Display ST7789 MOSI | 39 |
| Display ST7789 DC | 41 |
| Display ST7789 CS | 40 |
| Backlight | 38 |
| SD SCK | 5 |
| SD MOSI | 6 |
| SD MISO | 4 |
| SD CS | 7 |

Touch FT5x06 lives at I²C address `0x38`.

The SD slot is wired as SPI (not SDIO/SDMMC) on its own controller —
independent of the display's SPI2. In code, use `SPIClass(HSPI)` (which
maps to SPI3 on ESP32-S3 Arduino); LGFX owns `SPI2_HOST` / `FSPI`.

**Elecrow wiki pinout note**: the wiki page lists SD CS as "3.3V",
which is incorrect for the 2.4" model. The actual CS is **GPIO 7**
(verified against Elecrow's own demo source in the product GitHub repo).

## Upload / serial

USB1 = ESP32-S3 native USB (`/dev/cu.usbmodem*`). USB0 (CH340) does not
enumerate on this unit, so both upload and serial monitor are routed
through native USB (`ARDUINO_USB_MODE=1`, `ARDUINO_USB_CDC_ON_BOOT=1` in
[platformio.ini](platformio.ini)).

## External I²C modules attached to this project

- **GY-BNO085 IMU** (Adafruit BNO08x library). Strapped to address
  `0x4B` on this unit (the ADR pin is tied high); if a replacement
  module comes up `0x4A`, update the address passed to
  `ImuDisplay::begin()`. `begin_I2C()` expects `Wire` to already be
  running — `powerOnTouchAndPanel()` starts it before sensor init.

## Display modes

Each display mode is one class with `begin()` / `update()`. Main wires
exactly one mode at a time. To switch active modes, change the include
and the global declaration in [src/main.cpp](src/main.cpp).

- [ImuDisplay](include/ImuDisplay.h) — yaw/pitch/roll + magnetometer
  from the BNO085. Currently dormant; useful for IMU debugging.
- [VideoPlayer](include/VideoPlayer.h) — random-access MJPEG playback
  from SD, decoded with JPEGDEC. Currently active.

## Input

[ImuInput](include/ImuInput.h) is a minimal BNO085 wrapper that exposes
`yaw()/pitch()/roll()` in radians (ZYX Tait-Bryan). It's used in
[src/main.cpp](src/main.cpp) to map the device's roll angle linearly
to a video frame index — tilt the device to scrub through the clip.
Don't confuse with `ImuDisplay`, which does the sensor *and* renders
the orientation on screen; `ImuInput` is sensor-only.

## Video file format (.mjp)

Played by `VideoPlayer`. Indexed MJPEG container — each frame is an
independent JPEG; a per-frame index at the head of the file lets us
seek to any frame in O(1).

```
offset      size       field
0           4          magic "MJP1"
4           2          width        (uint16 LE)
6           2          height       (uint16 LE)
8           2          fps          (uint16 LE; informational)
10          2          reserved
12          4          frame_count  (uint32 LE)
16          8 × N      index: { uint32 offset, uint32 size }
16 + 8N     …          concatenated JPEG payloads
```

Encode with [scripts/encode_video.sh](scripts/encode_video.sh):

```
./scripts/encode_video.sh input.mp4 video.mjp
```

Then copy the encoded `.mjp` file(s) to the SD card root. Override
encoder defaults via env vars: `WIDTH=240 HEIGHT=180 FPS=24 QUALITY=3
ROTATE=cw ...`.

At boot, [src/main.cpp](src/main.cpp) scans the SD root for `*.mjp`,
sorts the matches alphabetically (case-insensitive), and assigns up to
three to pitch-band slots: index 0 = level/middle (and the file shown
at startup), index 1 = tilted up, index 2 = tilted down. Files past
the third are ignored; if fewer than three are present, the missing
slots fall back to slot 0 so the same video plays in those bands. So
to put different content on individual devices, just rename the files
with a sort prefix that puts the level/middle clip first
(e.g. `1-level.mjp`, `2-up.mjp`, `3-down.mjp`).

`QUALITY` is ffmpeg's `-q:v` (1 best/largest, 31 worst/smallest;
5 is a sane default). The script letterboxes the source to preserve
aspect ratio.

## Bandwidth notes (for tuning)

- SD over SPI at 25 MHz delivers ~3 MB/s real-world; first read after a
  seek is similar to sustained-sequential because each frame is small.
- Per-frame budget at 30 fps = 33 ms. Typical 320×240 JPEG at q=5:
  read ~10 ms, decode ~15-25 ms (software, accelerated by S3's SIMD
  via JPEGDEC), DMA push ~15 ms (overlapped with the next decode). Net
  per-frame ≈ 25-35 ms.
- For higher frame rates, drop resolution (`WIDTH=240 HEIGHT=180`) or
  raise `QUALITY` (smaller files, faster decode).
- The pixel buffers (`_decodeBuf`, `_displayBuf`) are in internal DMA
  SRAM; the index table and the JPEG-byte buffer are in PSRAM.
  See "PSRAM not reliable as a DMA source" gotcha above for why.
