#pragma once

#include <SD.h>
#include <JPEGDEC.h>
#include "LGFX_CrowPanel24.h"

// Random-access MJPEG player. Each frame in the file is an independent
// JPEG; an index at the head of the file lets us jump to any frame in
// O(1). Designed for IMU-driven scrubbing where the requested frame can
// jump around between calls.
//
// File format (matches scripts/encode_video.sh):
//   offset  size       field
//   0       4          magic "MJP1"
//   4       2          width        (uint16 LE)
//   6       2          height       (uint16 LE)
//   8       2          fps          (uint16 LE; informational, not used for timing)
//   10      2          reserved
//   12      4          frame_count  (uint32 LE)
//   16      8 × N      index: { uint32 offset, uint32 size }
//   16+8N   ..         concatenated JPEG payloads
class VideoPlayer {
 public:
  explicit VideoPlayer(LGFX &gfx);

  // Open `path`, read header + index into RAM, allocate buffers.
  bool begin(const char *path);

  // Switch the active video file. Closes the current one, opens `path`,
  // reads its header + index, and reuses (or grows) the existing buffers.
  // All videos must share dimensions (width × height); if `path` reports
  // different ones, this returns false and the previous video remains
  // playable. Resets the requested/displayed frame indices.
  bool switchTo(const char *path);

  // Request that frame `idx` be displayed. Clamped to [0, frameCount-1].
  // No-op if it's the same frame already showing. The actual SD read +
  // JPEG decode + DMA push happens in update(), so callers can update
  // the request as often as they like without blocking.
  void seekToFrame(uint32_t idx);

  // If the requested frame differs from what's on screen, fetch + decode
  // + push it. Otherwise no-op. Call each loop().
  void update();

  uint32_t frameCount() const { return _frameCount; }
  uint16_t width()      const { return _width; }
  uint16_t height()     const { return _height; }
  uint16_t fps()        const { return _fps; }

 private:
  static int drawCallback(JPEGDRAW *pDraw);
  int handleBlock(JPEGDRAW *pDraw);

  struct FrameLoc {
    uint32_t offset;
    uint32_t size;
  };

  LGFX &_gfx;
  File _file;
  uint16_t _width{0};
  uint16_t _height{0};
  uint16_t _fps{0};
  uint32_t _frameCount{0};

  // Index of (offset, size) per frame. Lives in PSRAM.
  FrameLoc *_index{nullptr};

  // JPEG read buffer (sized to the largest frame's compressed size). PSRAM.
  uint8_t *_jpegBuf{nullptr};
  uint32_t _jpegBufSize{0};

  // Single decoded RGB565 buffer in internal DMA-capable SRAM. We can't
  // double-buffer here: 320x240x2 = 150 kB and the runtime heap leaves
  // less than ~200 kB internal SRAM after LGFX/Adafruit/JPEGDEC claims.
  // Tradeoff: each update() does waitDMA -> decode -> push, no overlap.
  uint16_t *_pixelBuf{nullptr};

  uint32_t _requestedIdx{0};
  uint32_t _displayedIdx{UINT32_MAX};

  JPEGDEC _jpeg;
};
