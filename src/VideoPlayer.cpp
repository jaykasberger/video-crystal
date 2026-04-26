#include "VideoPlayer.h"

#include <cstring>
#include <esp_heap_caps.h>

VideoPlayer::VideoPlayer(LGFX &gfx) : _gfx(gfx) {}

bool VideoPlayer::begin(const char *path) {
  _file = SD.open(path, FILE_READ);
  if (!_file) {
    Serial.printf("VideoPlayer: open '%s' failed\n", path);
    return false;
  }

  uint8_t header[16];
  if (_file.read(header, sizeof(header)) != (int)sizeof(header)) {
    Serial.println("VideoPlayer: header read failed");
    return false;
  }
  if (memcmp(header, "MJP1", 4) != 0) {
    Serial.printf("VideoPlayer: bad magic %02X%02X%02X%02X\n",
                  header[0], header[1], header[2], header[3]);
    return false;
  }
  _width  = uint16_t(header[4]) | (uint16_t(header[5]) << 8);
  _height = uint16_t(header[6]) | (uint16_t(header[7]) << 8);
  _fps    = uint16_t(header[8]) | (uint16_t(header[9]) << 8);
  _frameCount = uint32_t(header[12]) | (uint32_t(header[13]) << 8) |
                (uint32_t(header[14]) << 16) | (uint32_t(header[15]) << 24);
  if (_frameCount == 0) {
    Serial.println("VideoPlayer: no frames in header");
    return false;
  }

  // Index lives in PSRAM — small (8 B/frame) and not DMA'd.
  size_t indexBytes = size_t(_frameCount) * sizeof(FrameLoc);
  _index = (FrameLoc *)ps_malloc(indexBytes);
  if (!_index) {
    Serial.printf("VideoPlayer: index alloc failed (%u bytes)\n",
                  (unsigned)indexBytes);
    return false;
  }
  if (_file.read((uint8_t *)_index, indexBytes) != (int)indexBytes) {
    Serial.println("VideoPlayer: index read failed");
    return false;
  }

  // FrameLoc is two LE uint32s; ESP32 is little-endian, so the on-disk
  // bytes match in-memory layout — no swapping needed.

  // Find max compressed frame size for the JPEG read buffer.
  uint32_t maxSize = 0;
  for (uint32_t i = 0; i < _frameCount; i++) {
    if (_index[i].size > maxSize) maxSize = _index[i].size;
  }
  _jpegBufSize = maxSize;
  _jpegBuf = (uint8_t *)ps_malloc(_jpegBufSize);
  if (!_jpegBuf) {
    Serial.printf("VideoPlayer: jpeg buf alloc failed (%u bytes)\n",
                  _jpegBufSize);
    return false;
  }

  // Decoded pixel buffer must be DMA-capable internal SRAM (PSRAM-as-DMA
  // source on ESP32-S3 has cache-coherency issues — see CLAUDE.md
  // "Board gotchas"). Single-buffered: see VideoPlayer.h for tradeoffs.
  uint32_t pixelBytes = uint32_t(_width) * _height * 2;
  const uint32_t caps = MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL;
  _pixelBuf = (uint16_t *)heap_caps_malloc(pixelBytes, caps);
  if (!_pixelBuf) {
    Serial.printf("VideoPlayer: SRAM alloc failed (%u bytes, free %u)\n",
                  pixelBytes, (unsigned)heap_caps_get_free_size(caps));
    return false;
  }

  Serial.printf("VideoPlayer: %ux%u %u fps, %u frames, max JPEG %u B\n",
                _width, _height, _fps, _frameCount, maxSize);

  _requestedIdx = 0;
  _displayedIdx = UINT32_MAX;
  return true;
}

void VideoPlayer::seekToFrame(uint32_t idx) {
  if (_frameCount == 0) return;
  if (idx >= _frameCount) idx = _frameCount - 1;
  _requestedIdx = idx;
}

void VideoPlayer::update() {
  if (!_pixelBuf) return;
  if (_requestedIdx == _displayedIdx) return;

  uint32_t idx = _requestedIdx;  // snapshot — caller may change it again
  const FrameLoc &loc = _index[idx];

  // Wait for any in-flight DMA from the previous push to drain so we can
  // reuse _pixelBuf as the new decode target.
  _gfx.waitDMA();

  // Read this frame's JPEG bytes from SD.
  if (!_file.seek(loc.offset)) {
    Serial.printf("VideoPlayer: seek to %u failed (frame %u)\n",
                  loc.offset, idx);
    return;
  }
  if (_file.read(_jpegBuf, loc.size) != (int)loc.size) {
    Serial.printf("VideoPlayer: short JPEG read at frame %u\n", idx);
    return;
  }

  // Decode into _pixelBuf. The library calls handleBlock() for each MCU.
  if (!_jpeg.openRAM(_jpegBuf, (int)loc.size, drawCallback)) {
    Serial.printf("VideoPlayer: openRAM failed at frame %u\n", idx);
    return;
  }
  _jpeg.setUserPointer(this);
  _jpeg.setPixelType(RGB565_BIG_ENDIAN);  // matches lgfx::swap565_t on push
  if (!_jpeg.decode(0, 0, 0)) {
    Serial.printf("VideoPlayer: decode failed at frame %u\n", idx);
    _jpeg.close();
    return;
  }
  _jpeg.close();

  // Push the decoded pixels to the panel via DMA. Returns immediately;
  // the next update() will waitDMA before reusing _pixelBuf.
  _gfx.startWrite();
  _gfx.pushImageDMA(0, 0, _width, _height, (lgfx::swap565_t *)_pixelBuf);
  _gfx.endWrite();

  _displayedIdx = idx;
}

int VideoPlayer::drawCallback(JPEGDRAW *pDraw) {
  return ((VideoPlayer *)pDraw->pUser)->handleBlock(pDraw);
}

int VideoPlayer::handleBlock(JPEGDRAW *pDraw) {
  // Bounds-check against the destination buffer in case a malformed JPEG
  // reports dimensions that don't match the player's configured size.
  if (pDraw->x < 0 || pDraw->y < 0) return 0;
  if (pDraw->x + pDraw->iWidth  > (int)_width)  return 0;
  if (pDraw->y + pDraw->iHeight > (int)_height) return 0;

  const uint16_t *src = pDraw->pPixels;
  uint16_t *dst = _pixelBuf + pDraw->y * _width + pDraw->x;
  for (int row = 0; row < pDraw->iHeight; row++) {
    memcpy(dst, src, pDraw->iWidth * sizeof(uint16_t));
    src += pDraw->iWidth;
    dst += _width;
  }
  return 1;  // continue
}
