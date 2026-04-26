#!/usr/bin/env bash
#
# encode_video.sh — convert a video to the indexed-MJPEG (.mjp) format
# played by VideoPlayer on the device.
#
# File format (matches include/VideoPlayer.h):
#   16-byte header: "MJP1" + width(LE16) + height(LE16) + fps(LE16) +
#                   reserved(LE16) + frame_count(LE32)
#   Index of 8 bytes per frame: offset(LE32) + size(LE32)
#   Followed by concatenated JPEG payloads.

set -euo pipefail

usage() {
  cat <<EOF
Usage: $0 <input-video> [output.mjp]

Defaults: 320x240 @ 30 fps, JPEG quality 5. Override with env vars:

  WIDTH=240 HEIGHT=180 FPS=24 $0 in.mp4 out.mjp
  QUALITY=3 $0 in.mp4 out.mjp           # higher quality, larger file
  ROTATE=cw $0 in.mp4 out.mjp           # 90° clockwise
  ROTATE=ccw $0 in.mp4 out.mjp          # 90° counter-clockwise
  ROTATE=180 $0 in.mp4 out.mjp          # flip

QUALITY: ffmpeg -q:v scale, 1 (best/largest) to 31 (worst/smallest). 5
is a good default. Smaller numbers => bigger files but better images.

ROTATE values: none (default) | cw | ccw | 180

The output file should be copied to the SD card at /video.mjp.
EOF
}

if [ $# -lt 1 ]; then usage; exit 1; fi

INPUT="$1"
OUTPUT="${2:-video.mjp}"
WIDTH="${WIDTH:-320}"
HEIGHT="${HEIGHT:-240}"
FPS="${FPS:-30}"
QUALITY="${QUALITY:-5}"
ROTATE="${ROTATE:-none}"

case "$ROTATE" in
  none|0)  ROTATE_VF="" ;;
  cw|1)    ROTATE_VF="transpose=1," ;;
  ccw|2)   ROTATE_VF="transpose=2," ;;
  180)     ROTATE_VF="transpose=1,transpose=1," ;;
  *) echo "error: unknown ROTATE='$ROTATE' (use none|cw|ccw|180)" >&2; exit 1 ;;
esac

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "error: ffmpeg not found (install with: brew install ffmpeg)" >&2
  exit 1
fi
if ! command -v python3 >/dev/null 2>&1; then
  echo "error: python3 not found" >&2
  exit 1
fi

TMPDIR=$(mktemp -d -t encode_video.XXXXXX)
cleanup() { rm -rf "$TMPDIR"; }
trap cleanup EXIT

echo "Extracting frames -> ${WIDTH}x${HEIGHT} JPEG q=${QUALITY} @ ${FPS}fps (rotate=${ROTATE})..."
ffmpeg -hide_banner -loglevel warning -y \
  -i "$INPUT" \
  -vf "${ROTATE_VF}scale=${WIDTH}:${HEIGHT}:force_original_aspect_ratio=decrease,pad=${WIDTH}:${HEIGHT}:(ow-iw)/2:(oh-ih)/2:black,fps=${FPS}" \
  -q:v "$QUALITY" \
  "$TMPDIR/%06d.jpg"

WIDTH="$WIDTH" HEIGHT="$HEIGHT" FPS="$FPS" \
TMPDIR="$TMPDIR" OUTPUT="$OUTPUT" \
python3 <<'PY'
import os, struct, glob

w  = int(os.environ['WIDTH'])
h  = int(os.environ['HEIGHT'])
fps = int(os.environ['FPS'])
tmpdir = os.environ['TMPDIR']
out = os.environ['OUTPUT']

frame_paths = sorted(glob.glob(os.path.join(tmpdir, '*.jpg')))
n = len(frame_paths)
if n == 0:
    raise SystemExit('error: ffmpeg produced no frames')

# Read all JPEGs into memory. For typical clips this is comfortably
# under a few hundred MB; if it ever isn't, switch to two-pass.
data = []
total = 0
max_size = 0
for p in frame_paths:
    with open(p, 'rb') as fp:
        d = fp.read()
    data.append(d)
    total += len(d)
    if len(d) > max_size:
        max_size = len(d)

avg = total // n if n else 0
print(f'Frames: {n}, total JPEG: {total} bytes, avg {avg} per frame, '
      f'max {max_size}')

with open(out, 'wb') as o:
    # 16-byte header.
    o.write(b'MJP1')
    o.write(struct.pack('<HHHHI', w, h, fps, 0, n))

    # Per-frame index: offset + size, both uint32 LE.
    payload_offset = 16 + 8 * n
    cursor = payload_offset
    for jpg in data:
        o.write(struct.pack('<II', cursor, len(jpg)))
        cursor += len(jpg)

    # Payloads.
    for jpg in data:
        o.write(jpg)

print(f'Wrote {out} ({os.path.getsize(out)} bytes)')
PY

echo 'Copy to SD card as /video.mjp'
