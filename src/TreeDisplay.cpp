#include "TreeDisplay.h"

#include <math.h>

TreeDisplay::TreeDisplay(LGFX &gfx) : _gfx(gfx), _canvas(&gfx) {}

bool TreeDisplay::begin(uint32_t seed) {
  // Off-screen canvas. setPsram(false) keeps the buffer in internal SRAM,
  // which is DMA-capable — pushSprite() will then ship it to the panel via
  // SPI DMA without the cache-coherency artifacts that PSRAM streaming
  // sources can hit on this board (see CLAUDE.md "PSRAM not reliable as a
  // DMA source"). 320×240×2 = 150 kB, which fits in internal SRAM with
  // room to spare since this mode doesn't also hold VideoPlayer's buffers.
  _canvas.setColorDepth(16);
  _canvas.setPsram(false);
  if (!_canvas.createSprite(_gfx.width(), _gfx.height())) {
    return false;
  }

  _seed            = seed ? seed : 0xC0FFEEu;
  _shape           = 0.0f;
  _appliedShape    = _shape;
  _daylight        = 0.5f;        // boot = mid-morning blue
  _appliedDaylight = _daylight;
  recomputeColors();
  generateSkeleton();
  _ready = true;
  _drawnAge = -1.0f;  // force a paint on the first update()
  return true;
}

void TreeDisplay::reseed(uint32_t seed) {
  _seed = seed ? seed : 0xC0FFEEu;
  generateSkeleton();
  _drawnAge = -1.0f;
}

void TreeDisplay::setAge(float age) {
  if (age < 0.0f) age = 0.0f;
  else if (age > 1.0f) age = 1.0f;
  _age = age;
}

void TreeDisplay::setShape(float shape) {
  if (shape < -1.0f) shape = -1.0f;
  else if (shape >  1.0f) shape =  1.0f;
  _shape = shape;
  // Regenerate only when the shape has visibly moved. ~0.05 is well below
  // the smallest silhouette change a viewer can spot at 320×240, while
  // keeping the regen cost (~1 ms) off most loop iterations.
  if (_ready && fabsf(_shape - _appliedShape) >= 0.05f) {
    generateSkeleton();
    _appliedShape = _shape;
    _drawnAge = -1.0f;  // force update() to repaint with the new skeleton
  }
}

void TreeDisplay::setDaylight(float daylight) {
  if (daylight < 0.0f) daylight = 0.0f;
  else if (daylight > 1.0f) daylight = 1.0f;
  _daylight = daylight;
  if (_ready && fabsf(_daylight - _appliedDaylight) >= 0.01f) {
    recomputeColors();
    _appliedDaylight = _daylight;
    _drawnAge = -1.0f;  // force update() to repaint with the new palette
  }
}

void TreeDisplay::update() {
  if (!_ready) return;
  // Skip the redraw if age hasn't moved enough to change a pixel anywhere
  // on screen. Typical branch length is ~30 px, so an age delta of 0.002
  // moves the longest tip by well under one pixel — safe to coalesce.
  if (fabsf(_age - _drawnAge) < 0.002f) return;
  render();
  _drawnAge = _age;
}

void TreeDisplay::recomputeColors() {
  const float d = _daylight;

  // Sky: three keypoints, lerped piecewise.
  //   d=0   night          (  5,  10,  30)  deep navy
  //   d=0.5 horizon blue   ( 80, 150, 215)  saturated mid-morning sky
  //   d=1   bright high    (200, 230, 250)  near-white pale blue
  uint8_t sr, sg, sb;
  if (d < 0.5f) {
    const float t = d * 2.0f;
    sr = (uint8_t)(  5 + ( 80 -   5) * t);
    sg = (uint8_t)( 10 + (150 -  10) * t);
    sb = (uint8_t)( 30 + (215 -  30) * t);
  } else {
    const float t = (d - 0.5f) * 2.0f;
    sr = (uint8_t)( 80 + (200 -  80) * t);
    sg = (uint8_t)(150 + (230 - 150) * t);
    sb = (uint8_t)(215 + (250 - 215) * t);
  }
  _colSky = _canvas.color565(sr, sg, sb);

  // Tree & ground colors blend from cool-silhouette (d=0) up to the
  // original daylight palette at d=0.5, then hold — tilting further up
  // should brighten the sky, not bleach the foliage.
  const float lit = (d < 0.5f) ? (d * 2.0f) : 1.0f;
  auto blend = [&](int nr, int ng, int nb, int dr, int dg, int db) -> uint16_t {
    const uint8_t r = (uint8_t)(nr + (dr - nr) * lit);
    const uint8_t g = (uint8_t)(ng + (dg - ng) * lit);
    const uint8_t b = (uint8_t)(nb + (db - nb) * lit);
    return _canvas.color565(r, g, b);
  };

  _colGround = blend( 12,  12,  20,  60,  46,  32);
  _colTrunk  = blend( 15,  15,  22,  76,  54,  35);
  _colBranch = blend( 22,  22,  30, 103,  77,  50);
  _colTwig   = blend( 35,  35,  45, 138, 110,  78);
  _colLeafA  = blend( 15,  28,  20,  65, 135,  55);
  _colLeafB  = blend( 25,  38,  28, 150, 200,  85);
}

void TreeDisplay::generateSkeleton() {
  // Local xorshift32 — small, fast, and seedable so the same `_seed`
  // always yields the same tree. Avoids touching the global Arduino
  // random() state.
  uint32_t rng = _seed ? _seed : 0xC0FFEEu;
  auto next = [&]() -> uint32_t {
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return rng;
  };
  auto frand = [&](float lo, float hi) -> float {
    float u = (float)(next() & 0xFFFFFFu) * (1.0f / (float)0x1000000);
    return lo + u * (hi - lo);
  };

  _branchCount = 0;

  // Shape biases. `s` in [-1,+1] linearly shifts each parameter range
  // between two extremes. The midpoint of every range here is the same
  // value that was previously hard-coded, so _shape==0 reproduces the
  // original silhouette exactly.
  //   trunkLenFrac: 0.45 (tall)  ↔  0.30 (default)  ↔  0.15 (short)
  //   spread:       (0.20,0.50)  ↔  (0.35,0.80)     ↔  (0.50,1.10)
  //   lenScale:     (0.52,0.72)  ↔  (0.62,0.82)     ↔  (0.72,0.92)
  //   upBias:       0.13         ↔  0.05            ↔  -0.03 (slight droop)
  const float s            = _shape;
  const float trunkLenFrac = 0.30f + 0.15f * s;
  const float spreadLo     = 0.35f - 0.15f * s;
  const float spreadHi     = 0.80f - 0.30f * s;
  const float lenScaleLo   = 0.62f - 0.10f * s;
  const float lenScaleHi   = 0.82f - 0.10f * s;
  const float upBias       = 0.05f + 0.08f * s;

  const int   W        = _canvas.width();
  const int   H        = _canvas.height();
  const float cx       = W * 0.5f;
  const float baseY    = H - 14;             // top of the ground strip
  const float trunkLen = H * trunkLenFrac;
  const float initAng  = -(float)M_PI * 0.5f + frand(-0.06f, 0.06f);

  // Trunk: occupies the full [0,1] timeline initially — children will sub-
  // divide their own slices off the back end of it via startFrac, and the
  // normalization pass at the bottom rescales everything to land at 1.0.
  Branch &root = _branches[_branchCount++];
  root.x0 = (int16_t)cx;
  root.y0 = (int16_t)baseY;
  root.x1 = (int16_t)(cx + cosf(initAng) * trunkLen);
  root.y1 = (int16_t)(baseY + sinf(initAng) * trunkLen);
  root.startAge   = 0.0f;
  root.growDur    = 1.0f;
  root.parent     = -1;
  root.depth      = 0;
  root.isTerminal = 0;

  // BFS expansion. Each iteration pops the next un-expanded branch and
  // emits its children at the tip; emitted children join the back of the
  // queue. We stop expanding when depth/length hit caps or the array fills.
  int head = 0;
  while (head < _branchCount && _branchCount < MAX_BRANCHES) {
    const int idx = head++;
    const Branch p = _branches[idx];

    if (p.depth >= 6) continue;

    const float pdx  = (float)(p.x1 - p.x0);
    const float pdy  = (float)(p.y1 - p.y0);
    const float plen = sqrtf(pdx * pdx + pdy * pdy);
    if (plen < 7.0f) continue;            // too short to split usefully

    const float pang = atan2f(pdy, pdx);

    // Trunk fans into 3-4 main limbs; everything else into 2-4 sub-branches.
    int n = (p.depth == 0) ? (3 + (int)(next() & 1u))
                           : (2 + (int)(next() % 3u));

    for (int k = 0; k < n; k++) {
      if (_branchCount >= MAX_BRANCHES) break;

      // Spread the n children evenly across an angular fan around the
      // parent direction, with a small jitter per child. spread is the
      // half-fan width in radians; range varies with _shape (tight for
      // tall trees, wide for bushy ones).
      const float spread = frand(spreadLo, spreadHi);
      const float bias   = (n == 1) ? 0.0f
                                    : ((float)k / (float)(n - 1)) * 2.0f - 1.0f;
      float childAng = pang + bias * spread + frand(-0.10f, 0.10f);
      // Pull toward straight-up. upBias also varies with _shape: strong
      // for tall sky-reaching trees, slightly negative for bushy/willow
      // shapes (which then allow a mild droop in the canopy).
      const float upward = -(float)M_PI * 0.5f;
      childAng += (upward - childAng) * upBias;

      const float lenScale = frand(lenScaleLo, lenScaleHi);
      const float clen     = plen * lenScale;

      // Children begin extending before the parent fully finishes — gives
      // an organic look where the canopy keeps unfurling as the trunk grows.
      const float startFrac  = frand(0.55f, 0.95f);
      const float childStart = p.startAge + p.growDur * startFrac;
      const float childDur   = p.growDur * frand(0.55f, 0.85f);

      Branch &c = _branches[_branchCount++];
      c.x0 = p.x1;
      c.y0 = p.y1;
      c.x1 = (int16_t)(p.x1 + cosf(childAng) * clen);
      c.y1 = (int16_t)(p.y1 + sinf(childAng) * clen);
      c.startAge   = childStart;
      c.growDur    = childDur;
      c.parent     = (int16_t)idx;
      c.depth      = (uint8_t)(p.depth + 1);
      c.isTerminal = 0;
    }
  }

  // Mark terminal branches — those with no children and at least depth 2,
  // so leaves only sit on twigs, not on the trunk or main limbs.
  bool hasChild[MAX_BRANCHES] = {};
  for (int i = 0; i < _branchCount; i++) {
    if (_branches[i].parent >= 0) hasChild[_branches[i].parent] = true;
  }
  for (int i = 0; i < _branchCount; i++) {
    if (!hasChild[i] && _branches[i].depth >= 2) {
      _branches[i].isTerminal = 1;
    }
  }

  // Normalize the timeline so the latest-finishing branch ends at age 1.0.
  // Without this, the user-facing `age` would have to cap somewhere > 1
  // depending on how deep the deepest twig randomly landed.
  float maxEnd = 0.0f;
  for (int i = 0; i < _branchCount; i++) {
    float e = _branches[i].startAge + _branches[i].growDur;
    if (e > maxEnd) maxEnd = e;
  }
  if (maxEnd > 0.0f) {
    for (int i = 0; i < _branchCount; i++) {
      _branches[i].startAge /= maxEnd;
      _branches[i].growDur  /= maxEnd;
    }
  }
}

void TreeDisplay::render() {
  _canvas.fillScreen(_colSky);
  _canvas.fillRect(0, _canvas.height() - 14, _canvas.width(), 14, _colGround);

  // Overall figure scale grows linearly with age, applied around the trunk
  // base. The fractal branch-addition timeline below is unchanged: a branch
  // only starts drawing once `_age` enters its window. On top of that, all
  // geometry is scaled toward the base so an early-age tree reads as a
  // smaller version of the same silhouette rather than a full-size partial
  // skeleton. Thicknesses and leaf radii scale with it (floored at 1 px) so
  // a sapling doesn't carry the mature tree's chunky trunk.
  const float scale = _age;
  const float baseX = _canvas.width()  * 0.5f;
  const float baseY = (float)(_canvas.height() - 14);
  auto sx = [&](float x) -> float { return baseX + (x - baseX) * scale; };
  auto sy = [&](float y) -> float { return baseY + (y - baseY) * scale; };

  // Branches. We iterate the array in BFS order (the order generateSkeleton
  // produced), so parents draw before children and the joint at each fork
  // looks continuous rather than showing a seam from a later overdraw.
  for (int i = 0; i < _branchCount; i++) {
    const Branch &b = _branches[i];
    if (_age <= b.startAge) continue;

    float t = (_age - b.startAge) / b.growDur;
    if (t > 1.0f) t = 1.0f;

    const float exFull = (float)b.x0 + (float)(b.x1 - b.x0) * t;
    const float eyFull = (float)b.y0 + (float)(b.y1 - b.y0) * t;

    const int16_t x0s = (int16_t)sx((float)b.x0);
    const int16_t y0s = (int16_t)sy((float)b.y0);
    const int16_t exs = (int16_t)sx(exFull);
    const int16_t eys = (int16_t)sy(eyFull);

    const uint16_t color = (b.depth == 0) ? _colTrunk
                         : (b.depth <= 2) ? _colBranch
                                          : _colTwig;
    // Trunk fat, halves with depth, min 1 px. Depth 0 → 9 px, 1 → 4, 2 → 2…
    int thickFull = 9 >> b.depth;
    if (thickFull < 1) thickFull = 1;
    int thick = (int)((float)thickFull * scale);
    if (thick < 1) thick = 1;

    if (thick >= 2) {
      _canvas.drawWideLine(x0s, y0s, exs, eys, (float)thick * 0.5f, color);
    } else {
      _canvas.drawLine(x0s, y0s, exs, eys, color);
    }
  }

  // Leaves on terminals, rendered second so they sit on top of the bark.
  for (int i = 0; i < _branchCount; i++) {
    const Branch &b = _branches[i];
    if (!b.isTerminal) continue;
    const float branchEnd = b.startAge + b.growDur;
    if (_age <= branchEnd) continue;

    // Each leaf cluster has its own growth window from branch maturity to
    // global age 1. Late-maturing twigs (branchEnd close to 1) get a much
    // shorter leaf window, so they pop fast at the very end.
    const float remaining = 1.0f - branchEnd;
    float leafT = (remaining > 1e-3f) ? (_age - branchEnd) / remaining : 1.0f;
    if (leafT > 1.0f) leafT = 1.0f;

    const float leafRFull = 1.0f + leafT * 4.5f;
    int leafR = (int)(leafRFull * scale);
    if (leafR < 1) leafR = 1;

    const int16_t lx = (int16_t)sx((float)b.x1);
    const int16_t ly = (int16_t)sy((float)b.y1);

    // Two-tone canopy: alternate shade by branch index. Cheaper and just
    // as effective visually as picking the color from depth or position.
    const uint16_t leafColor = (i & 1) ? _colLeafA : _colLeafB;
    _canvas.fillCircle(lx, ly, leafR, leafColor);
  }

  _canvas.pushSprite(0, 0);
}
