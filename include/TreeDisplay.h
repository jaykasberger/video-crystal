#pragma once

#include "LGFX_CrowPanel24.h"

// Generative tree rendered as a function of an `age` parameter in [0,1]
// and a `shape` parameter in [-1,+1].
//
// `age` drives temporal growth: a full mature skeleton is generated once
// with a seeded RNG — a flat array of branches, each carrying its
// start/end pixel pair, depth, parent index, and a (startAge, growDur)
// window on the [0,1] timeline. render() draws each branch only as far
// as the current age has extended through that branch's window, so the
// same (seed, shape, age) triple always produces the same picture
// (deterministic) and increasing age looks like monotonic growth, not a
// random morph. Leaves bloom on terminal twigs once their branch matures.
// On top of the per-branch timeline, render() also scales the whole
// figure uniformly toward the trunk base by `age` so the tree reads as a
// smaller version of itself at younger ages instead of a full-size
// partial skeleton; thicknesses and leaf radii scale with it (1 px min).
//
// `shape` morphs the skeleton's silhouette by biasing the generator's
// parameter ranges (trunk length, branching angles, length taper, upward
// pull):
//   -1 → bushy / willow: short trunk, wide branches, slow taper, droop.
//    0 → balanced (the original look).
//   +1 → tall pole / conifer: long trunk, tight branches, fast taper,
//        strong upward pull.
// Changing shape regenerates the skeleton in place using the same seed,
// so the topology stays roughly similar (same branch ordering, similar
// branching factor) but the silhouette morphs smoothly. The current
// `age` is preserved through the shape change.
//
// Memory: ~5 KB for the skeleton (256 branches × 20 B). The off-screen
// canvas is a 320×240 RGB565 sprite (~150 KB) allocated from internal
// DMA-capable SRAM so pushSprite() can DMA cleanly to the panel.
class TreeDisplay {
 public:
  explicit TreeDisplay(LGFX &gfx);

  // Allocate the off-screen canvas and generate a deterministic tree
  // skeleton with the given seed. Returns false on allocation failure.
  bool begin(uint32_t seed = 0xC0FFEEu);

  // Re-generate the skeleton in place with a new seed. Doesn't touch the
  // canvas allocation — useful for a "give me a different tree" trigger.
  void reseed(uint32_t seed);

  // Set the current tree age. Values outside [0,1] are clamped. age 0 =
  // bare ground; age 1 = fully mature tree with leaves at every twig.
  void setAge(float age);

  // Set the silhouette shape. Values outside [-1,+1] are clamped. The
  // skeleton is regenerated in place when the change since the last
  // applied shape crosses ~0.05 (≈20 distinct silhouettes across the
  // full range, which is well below the visible threshold while keeping
  // generation cost off the hot loop). The current `age` is preserved.
  void setShape(float shape);

  // Set the scene's time-of-day in [0,1] (clamped). 0 = night (deep
  // navy sky, near-silhouette tree); 0.5 = mid-morning blue sky at full
  // daylight tree colors; 1 = very pale, bright high sky. Sky colors
  // interpolate through all three keypoints; tree/ground colors hold at
  // daylight values for d >= 0.5 so the tree doesn't bleach when aimed
  // further up. A repaint is forced when the change since the last
  // applied value crosses ~0.01 (≈100 distinct states across the range).
  void setDaylight(float daylight);

  // Redraw if `age` has changed enough since the last paint. Cheap no-op
  // when the on-screen image is already current. Call each loop().
  void update();

  float age()      const { return _age; }
  float shape()    const { return _shape; }
  float daylight() const { return _daylight; }

 private:
  static constexpr int MAX_BRANCHES = 256;

  struct Branch {
    int16_t x0, y0;       // start point (pixel coords)
    int16_t x1, y1;       // full-grown end point
    float   startAge;     // when this branch begins extending
    float   growDur;      // how long the extension takes
    int16_t parent;       // index into _branches, -1 for the trunk
    uint8_t depth;        // 0 = trunk
    uint8_t isTerminal;   // 1 if no children and depth >= 2 → grows a leaf
  };

  void generateSkeleton();
  void recomputeColors();
  void render();

  LGFX        &_gfx;
  LGFX_Sprite  _canvas;
  bool         _ready{false};

  Branch  _branches[MAX_BRANCHES];
  int     _branchCount{0};

  // Seed stashed so setShape() can regenerate at the current seed
  // without the caller having to remember it.
  uint32_t _seed{0xC0FFEEu};

  float   _age{0.0f};
  float   _drawnAge{-1.0f};

  // Sentinel start value > 1 forces the first setShape() call to
  // regenerate regardless of where the caller starts from.
  float   _shape{0.0f};
  float   _appliedShape{2.0f};

  // Default boot value is mid-morning so a device at rest on a level
  // surface shows the canonical blue sky. The sentinel forces a palette
  // recompute on the first setDaylight() regardless of caller value.
  float   _daylight{0.5f};
  float   _appliedDaylight{2.0f};

  uint16_t _colSky{0};
  uint16_t _colGround{0};
  uint16_t _colTrunk{0};
  uint16_t _colBranch{0};
  uint16_t _colTwig{0};
  uint16_t _colLeafA{0};
  uint16_t _colLeafB{0};
};
