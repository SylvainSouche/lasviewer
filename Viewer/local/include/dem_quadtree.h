// dem_quadtree.h — the CPU side of the DEM mesh's level of detail: which
// quadtree cells (patches) a DEM is cut into, and how each patch edge must be
// tessellated so that neighbours of different sizes meet without cracks.
//
// Cells are addressed on a grid: a level-L cell has integer coordinates
// (ix, iy) in [0, coarse·2^L) and covers raster pixels
//   col ∈ [ix·cw, (ix+1)·cw],  row ∈ [iy·ch, (iy+1)·ch],  cw = (w-1)/(coarse·2^L)
// (the cell corners sit on the raster's pixel-centre lattice; corner (w-1)
// is the last pixel centre).
#pragma once
#include <atomic>
#include <cstdint>
#include <unordered_set>
#include <vector>

struct QuadCell {
    int level = 0;
    int ix = 0, iy = 0;
    double col = 0, row = 0, cw = 0, ch = 0; // raster-pixel footprint
};

// A DEM as the quadtree sees it: elevations with nodata already filled
// (fillNodataNearest), the nodata mask, and size.
struct DemGrid {
    const float* elev = nullptr;
    const uint8_t* nodata = nullptr; // 1 = nodata
    int w = 0, h = 0;

    bool isNodata(int c, int r) const { return nodata[static_cast<size_t>(r) * w + c] != 0; }
    // Bilinear elevation at a fractional pixel position, clamped to the
    // raster (filled values included).
    float sample(double col, double row) const;
};

// Largest |DEM − bilinear patch| over every valid pixel centre inside the
// cell, the patch being the bilinear surface through the cell's 4 corner
// samples. This is exactly how far the patch drawn with no tessellation
// (far away) can be from the DEM. O(pixels in the cell).
float cellDeviation(const DemGrid& g, const QuadCell& cell);

// Number of nodata samples among the cell's 4 corners and centre (0..5).
int cellNodataSamples(const DemGrid& g, const QuadCell& cell);

// The leaves of a quadtree, indexed by (level, ix, iy) for O(maxLevel)
// point lookups (the previous linear scan made balancing and edge
// classification quadratic in the leaf count).
class LeafIndex {
  public:
    LeafIndex(const std::vector<QuadCell>& leaves, int coarse, int maxLevel);
    // Level of the leaf covering finest-level cell (fx, fy), or -1 if that
    // cell is outside the grid or has no leaf (dropped as nodata).
    int levelAt(int fx, int fy) const;
    int finestCount() const { return finest_; }

  private:
    static uint64_t key(int level, int ix, int iy) {
        return (static_cast<uint64_t>(level) << 56) | (static_cast<uint64_t>(ix) << 28) |
               static_cast<uint64_t>(iy);
    }
    std::unordered_set<uint64_t> cells_;
    int maxLevel_, finest_;
};

// Subdivides leaves until every leaf's neighbours, all along each edge, are
// at most one level finer. Returns the number of passes.
// cw0/ch0: the level-0 cell size in pixels.
int balanceLeaves(std::vector<QuadCell>& leaves, int coarse, int maxLevel, double cw0, double ch0);

// How each edge of a leaf must be tessellated, in the order
// row− (corners 0→1), col+ (1→2), row+ (2→3), col− (3→0):
//   0  free: same-level neighbour, or none (raster or nodata boundary);
//   1  this leaf is the finer side of a one-level transition;
//   2  this leaf is the coarser side (its neighbours are two half-size cells).
// The tessellation shaders give a coarse-side edge exactly twice the segments
// of each fine-side half edge, so every vertex on one side coincides with
// one on the other.
void edgeCodes(const QuadCell& c, const LeafIndex& index, int maxLevel, float out[4]);

// Replaces every nodata value by the value of the nearest valid pixel
// (breadth-first from all valid pixels, 4-neighbour distance), in place.
// The heightmap is filled this way so that vertices tessellated over nodata
// stay at the height of the data next to them instead of dropping to 0;
// the fragment shader then cuts the surface at the data's edge. A raster
// with no valid pixel is left unchanged.
void fillNodataNearest(std::vector<float>& elev, const std::vector<uint8_t>& nodata, int w, int h);

// Builds the leaves top-down from a coarse×coarse grid. A cell stops when
// cellDeviation() is within the angular tolerance tan(angle)·cellSize/2
// (cellSize in world units; pixelW/pixelH are a pixel's world size) and it
// spans at most maxSpanPx pixels (so GPU tessellation, capped at 64 segments
// per edge, can still reach every DEM pixel), or at maxLevel. Entirely
// nodata cells are dropped.
std::vector<QuadCell> buildLeaves(const DemGrid& g, int coarse, int maxLevel, double angleDeg,
                                  double pixelW, double pixelH, double maxSpanPx,
                                  const std::atomic<bool>* cancel = nullptr);
