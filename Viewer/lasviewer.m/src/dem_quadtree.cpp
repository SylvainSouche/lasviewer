// dem_quadtree.cpp — see dem_quadtree.h.
#include "dem_quadtree.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>

float DemGrid::sample(double col, double row) const {
    col = std::clamp(col, 0.0, static_cast<double>(w - 1));
    row = std::clamp(row, 0.0, static_cast<double>(h - 1));
    int c0 = static_cast<int>(col), r0 = static_cast<int>(row);
    int c1 = std::min(c0 + 1, w - 1), r1 = std::min(r0 + 1, h - 1);
    double fx = col - c0, fy = row - r0;
    auto at = [&](int c, int r) -> double { return elev[static_cast<size_t>(r) * w + c]; };
    return static_cast<float>(at(c0, r0) * (1 - fx) * (1 - fy) + at(c1, r0) * fx * (1 - fy) +
                              at(c0, r1) * (1 - fx) * fy + at(c1, r1) * fx * fy);
}

float cellDeviation(const DemGrid& g, const QuadCell& cell) {
    const double e00 = g.sample(cell.col, cell.row);
    const double e10 = g.sample(cell.col + cell.cw, cell.row);
    const double e01 = g.sample(cell.col, cell.row + cell.ch);
    const double e11 = g.sample(cell.col + cell.cw, cell.row + cell.ch);
    const int c0 = std::max(0, static_cast<int>(std::ceil(cell.col)));
    const int c1 = std::min(g.w - 1, static_cast<int>(std::floor(cell.col + cell.cw)));
    const int r0 = std::max(0, static_cast<int>(std::ceil(cell.row)));
    const int r1 = std::min(g.h - 1, static_cast<int>(std::floor(cell.row + cell.ch)));
    double dev = 0.0;
    for (int r = r0; r <= r1; ++r) {
        const double t = (r - cell.row) / cell.ch;
        const double left = e00 + (e01 - e00) * t;  // col side
        const double right = e10 + (e11 - e10) * t; // col + cw side
        const float* line = g.elev + static_cast<size_t>(r) * g.w;
        for (int c = c0; c <= c1; ++c) {
            if (g.isNodata(c, r)) continue;
            const double s = (c - cell.col) / cell.cw;
            dev = std::max(dev, std::abs(line[c] - (left + (right - left) * s)));
        }
    }
    return static_cast<float>(dev);
}

int cellNodataSamples(const DemGrid& g, const QuadCell& cell) {
    auto nd = [&](double col, double row) {
        int c = std::clamp(static_cast<int>(std::lround(col)), 0, g.w - 1);
        int r = std::clamp(static_cast<int>(std::lround(row)), 0, g.h - 1);
        return g.isNodata(c, r) ? 1 : 0;
    };
    return nd(cell.col, cell.row) + nd(cell.col + cell.cw, cell.row) +
           nd(cell.col, cell.row + cell.ch) + nd(cell.col + cell.cw, cell.row + cell.ch) +
           nd(cell.col + cell.cw * 0.5, cell.row + cell.ch * 0.5);
}

LeafIndex::LeafIndex(const std::vector<QuadCell>& leaves, int coarse, int maxLevel)
    : maxLevel_(maxLevel), finest_(coarse << maxLevel) {
    cells_.reserve(leaves.size() * 2);
    for (const QuadCell& c : leaves) cells_.insert(key(c.level, c.ix, c.iy));
}

int LeafIndex::levelAt(int fx, int fy) const {
    if (fx < 0 || fy < 0 || fx >= finest_ || fy >= finest_) return -1;
    for (int L = 0; L <= maxLevel_; ++L) {
        int shift = maxLevel_ - L;
        if (cells_.count(key(L, fx >> shift, fy >> shift))) return L;
    }
    return -1;
}

namespace {

QuadCell makeCell(int level, int ix, int iy, double cw0, double ch0) {
    QuadCell c;
    c.level = level;
    c.ix = ix;
    c.iy = iy;
    double n = static_cast<double>(1 << level);
    c.cw = cw0 / n;
    c.ch = ch0 / n;
    c.col = ix * c.cw;
    c.row = iy * c.ch;
    return c;
}

// Finest-level span of a level-L cell.
int span(int level, int maxLevel) {
    return 1 << (maxLevel - level);
}

// Deepest leaf level along one edge of `c` (side 0..3 as in edgeCodes), -1
// if no leaf borders it.
int maxLevelAlongEdge(const QuadCell& c, int side, const LeafIndex& index, int maxLevel) {
    const int s = span(c.level, maxLevel);
    const int x0 = c.ix * s, y0 = c.iy * s;
    int best = -1;
    for (int k = 0; k < s; ++k) {
        int fx, fy;
        switch (side) {
        case 0:
            fx = x0 + k;
            fy = y0 - 1;
            break; // row−
        case 1:
            fx = x0 + s;
            fy = y0 + k;
            break; // col+
        case 2:
            fx = x0 + k;
            fy = y0 + s;
            break; // row+
        default:
            fx = x0 - 1;
            fy = y0 + k;
            break; // col−
        }
        best = std::max(best, index.levelAt(fx, fy));
    }
    return best;
}

} // namespace

int balanceLeaves(std::vector<QuadCell>& leaves, int coarse, int maxLevel, double cw0, double ch0) {
    int passes = 0;
    for (bool changed = true; changed && passes < 2 * maxLevel + 2; ++passes) {
        changed = false;
        LeafIndex index(leaves, coarse, maxLevel);
        std::vector<QuadCell> next;
        next.reserve(leaves.size());
        for (const QuadCell& c : leaves) {
            bool split = false;
            for (int side = 0; side < 4 && !split; ++side)
                split = maxLevelAlongEdge(c, side, index, maxLevel) > c.level + 1;
            if (!split) {
                next.push_back(c);
                continue;
            }
            changed = true;
            for (int k = 0; k < 4; ++k)
                next.push_back(
                    makeCell(c.level + 1, 2 * c.ix + (k & 1), 2 * c.iy + (k >> 1), cw0, ch0));
        }
        leaves = std::move(next);
    }
    return passes;
}

void edgeCodes(const QuadCell& c, const LeafIndex& index, int maxLevel, float out[4]) {
    const int s = span(c.level, maxLevel);
    const int x0 = c.ix * s, y0 = c.iy * s, mid = s / 2;
    // Probe beside the edge's midpoint; after balancing, a neighbour region is
    // either one cell (same level or one coarser) or two cells one finer, so
    // one probe tells which.
    const int probes[4][2] = {
        {x0 + mid, y0 - 1}, {x0 + s, y0 + mid}, {x0 + mid, y0 + s}, {x0 - 1, y0 + mid}};
    for (int side = 0; side < 4; ++side) {
        int L = index.levelAt(probes[side][0], probes[side][1]);
        out[side] = (L < 0 || L == c.level) ? 0.0f : (L < c.level ? 1.0f : 2.0f);
    }
}

std::vector<QuadCell> buildLeaves(const DemGrid& g, int coarse, int maxLevel, double angleDeg,
                                  double pixelW, double pixelH, double maxSpanPx,
                                  const std::atomic<bool>* cancel) {
    const double cw0 = static_cast<double>(g.w - 1) / coarse;
    const double ch0 = static_cast<double>(g.h - 1) / coarse;
    const double tanA = std::tan(angleDeg * 3.14159265358979323846 / 180.0);

    std::function<void(const QuadCell&, std::vector<QuadCell>&)> build =
        [&](const QuadCell& cell, std::vector<QuadCell>& out) {
            if (cancel && cancel->load(std::memory_order_relaxed)) return;
            int nd = cellNodataSamples(g, cell);
            if (nd == 5) return; // entirely nodata
            if (nd == 0 && std::max(cell.cw, cell.ch) <= maxSpanPx) {
                double worldSize = std::min(cell.cw * pixelW, cell.ch * pixelH);
                if (cellDeviation(g, cell) <= tanA * worldSize * 0.5) {
                    out.push_back(cell);
                    return;
                }
            }
            if (cell.level >= maxLevel) {
                // Mixed nodata at the finest level: keep the cell if its centre
                // has data.
                if (nd > 0) {
                    int c = std::clamp(static_cast<int>(std::lround(cell.col + cell.cw * 0.5)), 0,
                                       g.w - 1);
                    int r = std::clamp(static_cast<int>(std::lround(cell.row + cell.ch * 0.5)), 0,
                                       g.h - 1);
                    if (g.isNodata(c, r)) return;
                }
                out.push_back(cell);
                return;
            }
            for (int k = 0; k < 4; ++k)
                build(makeCell(cell.level + 1, 2 * cell.ix + (k & 1), 2 * cell.iy + (k >> 1), cw0,
                               ch0),
                      out);
        };

    std::vector<std::vector<QuadCell>> perRoot(static_cast<size_t>(coarse) * coarse);
#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < coarse * coarse; ++i)
        build(makeCell(0, i % coarse, i / coarse, cw0, ch0), perRoot[i]);
    std::vector<QuadCell> leaves;
    for (auto& v : perRoot) leaves.insert(leaves.end(), v.begin(), v.end());
    return leaves;
}

void fillNodataNearest(std::vector<float>& elev, const std::vector<uint8_t>& nodata, int w, int h) {
    std::vector<uint8_t> done(nodata.size());
    std::deque<int> queue;
    for (size_t i = 0; i < nodata.size(); ++i) {
        done[i] = nodata[i] ? 0 : 1;
        if (done[i]) queue.push_back(static_cast<int>(i));
    }
    if (queue.empty()) return;
    while (!queue.empty()) {
        int i = queue.front();
        queue.pop_front();
        int c = i % w, r = i / w;
        const int nb[4][2] = {{c - 1, r}, {c + 1, r}, {c, r - 1}, {c, r + 1}};
        for (const auto& n : nb) {
            if (n[0] < 0 || n[0] >= w || n[1] < 0 || n[1] >= h) continue;
            int j = n[1] * w + n[0];
            if (done[j]) continue;
            done[j] = 1;
            elev[j] = elev[i];
            queue.push_back(j);
        }
    }
}
