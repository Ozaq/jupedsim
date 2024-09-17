#pragma once

#include "common_types.hpp"
#include "grid.hpp"

namespace distance
{

/// RasterMap represents the rasterised walkable area.
///
/// Rasterisation is always isotropic and cell sizes are configured at
/// construction time. Extend of the to-be-rasterised area need to be known at
/// construction time.
///
/// Cells in the raster are defined as half open intervalls
///     [index * cell_size, (index + 1) * cell_size)
///
/// Note! If you need to look at the raw data for debugging purposes for
/// example, you need to be aware that indices into the grid map onto the
/// positive x/y plane. If you dump the data and visualise it you need to
/// mirror the Y axis to match your input geometry.
///
/// World coordinates X,Y
///     +Y
///     ^         [n,n]
///     |
///     |
///     |
///     |[0,0]
///     |-----------> +X
///
///
template <Real Real, typename CellT>
class RasterMap
{
public:
    using GridT = Grid<CellT>;
    using PointT = Point<Real>;
    using LineSegmentT = LineSegment<Real>;
    using PolygonT = Polygon<Real>;

private:
    /// Cell size in 'cm'
    Real cell_size;
    /// Grid holding the cells state
    GridT grid;
    /// Point of the lower left corner of grid cell[0][0]
    PointT lower_left;

public:
    RasterMap(AABB<Real> bounds, Real _cell_size)
        : cell_size(_cell_size)
        , grid(
              static_cast<uint64_t>((bounds.Width() + cell_size) / cell_size),
              static_cast<uint64_t>((bounds.Height() + cell_size) / cell_size))
        , lower_left(bounds.LowerLeft())
    {
    }

    void MarkPoint(Point<Real> point, CellT value)
    {
        const auto [i, j] = gridIndex(point);
        grid.At(gridIndex(point)) = value;
    }

    void MarkLineSegment(LineSegmentT line, CellT value)
    {
        const auto xDim = grid.Width();
        const auto yDim = grid.Height();
        const auto p1_idx = gridIndex(line.p1);
        const auto p2_idx = gridIndex(line.p2);

        const int64_t si = p1_idx.x < p2_idx.x ? 1 : -1;
        const int64_t sj = p1_idx.y < p2_idx.y ? 1 : -1;

        int64_t dx = std::abs(static_cast<int64_t>(p2_idx.x - p1_idx.x));
        int64_t dy = -std::abs(static_cast<int64_t>(p2_idx.y - p1_idx.y));

        int64_t error = dx + dy, e2; // Initial error term

        int64_t i = p1_idx.x;
        int64_t j = p1_idx.y;

        while(true) {
            if(i >= 0 && i < static_cast<int64_t>(xDim) && j >= 0 && j < yDim) {
                grid.At({i, j}) = value;
            }

            if(i == p2_idx.x && j == p2_idx.y)
                break; // Check if the end point is reached

            e2 = 2 * error;
            if(e2 >= dy) { // Move to the next pixel in the x direction
                if(i == p2_idx.x)
                    break; // Check if the end point is reached in x direction
                error += dy;
                i += si;
            }
            if(e2 <= dx) { // Move to the next pixel in the y direction
                if(j == p2_idx.y)
                    break; // Check if the end point is reached in y direction
                error += dx;
                j += sj;
            }
        }
    }

    void MarkPolygon(PolygonT polygon, CellT value)
    {
        std::vector<GridIndex> vertexIndices;
        vertexIndices.reserve(polygon.size());

        std::transform(
            std::begin(polygon),
            std::end(polygon),
            std::back_inserter(vertexIndices),
            [this](const auto& p) { return gridIndex(p); });

        for(int64_t y = 0; y < static_cast<int64_t>(grid.Height()); ++y) {
            std::vector<int64_t> xIntersections;

            for(uint64_t i = 0; i < vertexIndices.size(); ++i) {
                const auto idx_a = vertexIndices[i];
                const int64_t x1 = static_cast<int64_t>(idx_a.x);
                const int64_t y1 = static_cast<int64_t>(idx_a.y);
                const auto idx_b = vertexIndices[(i + 1) % vertexIndices.size()];
                const int64_t x2 = static_cast<int64_t>(idx_b.x);
                const int64_t y2 = static_cast<int64_t>(idx_b.y);

                if(y1 == y2) {
                    continue; // Skip horizontal edges
                }

                // Check for scan line intersection
                if((y >= y1 && y < y2) || (y < y1 && y >= y2)) {
                    uint64_t xIntersect = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
                    assert(xIntersect < grid.Width());
                    xIntersections.push_back(xIntersect);
                }
            }

            std::sort(xIntersections.begin(), xIntersections.end());

            for(uint64_t i = 0; i + 1 < xIntersections.size(); i += 2) {
                for(uint64_t x = xIntersections[i]; x <= xIntersections[i + 1]; ++x) {
                    if(x >= 0 && x < grid.Width()) {
                        grid.At({x, static_cast<uint64_t>(y)}) = value;
                    }
                }
            }
        }
    }

    const CellT& At(Point<Real> point) const { return grid.At(gridIndex(point)); }

    CellT& At(Point<Real> point) { return grid.At(gridIndex(point)); }

    const GridT& Grid() { return grid; }

    GridIndex gridIndex(Point<Real> point) const
    {
        return {
            static_cast<uint64_t>((point.x - lower_left.x) / cell_size),
            static_cast<uint64_t>((point.y - lower_left.y) / cell_size)};
    }
};

} // namespace distance
