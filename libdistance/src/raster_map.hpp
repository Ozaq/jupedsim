#pragma once

#include "common_types.hpp"
#include "grid.hpp"

namespace distance
{

template <Real Real, typename CellT>
class RasterMap
{
    Real cell_size;
    Grid<CellT> grid;
    Point<Real> top_left;

public:
    RasterMap(AABB<Real> bounds, Real _cell_size)
        : cell_size(_cell_size)
        , grid(
              static_cast<uint64_t>((bounds.Width() + cell_size) / cell_size),
              static_cast<uint64_t>((bounds.Height() + cell_size) / cell_size))
        , top_left(bounds.TopLeft())
    {
    }

    void MarkPoint(Point<Real> point, CellT value)
    {
        const auto [i, j] = gridIndex(point);
        grid.At(gridIndex(point)) = value;
    }

    void MarkLineSegment(LineSegment<Real> line, CellT value)
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

    void MarkCircle(Circle<Real> circle);

    void MarkArc();

    void MarkPolygon();

    const CellT& At(Point<Real> point) const { return grid.At(gridIndex(point)); }

    CellT& At(Point<Real> point) { return grid.At(gridIndex(point)); }

private:
    GridIndex gridIndex(Point<Real> point) const
    {
        return {
            static_cast<uint64_t>((point.x - top_left.x) / cell_size),
            static_cast<uint64_t>(-(point.y - top_left.y) / cell_size)};
    }
};

} // namespace distance
