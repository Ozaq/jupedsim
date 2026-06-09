// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "HashCombine.hpp"
#include "SimulationError.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <ranges>
#include <span>
#include <unordered_map>
#include <vector>

struct GridCell {
    std::int32_t x;
    std::int32_t y;

    auto operator<=>(const GridCell& other) const = default;
    bool operator==(const GridCell& other) const = default;
};

struct GridCellHash {
    std::size_t operator()(const GridCell& cell) const noexcept
    {
        std::hash<std::int32_t> hasher{};
        return jps::hash_combine(hasher(cell.x), hasher(cell.y));
    }
};

template <typename Handle>
class UniformGrid
{
private:
    double cellSize{1.0};
    std::unordered_map<GridCell, std::vector<Handle>, GridCellHash> cells{};

    std::span<const Handle> HandlesForCell(GridCell cell) const
    {
        if(auto it = cells.find(cell); it != cells.end()) {
            return std::span<const Handle>{it->second};
        }
        static const std::vector<Handle> empty{};
        return std::span<const Handle>{empty};
    }

public:
    UniformGrid(double cellSize)
    {
        if(cellSize <= 0.0 || !std::isfinite(cellSize)) {
            throw SimulationError(
                "UniformGrid requires positive cellSize. CellSize was: {}", cellSize);
        }
        this->cellSize = cellSize;
    }

    void Clear() { cells.clear(); }

    template <typename T>
    void Insert(const T& item, Handle handle)
    {
        for(auto&& cell : CellsFor(item, cellSize)) {
            cells[cell].emplace_back(handle);
        }
    }

    template <typename T>
    auto QueryRaw(const T& query) const
    {
        return CellsFor(query, cellSize) |
               std::views::transform([this](auto cell) { return HandlesForCell(cell); }) |
               std::views::join;
    }

    template <typename T>
    std::vector<Handle> Query(const T& query) const
    {
        std::vector<Handle> handles{};
        handles.reserve(64);
        std::ranges::for_each(
            QueryRaw(query), [&handles](auto handle) { handles.emplace_back(handle); });
        std::ranges::sort(handles, std::less<>{});
        const auto duplicates = std::ranges::unique(handles);
        handles.erase(std::begin(duplicates), std::end(duplicates));
        return handles;
    }
};
