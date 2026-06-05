// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "HashCombine.hpp"
#include "Point.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <unordered_map>
#include <vector>

template <typename T>
T* as_ptr(T* t)
{
    return t;
}

template <typename T>
const T* as_ptr(const T* t)
{
    return t;
}

template <typename T>
T* as_ptr(T& t)
{
    return &t;
}

template <typename T>
const T* as_ptr(const T& t)
{
    return &t;
}

struct Grid2DIndex {
    std::int32_t Idx;
    std::int32_t Idy;

    bool operator<(const Grid2DIndex& other) const
    {
        return Idx < other.Idx || (Idx == other.Idx && Idy < other.Idy);
    }

    bool operator==(const Grid2DIndex& other) const { return Idx == other.Idx && Idy == other.Idy; }
};

template <>
struct std::hash<Grid2DIndex> {
    std::size_t operator()(const Grid2DIndex& id) const noexcept
    {
        std::hash<std::int32_t> hasher{};
        return jps::hash_combine(hasher(id.Idx), hasher(id.Idy));
    }
};

template <typename Value>
class NeighborhoodSearch
{
    using Grid = std::unordered_map<Grid2DIndex, std::vector<Value>>;

    double cellSize;
    Grid grid{};

private:
    Grid2DIndex getIndex(const Point& pos) const
    {
        const int32_t Idx = static_cast<int32_t>(pos.X / cellSize);
        const int32_t Idy = static_cast<int32_t>(pos.Y / cellSize);
        return Grid2DIndex{Idx, Idy};
    }

public:
    explicit NeighborhoodSearch(double cellSize) : cellSize(cellSize) {};

    void AddAgent(const Value& item)
    {
        auto index = getIndex(item.Pos);
        auto& vec = grid[index];
        vec.push_back(item);
    }

    void RemoveAgent(const Value& item)
    {
        for(auto& [_, agents] : grid) {
            const auto iter =
                std::find_if(std::begin(agents), std::end(agents), [item](auto& agent) {
                    return agent.AgentID == item.AgentID;
                });
            if(iter != std::end(agents)) {
                agents.erase(iter);
                return;
            }
        }
        throw SimulationError("Unknown agent id {}", item.AgentID);
    }

    void Update(const std::vector<Value>& items)
    {
        grid.clear();
        for(const auto& item : items) {
            auto index = getIndex(item.Pos);
            auto& vec = grid[index];
            vec.push_back(item);
        }
    }

    std::vector<Value> GetNeighboringAgents(Point pos, double radius) const
    {
        std::vector<Value> result{};
        result.reserve(128);

        const auto posIdx = getIndex(pos);
        const auto offset = static_cast<int32_t>(std::ceil(radius / cellSize));
        const int32_t xMin = posIdx.Idx - offset;
        const int32_t xMax = posIdx.Idx + offset;
        const int32_t yMin = posIdx.Idy - offset;
        const int32_t yMax = posIdx.Idy + offset;

        const auto radiusSquared = radius * radius;

        for(int32_t x = xMin; x <= xMax; ++x) {
            for(int32_t y = yMin; y <= yMax; ++y) {
                auto it = grid.find({x, y});
                if(it != grid.cend()) {
                    for(const auto& item : it->second) {
                        if(DistanceSquared(item.Pos, pos) <= radiusSquared) {
                            result.emplace_back(item);
                        }
                    }
                }
            }
        }
        return result;
    }
};
