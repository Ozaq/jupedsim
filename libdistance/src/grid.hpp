#pragma once

#include "common_types.hpp"

#include <cassert>
#include <span>
#include <sstream>

namespace distance
{

template <typename T>
class Grid
{
    uint64_t width;
    uint64_t height;
    std::vector<T> data;

public:
    using ValueType = T; // Define ValueType for access in MapStencilView

    Grid(uint64_t width_, uint64_t height_, T default_value = {})
        : width(width_), height(height_), data(width * height, default_value)
    {
    }

    T& At(GridIndex idx)
    {
        assert(idx.x < width);
        assert(idx.y < height);

        return data[idx.x + idx.y * width];
    }

    const T& At(GridIndex idx) const
    {
        assert(idx.x < width);
        assert(idx.y < height);

        return data[idx.x + idx.y * width];
    }

    uint64_t Height() const { return height; }

    uint64_t Width() const { return width; }

    std::span<const T> Data() const { return std::span{data}; };

    uint64_t Size() const { return data.size(); }

    std::string DumpCSV() const
    {
        std::stringstream buf{};
        for(int linear_index = 0; linear_index < data.size(); ++linear_index) {
            buf << data[linear_index];
            if((linear_index + 1) % width == 0) {
                buf << '\n';
            } else {
                buf << ',';
            }
        }
        return buf.str();
    }
};

} // namespace distance
