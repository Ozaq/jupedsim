// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

/// Pair of Iterators for range based for loops
///
/// Provides begin and end method required for range based for loops.
/// _it_second must be reachable by _it_first.
#include <cstddef>
#include <iterator>
template <typename IteratorFirst, typename IteratorSecond = IteratorFirst>
class IteratorPair
{
    IteratorFirst itFirst;
    IteratorSecond itSecond;

public:
    IteratorPair(IteratorFirst it1, IteratorSecond it2) : itFirst(it1), itSecond(it2) {}

    IteratorFirst First() const { return itFirst; }
    IteratorSecond Second() const { return itSecond; }

    // NOLINTNEXTLINE(readability-identifier-naming)
    IteratorFirst begin() const { return First(); }
    // NOLINTNEXTLINE(readability-identifier-naming)
    IteratorSecond end() const { return Second(); }

    bool Empty() const { return itFirst == itSecond; }
    size_t Size() const { return std::distance(itFirst, itSecond); }
};
