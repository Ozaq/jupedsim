#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include <fmt/format.h>

template <typename T>
concept UnsignedIntegral = std::is_integral_v<T> && std::is_unsigned_v<T>;

template <typename T>
concept Arithmetic = std::is_arithmetic_v<T>;

template <Arithmetic T>
struct Point {
    T x;
    T y;
};

template <Arithmetic T>
struct Polygon {
    std::vector<Point<T>> points;
};

template <Arithmetic T>
struct ExitLine {
    Point<T> p1;
    Point<T> p2;
};

template <Arithmetic T>
struct ExitPolygon {
    Polygon<Point<T>> polygon;
};

template <Arithmetic T>
struct Line {
    Point<T> p1;
    Point<T> p2;
};

template <Arithmetic T>
struct Arc {
    Point<T> center;
    T radius;
    T startAngle;
    T endAngle;
};

template <Arithmetic T>
struct Circle {
    Point<T> center;
    T radius;
};

inline size_t ToIndex(size_t x, size_t y, size_t xDim)
{
    return x + y * xDim;
}

inline std::pair<size_t, size_t> FromIndex(size_t i, size_t xDim)
{
    auto x = i % xDim;
    auto y = i / xDim;
    return {x, y};
}

template <UnsignedIntegral UI>
class Map
{
    size_t width;
    size_t height;
    std::vector<UI> data;

public:
    Map(size_t width_, size_t height_, UI default_value = {})
        : width(width_), height(height_), data(width * height, default_value)
    {
    }

    UI& At(size_t x, size_t y) { return data[x + y * width]; }
    UI At(size_t x, size_t y) const { return data[x + y * width]; }

    size_t Height() const { return height; }
    size_t Width() const { return width; }
};

/**
 * Distance Map
 *
 * Index -> Grid cell center
 *
 * @tparam T
 * @tparam U
 */
template <UnsignedIntegral T, Arithmetic U>
class DistanceMap
{
private:
    using SignedT = typename std::make_signed_t<T>; // Get the corresponding signed type

    // TODO(TS) data type
    U xMin{};
    U xMax{};
    U yMin{};
    U yMax{};

    T xDim{};
    T yDim{};

    std::vector<T> distance{};

    Map<T> personToIntermediate;

public:
    static constexpr U CELL_SIZE = 0.2; // in meter
    static constexpr size_t BLOCK_SIZE = 11; // size of one quadrant
    static constexpr T CELL_SIZE_CM = DistanceMap<T, U>::CELL_SIZE * 100;

    static constexpr T FREE_SPACE = std::numeric_limits<T>::max();
    static constexpr T BLOCKED = std::numeric_limits<T>::max() - 1;

    DistanceMap(
        U xMin,
        U yMin,
        U xMax,
        U yMax,
        T xDim,
        T yDim,
        std::vector<T> distance,
        Map<T> personToIntermediate)
        : xMin(xMin)
        , yMin(yMin)
        , xMax(xMax)
        , yMax(yMax)
        , xDim(xDim)
        , yDim(yDim)
        , distance(std::move(distance))
        , personToIntermediate(std::move(personToIntermediate))
    {
    }

    std::pair<size_t, size_t> GridSize() const { return {xDim, yDim}; }

    T GetValue(U x, U y) const;
    T GetValue(size_t i, size_t j) const;

    class Proxy
    {
        DistanceMap& map;
        size_t x;

    public:
        Proxy(DistanceMap& map, size_t x) : map(map), x(x) {}

        T& operator[](size_t y)
        {
            assert(y < map.yDim && "y index out of bounds.");
            return map.distance[x + map.xDim * y];
        }
    };

    // Overload the [] operator to return a Proxy instance for row access
    Proxy operator[](size_t x)
    {
        assert(x < xDim && "x index out of bounds.");
        return Proxy(*this, x);
    }

    Point<U> GetNextTarget(Point<U> position)
    {
        // get local distances (all quadrants) -> localDist
        // get stencil around position (values from distance map) -> distance

        // localDist + distance -> distanceToExit

        // distanceToExit - (value at position) -> surplusDistanceToExit

        // find 0 value with largest distance to center in surplusDistanceToExit
        //
    }

private:
    //    void markCircle(int xm, int ym, int r, T value)
    //    {
    //        int x = -r, y = 0, err = 2 - 2 * r; /* II. Quadrant */
    //        do {
    //            (*this)[xm - x][ym + y] = value;
    //            (*this)[xm - y][ym - x] = value;
    //            (*this)[xm + x][ym - y] = value;
    //            (*this)[xm + y][ym + x] = value;
    //
    //            //          setPixel(xm-x, ym+y); /*   I. Quadrant */
    //            //          setPixel(xm-y, ym-x); /*  II. Quadrant */
    //            //          setPixel(xm+x, ym-y); /* III. Quadrant */
    //            //          setPixel(xm+y, ym+x); /*  IV. Quadrant */
    //            r = err;
    //            if(r <= y)
    //                err += ++y * 2 + 1; /* e_xy+e_y < 0 */
    //            if(r > x || err > y)
    //                err += ++x * 2 + 1; /* e_xy+e_x > 0 or no 2nd y-step */
    //        } while(x < 0);
    //    }
    //
    //    void
    //    drawCircleArc(size_t xCenter, size_t yCenter, size_t radius, double startAngle, double
    //    endAngle)
    //    {
    //        // Convert angles from degrees to radians
    //        double startRad = startAngle * (M_PI / 180.0);
    //        double endRad = endAngle * (M_PI / 180.0);
    //
    //        // Ensure the end angle is greater than the start angle
    //        if(endRad < startRad) {
    //            std::swap(endRad, startRad);
    //        }
    //
    //        // Step size for drawing (smaller = more precise)
    //        double thetaStep =
    //            1.0 / static_cast<double>(radius); // Adjust based on radius for smoother curves
    //
    //        for(double theta = startRad; theta <= endRad; theta += thetaStep) {
    //            size_t x = static_cast<size_t>(round(xCenter + radius * cos(theta)));
    //            size_t y = static_cast<size_t>(round(yCenter + radius * sin(theta)));
    //
    //            if(x < xDim && y < yDim) { // Ensure the point is within bounds
    //                (*this)[y][x] = BLOCKED;
    //            }
    //        }
    //    }
    //
    //    void fillPolygon2(const std::vector<std::pair<T, T>> vertices, T value)
    //    {
    //        for(size_t i = 0; i < vertices.size(); ++i) {
    //            auto p1 = vertices[i];
    //            auto p2 = vertices[(i + 1) % vertices.size()];
    //            markLine(p1.first, p1.second, p2.first, p2.second, value);
    //        }
    //    }
    //
    //    void fillPolygon(const std::vector<std::pair<int, int>>& vertices)
    //    {
    //        for(int y = 0; y < static_cast<int>(yDim); ++y) {
    //            std::vector<int> xIntersections;
    //
    //            for(size_t i = 0; i < vertices.size(); ++i) {
    //                auto [x1, y1] = vertices[i];
    //                auto [x2, y2] = vertices[(i + 1) % vertices.size()];
    //
    //                if(y1 == y2)
    //                    continue; // Skip horizontal edges
    //
    //                if((y >= y1 && y < y2) || (y < y1 && y >= y2)) {
    //                    int xIntersect =
    //                        static_cast<int>(x1 + (y - y1) * (double) (x2 - x1) / (double) (y2 -
    //                        y1));
    //                    xIntersections.push_back(xIntersect);
    //                }
    //            }
    //
    //            std::sort(xIntersections.begin(), xIntersections.end());
    //
    //            for(size_t i = 0; i + 1 < xIntersections.size(); i += 2) {
    //                for(int x = xIntersections[i]; x <= xIntersections[i + 1]; ++x) {
    //                    if(x >= 0 && x < static_cast<int>(xDim) && y >= 0 &&
    //                       y < static_cast<int>(yDim)) {
    //                        (*this)[y][x] = BLOCKED;
    //                    }
    //                }
    //            }
    //        }
    //    }
};

template <UnsignedIntegral UI, Arithmetic A>
void PrintDistanceMap(const std::vector<UI>& distance, size_t xDim, size_t yDim)
{
    using I = typename std::make_signed_t<UI>; // Get the corresponding signed type

    std::string output;
    for(I y = yDim - 1; y >= 0; --y) {
        for(size_t x = 0; x < xDim; ++x) {
            const auto& val = distance[ToIndex(x, y, xDim)];
            if(val == DistanceMap<UI, A>::FREE_SPACE) {
                output += fmt::format("  -  ");
            } else if(val == DistanceMap<UI, A>::BLOCKED) {
                output += fmt::format("  x  ");
            } else {
                output += fmt::format("{:^4d} ", val);
            }
        }
        output += '\n';
    }
    std::cout << output << "\n";
}

void PrintBoolDistanceMap(const std::vector<bool>& visited, size_t xDim, size_t yDim)
{
    using I = typename std::make_signed_t<size_t>; // Get the corresponding signed type

    std::string output;
    for(I y = yDim - 1; y >= 0; --y) {
        for(size_t x = 0; x < xDim; ++x) {
            output += fmt::format("{:^6} ", static_cast<bool>(visited[ToIndex(x, y, xDim)]));
        }
        output += '\n';
    }
    std::cout << output << "\n";
}

template <UnsignedIntegral UI, Arithmetic A>
std::pair<size_t, size_t> ToGrid(UI x, UI y, A xMin, A yMin)
{
    // TODO(TS) asserts
    auto i = (x - xMin) / DistanceMap<UI, A>::CELL_SIZE;
    auto j = (y - yMin) / DistanceMap<UI, A>::CELL_SIZE;
    return {i, j};
}

template <UnsignedIntegral UI, Arithmetic A>
std::pair<size_t, size_t> ToGrid(Point<A> p, A xMin, A yMin)
{
    // TODO(TS) asserts
    auto i = (p.x - xMin) / DistanceMap<UI, A>::CELL_SIZE;
    auto j = (p.y - yMin) / DistanceMap<UI, A>::CELL_SIZE;
    return {i, j};
}

template <UnsignedIntegral UI, Arithmetic A>
std::pair<A, A> ToWorld(size_t i, size_t j, A xMin, A yMin)
{
    // TODO(TS) asserts
    auto x = xMin + i * DistanceMap<UI, A>::CELL_SIZE + 0.5 * DistanceMap<UI, A>::CELL_SIZE;
    auto y = yMin + j * DistanceMap<UI, A>::CELL_SIZE + 0.5 * DistanceMap<UI, A>::CELL_SIZE;
    return {x, y};
}

template <UnsignedIntegral T, Arithmetic U>
class DistanceMapBuilder
{
private:
    using SignedT = typename std::make_signed_t<T>; // Get the corresponding signed type

    std::vector<Line<U>> lines{};
    std::vector<Polygon<U>> polygons{};
    std::vector<Arc<U>> arcs{};
    std::vector<Circle<U>> circles{};

    std::vector<Line<U>> exitLines{};
    std::vector<Polygon<U>> exitPolygons{};

    std::tuple<U, U, U, U> computeBoundingBox() const
    {
        U xMin{std::numeric_limits<U>::max()};
        U xMax{std::numeric_limits<U>::lowest()};
        U yMin{std::numeric_limits<U>::max()};
        U yMax{std::numeric_limits<U>::lowest()};

        for(const auto& line : lines) {
            xMin = std::min(xMin, std::min(line.p1.x, line.p2.x));
            xMax = std::max(xMax, std::max(line.p1.x, line.p2.x));
            yMin = std::min(yMin, std::min(line.p1.y, line.p2.y));
            yMax = std::max(yMax, std::max(line.p1.y, line.p2.y));
        }
        //        for(const auto& polygon : polygons) {
        //            for(const auto& point : polygon.points) {
        //                xMin = std::min(xMin, point.x);
        //                yMin = std::min(yMin, point.y);
        //            }
        //        }
        //        for(const auto& arc : arcs) {
        //        }

        //        for(const auto& circle : circles) {
        //            xMin =
        //        }

        for(const auto& exitLine : exitLines) {
            xMin = std::min(xMin, std::min(exitLine.p1.x, exitLine.p2.x));
            xMax = std::max(xMax, std::max(exitLine.p1.x, exitLine.p2.x));
            yMin = std::min(yMin, std::min(exitLine.p1.y, exitLine.p2.y));
            yMax = std::max(yMax, std::max(exitLine.p1.y, exitLine.p2.y));
        }

        //        for(const auto& exitPolygon : exitPolygons) {
        //            for(const auto& point : exitPolygon.points) {
        //                xMin = std::min(xMin, point.x);
        //                yMin = std::min(yMin, point.y);
        //            }
        //        }

        xMin -= leftPadding;
        yMin -= downPadding;
        xMax += rightPadding;
        yMax += upPadding;

        return std::make_tuple(xMin, yMin, xMax, yMax);
        //        return std::make_tuple(
        //            xMin - leftPadding, yMin - downPadding, xMax + rightPadding, yMax +
        //            upPadding);
    }

    U leftPadding{0};
    U rightPadding{0};
    U upPadding{0};
    U downPadding{0};

    void MarkObstacles(
        std::vector<T>& distance,
        size_t xDim,
        size_t yDim,
        U xMin,
        U yMin,
        T fillValue = DistanceMap<T, U>::BLOCKED) const
    {
        // mark lines
        for(const auto& line : lines) {
            MarkLine(distance, xDim, yDim, xMin, yMin, line, DistanceMap<T, U>::BLOCKED);
        }

        // mark arcs
        // mark circles
    }

    void
    MarkExits(std::vector<T>& distance, size_t xDim, size_t yDim, U xMin, U yMin, T fillValue = 0)
        const
    {
        // mark exit lines
        for(const auto& exitLine : exitLines) {
            MarkLine(distance, xDim, yDim, xMin, yMin, exitLine, 0);
        }

        // mark exit polygons
    }

    void MarkLine(
        std::vector<T>& distance,
        size_t xDim,
        size_t yDim,
        U xMin,
        U yMin,
        Line<U> line,
        T fillValue = DistanceMap<T, U>::BLOCKED) const
    {

        const auto [i1, j1] = ToGrid<T, U>(line.p1, xMin, yMin);
        const auto [i2, j2] = ToGrid<T, U>(line.p2, xMin, yMin);

        const SignedT si = i1 < i2 ? 1 : -1;
        const SignedT sj = j1 < j2 ? 1 : -1;

        SignedT dx = std::abs(static_cast<SignedT>(i2 - i1));
        SignedT dy = -std::abs(static_cast<SignedT>(j2 - j1));

        SignedT error = dx + dy, e2; // Initial error term

        SignedT i = i1, j = j1;

        while(true) {
            if(i >= 0 && i < xDim && j >= 0 && j < yDim) {
                distance[ToIndex(i, j, xDim)] = fillValue;
            }

            if(i == i2 && j == j2)
                break; // Check if the end point is reached

            e2 = 2 * error;
            if(e2 >= dy) { // Move to the next pixel in the x direction
                if(i == i2)
                    break; // Check if the end point is reached in x direction
                error += dy;
                i += si;
            }
            if(e2 <= dx) { // Move to the next pixel in the y direction
                if(j == j2)
                    break; // Check if the end point is reached in y direction
                error += dx;
                j += sj;
            }
        }
    }

    void MarkCircle(Circle<U> circle, T fillValue = DistanceMap<T, U>::BLOCKED) const;
    void MarkArc(Arc<U> arc, T fillValue = DistanceMap<T, U>::BLOCKED) const;
    void MarkPolygon(Arc<U> arc, T fillValue = DistanceMap<T, U>::BLOCKED) const;

    void
    PrepareDistanceMap(std::vector<T>& distance, size_t xDim, size_t yDim, U xMin, U yMin) const
    {
        MarkObstacles(distance, xDim, yDim, xMin, yMin);
        MarkExits(distance, xDim, yDim, xMin, yMin);
    }

    void ComputeDistanceMap(
        std::vector<T>& distance,
        size_t xDim,
        size_t yDim,
        const Map<T>& localDistance) const
    {
        std::vector<bool> visited(distance.size(), false);

        std::queue<std::pair<size_t, size_t>> queue;

        // find zero values and put in queue, mark as visited
        for(size_t i = 0; i < distance.size(); ++i) {
            if(distance[i] == 0) {
                visited[i] = true;
                queue.emplace(FromIndex(i, xDim));
            }
        }

        while(!queue.empty()) {

            const auto [idx_x, idx_y] = queue.front();
            queue.pop();
            UpdateDistances(distance, xDim, yDim, idx_x, idx_y, localDistance);

            const auto signed_idx_x = static_cast<int64_t>(idx_x);
            const auto signed_idx_y = static_cast<int64_t>(idx_y);

            for(int dx = -1; dx <= 1; ++dx) {
                for(int dy = -1; dy <= 1; ++dy) {
                    const auto neighbor_idx_x = signed_idx_x + dx;
                    const auto neighbor_idx_y = signed_idx_y + dy;

                    if(neighbor_idx_x >= 0 && neighbor_idx_x < xDim && neighbor_idx_y >= 0 &&
                       neighbor_idx_y < yDim &&
                       !visited[ToIndex(neighbor_idx_x, neighbor_idx_y, xDim)]) {
                        queue.emplace(neighbor_idx_x, neighbor_idx_y);
                        visited[ToIndex(neighbor_idx_x, neighbor_idx_y, xDim)] = true;
                    }
                }
            }
        }
    }

    void UpdateDistances(
        std::vector<T>& distance,
        size_t xDim,
        size_t yDim,
        size_t x,
        size_t y,
        const Map<T>& localDistance) const
    {
        const auto centerValue = distance[ToIndex(x, y, xDim)];

        // spiral movement in to out
        const int movementDirections[2] = {-1, 1};

        for(const auto yMovementDirection : movementDirections) {
            const auto yLimit = ((0.5 * DistanceMap<T, U>::BLOCK_SIZE + 1) * yMovementDirection);
            for(const auto xMovementDirection : movementDirections) {
                auto xLimit = ((0.5 * DistanceMap<T, U>::BLOCK_SIZE + 1) * xMovementDirection);
                auto xObstacle = std::numeric_limits<T>::max();

                for(int yDisplacement = 0; std::abs(yDisplacement) <= std::abs(yLimit);
                    yDisplacement += yMovementDirection) {
                    const auto yCurrent = y + yDisplacement;

                    if(yCurrent >= yDim) {
                        break;
                    }
                    for(int xDisplacement = 0; std::abs(xDisplacement) <= std::abs(xLimit);
                        xDisplacement += xMovementDirection) {
                        const auto xCurrent = x + xDisplacement;
                        if(xCurrent >= xDim || xCurrent >= xObstacle) {
                            break;
                        }

                        const auto valueCurrent = distance[ToIndex(xCurrent, yCurrent, xDim)];

                        if(valueCurrent == DistanceMap<T, U>::BLOCKED) {
                            xObstacle = xCurrent;
                            break;
                        }

                        if(valueCurrent < centerValue) {
                            continue;
                        }

                        const auto candidateValue =
                            centerValue +
                            localDistance.At(std::abs(xDisplacement), std::abs(yDisplacement));
                        //                            std::sqrt(
                        //                                std::pow(
                        //                                    std::abs(
                        //                                        xDisplacement *
                        //                                        static_cast<SignedT>(DistanceMap<T,
                        //                                        U>::CELL_SIZE_CM)),
                        //                                    2) +
                        //                                std::pow(
                        //                                    std::abs(
                        //                                        yDisplacement *
                        //                                        static_cast<SignedT>(DistanceMap<T,
                        //                                        U>::CELL_SIZE_CM)),
                        //                                    2));

                        if(valueCurrent == DistanceMap<T, U>::FREE_SPACE ||
                           candidateValue < valueCurrent) {
                            distance[ToIndex(xCurrent, yCurrent, xDim)] = candidateValue;
                        }
                    }
                }
            }
        }
    }

    Map<T> ComputeLocalDistance() const
    {
        Map<T> localDistance(DistanceMap<T, U>::BLOCK_SIZE, DistanceMap<T, U>::BLOCK_SIZE);
        for(size_t x = 0; x < localDistance.Width(); ++x) {
            for(size_t y = 0; y < localDistance.Height(); ++y) {
                localDistance.At(x, y) = static_cast<T>(
                    0.5 +
                    DistanceMap<T, U>::CELL_SIZE_CM * std::sqrt(static_cast<float>(x * x + y * y)));
            }
        }
        return localDistance;
    }

public:
    // TODO(TS)
    void SetBoundingBox();

    void AddLeftPadding(U leftPadding_) { leftPadding = leftPadding_; }
    void AddRightPadding(U rightPadding_) { rightPadding = rightPadding_; }
    void AddDownPadding(U downPadding_) { downPadding = downPadding_; }
    void AddUpPadding(U upPadding_) { upPadding = upPadding_; }

    void AddLine(const Line<U>& line) { lines.emplace_back(line); }

    void AddPolygon(const Polygon<U>& polygon)
    {
        // TODO(TS) add ...
    }
    void AddArc(const Arc<U>& arc) { arcs.emplace_back(arc); }
    void AddCircle(const Circle<U>& circle) { circles.emplace_back(circle); }
    void AddExitLine(const Line<U>& exitLine) { exitLines.emplace_back(exitLine); }
    void AddExitPolygon(const Polygon<U> exitPolygon);
    //    TODO(TS): needed for Simulex?
    //    void AddBoundingBox(const )

    std::unique_ptr<const DistanceMap<T, U>> Build() const
    {
        //        if (exitLines.empty() && exitPolygons.empty()){
        //            throw std::runtime_error("No exit lines and no exit polygons specified");
        //        }

        // compute bounding box -> mapping coordinate to grid cell
        const auto [xMin, yMin, xMax, yMax] = computeBoundingBox();
        std::cout << fmt::format("x: {} - {}\ty: {} - {}\n", xMin, xMax, yMin, yMax);

        //        auto distanceMap = std::make_unique<DistanceMap<T, U>>(xMin, yMin, xMax, yMax);
        auto xDim = computeXGridSize(xMin, xMax);
        auto yDim = computeYGridSize(yMin, yMax);

        std::vector<T> distance(xDim * yDim, DistanceMap<T, U>::FREE_SPACE);

        auto localDistance = ComputeLocalDistance();

        std::cout << "empty: \n";
        PrintDistanceMap<T, U>(distance, xDim, yDim);

        PrepareDistanceMap(distance, xDim, yDim, xMin, yMin);
        std::cout << "Prepared: \n";
        PrintDistanceMap<T, U>(distance, xDim, yDim);

        ComputeDistanceMap(distance, xDim, yDim, localDistance);

        std::cout << "Final: \n";
        PrintDistanceMap<T, U>(distance, xDim, yDim);

        return std::make_unique<DistanceMap<T, U>>(
            xMin, yMin, xMax, yMax, xDim, yDim, std::move(distance), std::move(localDistance));
    }

    T computeXGridSize(U xMin, U xMax) const
    {
        return static_cast<T>(
            (std::abs(xMax - xMin) + DistanceMap<T, U>::CELL_SIZE) / DistanceMap<T, U>::CELL_SIZE);
        //        return static_cast<T>((std::abs(xMax - xMin) + 0.9999 * CELL_SIZE) / CELL_SIZE);
    }

    T computeYGridSize(U yMin, U yMax) const
    {
        return static_cast<T>(
            (std::abs(yMax - yMin) + DistanceMap<T, U>::CELL_SIZE) / DistanceMap<T, U>::CELL_SIZE);
        //        return static_cast<T>((std::abs(yMax - yMin) + 0.9999 * CELL_SIZE) / CELL_SIZE);
    }
};
