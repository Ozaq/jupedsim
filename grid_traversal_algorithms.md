# Grid Traversal Algorithms

This note describes algorithms for enumerating grid cells touched by a line
segment. The immediate use case is `CollisionGeometry`: inserting wall segments
into a spatial grid and querying candidate wall segments along an agent line of
sight.

## Problem

Given:

- a uniform 2D grid with fixed cell size
- a line segment from `p0` to `p1`

we need to visit every grid cell the segment passes through.

This is needed for two operations:

- construction: insert each wall segment into all cells it intersects
- query: collect wall candidates from all cells touched by a line-of-sight
  segment

The current custom implementation in `cellsFromLineSegment()` computes
intersections with grid lines, sorts them, and samples midpoints. That approach
is workable, but fragile around vertical/horizontal segments, duplicate grid
corner intersections, and floating-point edge cases.

## Amanatides-Woo / DDA

Amanatides-Woo is a grid traversal algorithm originally used for fast voxel
ray traversal. In 2D, it is also commonly called DDA traversal.

The algorithm walks through cells in the same order a ray or segment crosses
grid boundaries.

For a segment:

```text
p(t) = p0 + t * (p1 - p0), t in [0, 1]
```

it tracks:

- current cell
- target/end cell
- step direction in x and y
- parametric distance `tMaxX` to the next vertical grid boundary
- parametric distance `tMaxY` to the next horizontal grid boundary
- parametric delta `tDeltaX` between vertical grid crossings
- parametric delta `tDeltaY` between horizontal grid crossings

At each step:

```text
if tMaxX < tMaxY:
    move one cell in x
    tMaxX += tDeltaX
else if tMaxY < tMaxX:
    move one cell in y
    tMaxY += tDeltaY
else:
    the segment crosses a grid corner
    move in both x and y, or handle the corner convention explicitly
```

Stop once the end cell is reached.

## Why It Fits Here

For `CollisionGeometry`, this algorithm is a good fit because:

- it visits cells in traversal order
- it does not need to allocate or sort intersection points
- vertical and horizontal segments are natural cases
- it can be implemented as a callback to avoid temporary containers
- it works for both wall insertion and line-of-sight queries

The preferred shape is:

```cpp
template <typename Fn>
void ForEachCellTouchedBy(LineSegment segment, Fn&& fn);
```

Then construction can do:

```cpp
ForEachCellTouchedBy(wall, [&](Cell cell) {
    _grid[cell].push_back(segmentIndex);
});
```

And LOS querying can do:

```cpp
ForEachCellTouchedBy(los, [&](Cell cell) {
    collectCandidatesFrom(cell);
});
```

## Basic 2D Pseudocode

Assume integer grid cells:

```cpp
struct Cell {
    std::int32_t x;
    std::int32_t y;
};
```

and:

```cpp
Cell makeCell(Point p)
{
    return {
        static_cast<std::int32_t>(std::floor(p.x / cellSize)),
        static_cast<std::int32_t>(std::floor(p.y / cellSize)),
    };
}
```

Traversal outline:

```cpp
template <typename Fn>
void ForEachCellTouchedBy(Point p0, Point p1, double cellSize, Fn&& visit)
{
    Cell cell = makeCell(p0);
    const Cell end = makeCell(p1);

    visit(cell);

    const Point d = p1 - p0;

    if(d.x == 0.0 && d.y == 0.0) {
        return;
    }

    const int stepX = (d.x > 0.0) ? 1 : (d.x < 0.0) ? -1 : 0;
    const int stepY = (d.y > 0.0) ? 1 : (d.y < 0.0) ? -1 : 0;

    const double inf = std::numeric_limits<double>::infinity();

    const double nextBoundaryX = stepX > 0
        ? (cell.x + 1) * cellSize
        : cell.x * cellSize;
    const double nextBoundaryY = stepY > 0
        ? (cell.y + 1) * cellSize
        : cell.y * cellSize;

    double tMaxX = stepX == 0 ? inf : (nextBoundaryX - p0.x) / d.x;
    double tMaxY = stepY == 0 ? inf : (nextBoundaryY - p0.y) / d.y;

    const double tDeltaX = stepX == 0 ? inf : cellSize / std::abs(d.x);
    const double tDeltaY = stepY == 0 ? inf : cellSize / std::abs(d.y);

    while(cell != end) {
        if(tMaxX < tMaxY) {
            cell.x += stepX;
            tMaxX += tDeltaX;
        } else if(tMaxY < tMaxX) {
            cell.y += stepY;
            tMaxY += tDeltaY;
        } else {
            cell.x += stepX;
            cell.y += stepY;
            tMaxX += tDeltaX;
            tMaxY += tDeltaY;
        }

        visit(cell);
    }
}
```

This pseudocode uses one specific corner convention: when the segment crosses a
grid corner exactly, it steps diagonally and visits the cell on the far side of
the corner.

That matches a common half-open cell interpretation, but the convention must be
chosen deliberately.

## Corner Cases

### Vertical Segments

For vertical segments:

```text
d.x == 0
```

Set:

```text
tMaxX = infinity
tDeltaX = infinity
stepX = 0
```

The traversal only steps in y.

No division by zero is needed.

### Horizontal Segments

For horizontal segments:

```text
d.y == 0
```

Set:

```text
tMaxY = infinity
tDeltaY = infinity
stepY = 0
```

The traversal only steps in x.

No division by zero is needed.

### Zero-Length Segments

If:

```text
p0 == p1
```

only visit `makeCell(p0)`.

### Segment Starts On A Grid Boundary

This is the most important semantic choice.

With cells defined as half-open intervals:

```text
[min.x, min.x + cellSize) x [min.y, min.y + cellSize)
```

`makeCell()` assigns boundary points to the cell on the positive side for that
axis. For example, with `cellSize = 4`:

```text
x = 4 -> cell index 1
x = 0 -> cell index 0
x = -4 -> cell index -1
```

The traversal should use the same convention consistently.

### Segment Crosses A Grid Corner

When:

```text
tMaxX == tMaxY
```

the segment crosses a grid corner exactly.

There are three possible policies:

1. step diagonally and visit only the cell beyond the corner
2. visit both side-adjacent cells and then the diagonal cell
3. choose one side-adjacent cell based on a tie-break rule

For broad-phase collision detection, policy 2 is the most conservative because
it includes every cell that touches the segment at the corner. It can produce
more candidates.

For half-open cell semantics and consistency with the existing tests, policy 1
may be sufficient. It treats a corner touch as entering the cell on the far side
only.

The choice affects whether segments that only touch at a grid corner share a
cell. If `IntersectsAny()` is used as a broad phase, conservative traversal is
safer.

## Conservative Corner Traversal

If false negatives are unacceptable, use the conservative tie case:

```cpp
if(tMaxX == tMaxY) {
    const Cell xCell{cell.x + stepX, cell.y};
    const Cell yCell{cell.x, cell.y + stepY};
    const Cell xyCell{cell.x + stepX, cell.y + stepY};

    visit(xCell);
    visit(yCell);
    visit(xyCell);

    cell = xyCell;
    tMaxX += tDeltaX;
    tMaxY += tDeltaY;
}
```

This may visit duplicate cells, so either:

- tolerate duplicates and deduplicate segment candidates later
- keep the previous visited cell and skip immediate duplicates
- collect cells into a small vector and sort/unique

For geometry broad-phase insertion/query, conservative traversal is usually the
safer default.

## Floating-Point Equality

Comparing:

```cpp
tMaxX == tMaxY
```

can be fragile. A practical implementation should use either:

```cpp
std::abs(tMaxX - tMaxY) <= eps
```

or avoid relying on equality except for exact arithmetic cases.

Using an epsilon makes traversal more conservative near grid corners.

That is usually acceptable for broad-phase geometry queries because extra cells
only add extra candidates, while missing cells can cause false negatives.

## Area Query For Radius-Based Candidates

For wall repulsion and occlusion candidate reuse, a radius query is also useful:

```cpp
std::vector<const LineSegment*> LineSegmentsInApproxDistanceTo(
    Point p,
    double radius) const;
```

This does not need DDA. It is a grid range query:

```text
minCell = makeCell(p - Point{radius, radius})
maxCell = makeCell(p + Point{radius, radius})
for x in [minCell.x, maxCell.x]
    for y in [minCell.y, maxCell.y]
        collect grid[{x, y}]
```

This returns a candidate set, not an exact distance-filtered set.

If the same candidate set is reused for many neighbors, this can be faster than
running a DDA line query for every neighbor.

## Data Structure Recommendation

Use one geometry grid:

```cpp
std::vector<LineSegment> _segments;
std::unordered_map<Cell, std::vector<std::size_t>> _grid;
```

Store segment indices, not raw pointers.

Reasons:

- copying `CollisionGeometry` remains safe
- moving `CollisionGeometry` remains safe
- no pointer rebinding is needed
- one segment is stored once in `_segments`
- grid cells only store compact handles

Return pointers or references from query APIs if convenient:

```cpp
std::vector<const LineSegment*> result;
result.push_back(&_segments[index]);
```

Deduplicate indices before materializing pointers:

```cpp
std::sort(indices.begin(), indices.end());
indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
```

## Alternatives

### Current Intersection-Sort-Midpoint Algorithm

The current approach computes all intersections with grid lines, sorts them,
and samples midpoint cells.

It is simple to reason about for many diagonal cases, but it has drawbacks:

- vertical/horizontal segments need special handling
- duplicate grid-corner intersections need deduplication
- sorting points with NaN coordinates is dangerous
- it allocates a container of intersections
- it is harder to use as a no-allocation callback

It can be fixed, but DDA is a better fit.

### Bresenham-Style Integer Line Traversal

Bresenham algorithms are excellent for rasterizing integer grid lines.

They are less natural here because geometry uses floating-point coordinates and
segments can start/end anywhere inside cells.

Use Bresenham only if all coordinates are already quantized to grid indices.

That is not the case here.

### Liang-Barsky / Cohen-Sutherland Clipping Per Cell

Another approach is to query a cell range and clip the segment against each
cell AABB.

This is robust but usually more expensive:

- visits many cells in the segment bounding box
- performs a segment-AABB test per cell

It may matter if the query area is already needed for another reason, but it is
not ideal for line traversal.

### R-tree Or BVH

Spatial trees such as R-trees or bounding volume hierarchies can answer segment
intersection candidate queries efficiently.

They may be worth considering if:

- geometry becomes very large
- wall segments are highly non-uniformly distributed
- fixed grid cell size performs poorly across different map scales
- memory usage of grid duplication becomes problematic

For the current simulator geometry, a uniform grid is simpler and likely good
enough.

### Sweep And Prune

Sweep-and-prune is useful for dynamic broad-phase collision detection where
objects move and sorted bounds can be updated incrementally.

It does not fit static wall segment line-of-sight queries as well as a grid or
spatial tree.

### Exact Segment Against All Walls

The simplest fallback is:

```cpp
for(const auto& wall : _segments) {
    if(intersects(query, wall)) {
        return true;
    }
}
```

This is useful as a reference implementation in tests.

It is not suitable for hot simulation loops with large geometries.

## Recommended Direction

Use DDA/Amanatides-Woo for line-cell traversal.

Use a separate cell-range query for radius-based candidate collection.

Back both queries with one grid:

```cpp
std::unordered_map<Cell, std::vector<std::size_t>>
```

Do not keep a separate fixed-radius `_approximateGrid` unless profiling shows a
strong need for pre-expanded cells.

This gives:

- one source of truth for geometry spatial indexing
- no duplicated `LineSegment` storage in grid cells
- configurable query radius
- safe copy/move behavior if indices are used
- a clean path to accurate and fast line-of-sight occlusion filtering
