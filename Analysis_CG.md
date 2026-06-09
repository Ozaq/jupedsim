`libsimulator/src/CollisionGeometry.hpp` and
`libsimulator/src/CollisionGeometry.cpp` implement the static obstacle/collision
representation for the simulator geometry.

It takes a CGAL polygon-with-holes representing the accessible area, extracts
all boundary and hole edges as `LineSegment`s, and builds spatial acceleration
structures for intersection and proximity queries.

The general design is sensible: the geometry is immutable after construction,
wall segments are extracted once, and repeated simulation queries use
precomputed structures. There are also dedicated tests for cell adjacency, cell
coverage, and approximate line-segment lookup. However, there are several
correctness and maintainability concerns. The most important glaring issue is in
`cellsFromLineSegment()`: it divides by `vec_p1p2.x` and `vec_p1p2.y` without
guarding vertical or horizontal line segments. In common cases infinities happen
to flow through harmlessly enough, but it is fragile and can generate invalid
intermediate points.

**What It Does**

`CollisionGeometry` stores:

```cpp
ID _id{};
PolyWithHoles _accessibleAreaPolygon;
std::vector<LineSegment> _segments;
std::unordered_map<Cell, std::set<LineSegment>> _grid{};
std::unordered_map<Cell, std::vector<LineSegment>> _approximateGrid{};
std::tuple<std::vector<Point>, std::vector<std::vector<Point>>> _accessibleArea{};
```

The important pieces are:

- `_accessibleAreaPolygon`: the CGAL polygon-with-holes used for exact
  inside/outside tests.
- `_segments`: all outer-boundary and hole-boundary line segments.
- `_grid`: an exact-ish spatial grid mapping touched cells to wall segments,
  used by `IntersectsAny()`.
- `_approximateGrid`: a broader spatial grid mapping cells to nearby wall
  segments, used by `LineSegmentsInApproxDistanceTo()`.
- `_accessibleArea`: a converted point-vector representation of the polygon and
  holes, likely for API/bindings/inspection.

`Cell` is currently just an alias:

```cpp
using Cell = Point;
```

Cells are represented by their bottom-left world coordinate, not by integer
indices. With `CELL_EXTEND = 4`, a cell at `{8, -4}` represents roughly:

```cpp
[8, 12) x [-4, 0)
```

The helper:

```cpp
Cell makeCell(Point p)
```

maps a world coordinate to the bottom-left coordinate of its containing grid
cell:

```cpp
return {floor(p.x / CELL_EXTEND) * CELL_EXTEND,
        floor(p.y / CELL_EXTEND) * CELL_EXTEND};
```

Unlike `NeighborhoodSearch`, this correctly uses `floor`, so negative
coordinates map to the expected negative cells.

**Construction Flow**

The constructor receives a `PolyWithHoles`:

```cpp
explicit CollisionGeometry(PolyWithHoles accessibleArea);
```

It then:

1. Reserves enough storage in `_segments` by counting outer and hole polygon
   edges.
2. Extracts line segments from the outer polygon boundary.
3. Extracts line segments from every hole boundary.
4. Inserts each segment into `_grid` for exact intersection acceleration.
5. Inserts each segment into `_approximateGrid` for approximate proximity
   acceleration.
6. Shrinks vectors in `_approximateGrid`.
7. Converts the CGAL polygon and holes into `std::vector<Point>` form and stores
   it in `_accessibleArea`.

Segment extraction is straightforward:

```cpp
for(size_t index = 1; index < boundary.size(); ++index) {
    segments.emplace_back(fromPoint_2(boundary[index - 1]), fromPoint_2(boundary[index]));
}
segments.emplace_back(fromPoint_2(boundary.back()), fromPoint_2(boundary.front()));
```

That explicitly closes each polygon ring.

**Public API**

`LineSegmentsInDistanceTo(double distance, Point p) const`

Returns an iterator range over all stored segments whose exact distance to `p`
is at most `distance`.

This scans `_segments` linearly and filters lazily through
`DistanceQueryIterator`.

`LineSegmentsInApproxDistanceTo(Point p) const`

Returns a reference to a vector of line segments from `_approximateGrid` for the
cell containing `p`.

This does not check exact distance. It is a broad-phase/candidate query.

`IntersectsAny(const LineSegment& linesegment) const`

Computes all grid cells touched by the query line segment, checks those cells in
`_grid`, and returns true if any candidate segment geometrically intersects the
query.

`InsideGeometry(Point p) const`

Uses CGAL oriented-side classification against the polygon-with-holes:

```cpp
return CGAL::oriented_side(K::Point_2(p.x, p.y), _accessibleAreaPolygon) !=
       CGAL::ON_NEGATIVE_SIDE;
```

This treats points inside the accessible area or on its boundary as inside.

`AccessibleArea() const`

Returns the cached tuple of exterior points and holes.

`Polygon() const`

Returns the underlying CGAL polygon-with-holes.

`Id() const`

Returns the unique geometry id.

**Cell And Grid Logic**

`makeCell()` is good in principle:

```cpp
Cell makeCell(Point p)
{
    return {floor(p.x / CELL_EXTEND) * CELL_EXTEND, floor(p.y / CELL_EXTEND) * CELL_EXTEND};
}
```

It uses mathematical floor and therefore handles negative coordinates correctly.

For example, with `CELL_EXTEND = 4`:

```cpp
makeCell({ 3.9,  0.0}) -> { 0, 0}
makeCell({ 4.0,  0.0}) -> { 4, 0}
makeCell({-0.1,  0.0}) -> {-4, 0}
makeCell({-4.0,  0.0}) -> {-4, 0}
makeCell({-4.1,  0.0}) -> {-8, 0}
```

That is the right behavior for a grid.

`IsN8Adjacent()` checks whether two cells are one grid step apart in either
axis, excluding equality:

```cpp
const auto dx = static_cast<int>(abs(a.x - b.x) / CELL_EXTEND);
const auto dy = static_cast<int>(abs(a.y - b.y) / CELL_EXTEND);
if((dx == 0 && dy == 0) || dx > 1 || dy > 1) {
    return false;
}
return true;
```

The intent is N8 adjacency: horizontal, vertical, and diagonal neighbors are
adjacent; the same cell is not adjacent.

`cellsFromLineSegment()` maps a line segment to every grid cell it passes
through. That is the key function for `IntersectsAny()` correctness, because
`_grid` only contains a segment in cells returned by this function.

The rough algorithm is:

1. Compute the cell for each endpoint.
2. If both endpoints are in the same cell, return that cell.
3. If endpoint cells are N8 adjacent, return just the endpoint cells.
4. Otherwise, collect intersections with vertical and horizontal grid lines
   inside the segment bounding box.
5. Sort those intersection points along lexicographic `Point` order.
6. Insert the cell containing the midpoint between each consecutive pair of
   intersections.
7. Return all collected cells.

This is a custom grid traversal algorithm.

**What Is Good**

The main architecture is pragmatic and efficient for static geometry.

Extracting all wall/hole boundaries once during construction is the right shape
for simulation use. Most simulation steps query the same geometry repeatedly, so
paying construction cost once is good.

`InsideGeometry()` delegates to CGAL rather than reimplementing point-in-polygon
logic. That is a good correctness choice, especially with holes.

`makeCell()` correctly handles negative coordinates using `floor`.

The distinction between `_grid` and `_approximateGrid` is useful:

- `_grid` is for precise segment-intersection broad-phase lookup.
- `_approximateGrid` is for fast wall-nearby candidate lookup in operational
  models.

`IntersectsAny()` avoids testing against every geometry segment. It only checks
candidates in cells touched by the query segment.

`LineSegmentsInDistanceTo()` exposes a lazy filtered range rather than eagerly
allocating a result vector.

The tests cover a meaningful amount of tricky grid behavior, including negative
coordinates and long diagonal segments.

The approximate grid tests are particularly valuable because they lock down
expected candidate sets for simple and diagonal geometry.

The code is reasonably small and readable. There is not much unnecessary
abstraction.

**Glaring Bug: Division By Zero In `cellsFromLineSegment()`**

This is the biggest correctness smell:

```cpp
const auto vec_p1p2 = ls.p2 - ls.p1;

for(double x_intersect = toMultiple(bounds.xmin); x_intersect <= bounds.xmax;
    x_intersect += CELL_EXTEND) {
    const double fact = (x_intersect - ls.p1.x) / vec_p1p2.x;
    intersections.emplace_back(x_intersect, ls.p1.y + fact * vec_p1p2.y);
}
for(double y_intersect = toMultiple(bounds.ymin); y_intersect <= bounds.ymax;
    y_intersect += CELL_EXTEND) {
    const double fact = (y_intersect - ls.p1.y) / vec_p1p2.y;
    intersections.emplace_back(ls.p1.x + fact * vec_p1p2.x, y_intersect);
}
```

For vertical line segments, `vec_p1p2.x == 0`.

For horizontal line segments, `vec_p1p2.y == 0`.

For zero-length line segments, both are zero.

The function has early returns for same-cell and N8-adjacent endpoints, so many
small vertical/horizontal cases avoid the dangerous division. But long vertical
and horizontal segments do reach the loops.

Example from the tests:

```cpp
LineSegment{{0, 0}, {0, 8}}
```

This is vertical and spans multiple cells. In the x-intersection loop:

```cpp
fact = (0 - 0) / 0
```

That is `0 / 0`, which produces `NaN` for floating-point arithmetic. The code
then creates an intersection point with a NaN coordinate:

```cpp
intersections.emplace_back(0, NaN)
```

For horizontal segments, the same issue appears in the y-intersection loop.

The current tests pass or are expected to pass because the other loop still adds
useful points, endpoint cells are already inserted, and sorting/midpoint
behavior may accidentally work with the existing `Point` ordering. But relying
on NaNs inside sorting is fragile.

Sorting a vector containing NaN coordinates can violate strict weak ordering
depending on how `Point::operator<` is implemented. If comparisons involving NaN
return inconsistent or non-useful results, `std::sort` behavior is not something
to rely on.

Even if it does not crash, it can produce missing cells or
platform/compiler-dependent results.

This should be fixed by skipping vertical-grid intersections when
`vec_p1p2.x == 0` and skipping horizontal-grid intersections when
`vec_p1p2.y == 0`, or by replacing this function with a more standard grid
traversal algorithm.

**Likely Bug: Duplicate Grid-Boundary Intersections**

`cellsFromLineSegment()` includes grid lines at both bounds:

```cpp
for(double x_intersect = toMultiple(bounds.xmin); x_intersect <= bounds.xmax;
    x_intersect += CELL_EXTEND)
```

and similarly for y.

This means endpoint grid lines and corner crossings are included.

For a diagonal crossing exactly through grid corners, the same physical
intersection can be inserted twice: once from the vertical-grid loop and once
from the horizontal-grid loop.

For example, a segment from `{0, 0}` to `{8, 8}` crosses grid corners at
`{0,0}`, `{4,4}`, and `{8,8}`.

The function may add the same intersection multiple times. Then it computes
midpoint cells between consecutive sorted intersections:

```cpp
for(size_t index = 1; index < intersections.size(); ++index) {
    cells.insert(makeCell((intersections[index - 1] + intersections[index]) / 2));
}
```

If two consecutive intersections are identical, the midpoint is exactly the
intersection point. `makeCell()` assigns boundary points to the cell on the
positive side of the boundary.

That can insert cells the segment only touches at a point, or fail to insert one
of the cells touching a corner, depending on intended semantics.

The tests currently expect:

```cpp
LineSegment{{0, 0}, {7, 7}} -> {{0, 0}, {4, 4}}
```

So this implementation intentionally does not include all cells touched by a
diagonal corner contact. It includes the cells the segment passes through as
area intervals, not every cell that shares the geometric point.

That may be acceptable for broad-phase intersection if all segments use the same
cell convention. But the comment says:

```cpp
/// Creates all cells that are trouched by the linesegment
```

If interpreted literally, corner-touching cells are omitted.

Also, the typo `trouched` should be fixed if this file is touched.

**Potential Bug: `cellsFromLineSegment()` Sort Order May Be Wrong For Some
Directions**

The algorithm sorts intersection points using `Point::operator<`:

```cpp
std::sort(std::begin(intersections), std::end(intersections));
```

This is likely lexicographic by x/y.

For monotonic line segments, lexicographic order can match traversal order if x
changes monotonically in the forward direction. But if the line is steep and x/y
directions differ, lexicographic order may still produce a consistent geometric
order along the segment only because a straight segment is monotonic in x and y
between its endpoints.

However, for vertical lines, all x values are equal and sorting depends entirely
on y. For reversed vertical lines, the geometric traversal direction is
opposite, but midpoint cells are symmetric enough that order direction does not
matter.

The real problem is not ascending versus descending; midpoint cells are the same
if the order is reversed. The problem is ties and NaNs.

If duplicate points or NaNs enter the list, lexicographic sorting becomes
fragile.

A more robust implementation would compute parametric `t` values in `[0, 1]`,
sort by `t`, deduplicate, and then sample midpoints by parameter, not by
lexicographic point order.

**Potential Bug: `DistanceQueryIterator` End Comparison Is Too Weak**

The iterator equality operator only compares `_current`:

```cpp
bool operator==(const DistanceQueryIterator& other) const { return _current == other._current; }
```

It ignores `_end`, `_distance`, and `_p`.

For normal use in the returned `IteratorPair`, this is fine because both
iterators come from the same `_segments` vector and represent one query.

But as a general iterator, two iterators over different vectors could compare
equal if their underlying vector iterators compare equal in undefined or
meaningless ways. Comparing iterators from different containers is generally
undefined.

This is not a practical bug if the iterator is only used internally as intended,
but it is not a robust standalone iterator.

**Potential Bug: `DistanceQueryIterator` Declares Mutable Reference Types
Incorrectly**

The iterator uses a const backing iterator:

```cpp
using BackingIterator = typename std::vector<T>::const_iterator;
```

but declares:

```cpp
using pointer = T*;
using reference = T&;
```

and returns:

```cpp
const T& operator*() const { return *_current; }
```

The associated iterator traits should probably be:

```cpp
using pointer = const T*;
using reference = const T&;
```

The current declarations may confuse generic code that relies on iterator
traits.

This is not necessarily breaking current callers, but it is wrong in principle.

**Potential Bug: Negative Distance Queries Behave Oddly**

`LineSegmentsInDistanceTo(double distance, Point p)` accepts any distance.

If `distance < 0`, the filter:

```cpp
dist(t, _p) <= _distance
```

will normally return false for all real distances. That may be acceptable, but a
negative distance is probably invalid input and should either assert/throw or be
documented as returning empty.

If `distance` is NaN, all comparisons are false and the result is empty.

There is no validation.

**Potential Bug: `LineSegmentsInApproxDistanceTo()` Has Fixed Hidden Radius**

The approximate grid insertion uses:

```cpp
constexpr double searchRadius = 4.;
```

This is equal to `CELL_EXTEND`, but it is not parameterized and is not exposed
in the public API.

The method name says:

```cpp
LineSegmentsInApproxDistanceTo(Point p)
```

but there is no distance parameter. The approximate distance is implicitly
hardcoded to 4.

That may match operational model assumptions, but the API is vague. Callers need
to know what “approx distance” means.

This is more of a design/documentation issue than a bug.

**Potential Bug: Approximate Grid Uses AABB Intersection, Not Segment Distance**

`insertIntoApproximateGrid()` computes a bounding box around each query cell
expanded by `searchRadius`:

```cpp
const AABB bbWithSearchRadius(
    {cell.x - searchRadius, cell.y - searchRadius},
    {cell.x + searchRadius + CELL_EXTEND, cell.y + searchRadius + CELL_EXTEND});

if(bbWithSearchRadius.Intersects(ls)) {
    _approximateGrid[cell].push_back(ls);
}
```

This guarantees a broad candidate set, not an exact distance-limited set. A
segment can be included because it intersects the expanded AABB even if parts of
the cell are farther than 4 from it, and a segment can be excluded if the
intended notion is not AABB-based.

That is probably intended: it is an approximate query.

But the name can be misleading. It returns candidates, not segments proven to be
within a specific distance of the query point.

**Potential Bug: `IntersectsAny()` Depends Completely On Cell Coverage
Correctness**

`IntersectsAny()` only checks geometry segments stored in cells touched by the
query line:

```cpp
const auto cellsToQuery = cellsFromLineSegment(linesegment);
for(const auto& cell : cellsToQuery) {
    const auto iter = _grid.find(cell);
    ...
}
```

If `cellsFromLineSegment()` misses a cell for either the stored geometry segment
or the query segment, then `IntersectsAny()` can return false even when an
intersection exists.

This makes the quality of `cellsFromLineSegment()` critical.

Given the NaN/division-by-zero behavior and corner-boundary subtleties, this is
the area with the highest correctness risk.

**Potential Bug: No Test Coverage For `IntersectsAny()` Or `InsideGeometry()`**

The existing `TestCollisionGeometry.cpp` covers:

- `IsN8Adjacent()`
- `cellsFromLineSegment()`
- `LineSegmentsInApproxDistanceTo()`

It does not appear to test:

- `IntersectsAny()` positive cases
- `IntersectsAny()` negative cases
- `IntersectsAny()` near cell boundaries
- `IntersectsAny()` with vertical/horizontal/diagonal query lines
- `InsideGeometry()` with holes
- `InsideGeometry()` on boundaries
- `LineSegmentsInDistanceTo()` exact-distance filtering

That is a notable gap because `IntersectsAny()` is probably the most
behaviorally important collision function.

**Problem: Global Namespace Pollution**

This header defines several names in the global namespace:

```cpp
CollisionGeometry
dist
DistanceQueryIterator
CELL_EXTEND
Cell
IsN8Adjacent
makeCell
cellsFromLineSegment
```

This may be consistent with the existing simulator code, but from a
library-design standpoint it is not ideal.

Especially problematic are generic names like:

```cpp
dist
Cell
makeCell
```

These are easy to collide with other code.

**Problem: `std::hash<Cell>` Specialization Is Really `std::hash<Point>`**

This code says:

```cpp
using Cell = Point;

template <>
struct std::hash<Cell> {
    std::size_t operator()(const Point& pos) const noexcept
```

Because `Cell` is just a type alias, this specializes `std::hash<Point>`, not a
distinct cell type.

That means any `std::unordered_map<Point, ...>` in the program now gets this
hash behavior too.

This may be intentional or harmless, but it is surprising.

A stronger design would use a distinct type:

```cpp
struct Cell {
    int x;
    int y;
};
```

or:

```cpp
struct Cell {
    double x;
    double y;
};
```

Then `std::hash<Cell>` would only apply to grid cells.

Also, hashing doubles is generally delicate. Here cell coordinates are produced
by `floor(...)*CELL_EXTEND`, so they should be exact integer-valued doubles for
reasonable coordinate ranges. But using integral cell indices would be cleaner
and more robust.

**Problem: Cell Coordinates Are Doubles**

Cells are represented as `Point`, which stores doubles.

But cells are grid coordinates. They would be more naturally represented as
integer indices or integer world-grid coordinates.

Current code relies on floating-point equality and hashing for map keys:

```cpp
std::unordered_map<Cell, ...>
```

This is probably safe as long as cells only come from `makeCell()` and
increments by `CELL_EXTEND`, where values are small exact integers representable
by double.

But it is less robust than integer cell keys.

The risk grows if coordinates are very large, because doubles cannot exactly
represent every integer beyond `2^53`.

**Problem: `CELL_EXTEND` Is A Global `const int`**

This is exposed in the header:

```cpp
const int CELL_EXTEND = 4;
```

Since it is in a header, namespace/global pollution aside, it fixes the spatial
grid resolution globally at compile time.

That might be okay, but it means all geometries use the same grid cell size
regardless of geometry scale or operational model needs.

It also means the approximate search radius is coupled to this value indirectly
by convention, but not by code:

```cpp
constexpr double searchRadius = 4.;
```

If `CELL_EXTEND` changes, `searchRadius` will not automatically change unless
someone remembers to update it.

This should probably be:

```cpp
constexpr double searchRadius = CELL_EXTEND;
```

or a named constant shared by both.

**Problem: `_grid` Uses `std::set<LineSegment>`**

The exact grid stores:

```cpp
std::unordered_map<Cell, std::set<LineSegment>> _grid{};
```

This deduplicates line segments in each cell and provides ordered storage, but
it is relatively expensive.

Construction pays tree insertion cost for every cell/segment relation.

`IntersectsAny()` only iterates through candidates. It does not need ordering.

If duplicate insertions are not a real issue, `std::vector<LineSegment>` would
likely be faster and smaller. If deduplication is needed due to duplicate
intersection points, that points back to `cellsFromLineSegment()` needing
cleanup/deduplication before insertion.

This is not a correctness bug, but it is a performance/design tradeoff.

**Problem: Copies Line Segments Into Both Grids**

Line segments are stored by value in `_segments`, `_grid`, and
`_approximateGrid`.

That duplicates data.

Line segments are probably small, so this may be acceptable. But a cleaner
design would store line segments once in `_segments` and have grids store
indices or pointers into `_segments`.

Benefits of indices/pointers:

- less memory duplication
- no risk of different copied segment values diverging in future changes
- faster grid construction if avoiding set/tree copies
- easier to attach metadata to segments later

Downside:

- pointer/index lifetime and copy/move behavior must be handled carefully

Given geometry is immutable after construction, segment indices would be
natural.

**Problem: `CollisionGeometry` Copying May Be Expensive And ID Semantics Are
Ambiguous**

The class is default-copyable:

```cpp
CollisionGeometry(const CollisionGeometry& other) = default;
CollisionGeometry& operator=(const CollisionGeometry& other) = default;
```

That copies:

- the CGAL polygon
- all segments
- both grids
- accessible-area vectors
- the `_id`

Depending on `UniqueID` semantics, copying the ID may or may not be intended.

If copied geometries are supposed to represent the same logical geometry,
preserving `_id` is okay. If every geometry object should have a unique
identity, default copying is wrong.

The code in `Simulation::Geo()` appears to return `CollisionGeometry` by value,
so copying is part of the public API. That means this may be intentional. But it
is still potentially expensive.

**Problem: Constructor Takes `PolyWithHoles` By Value Without Moving**

The constructor signature is:

```cpp
CollisionGeometry::CollisionGeometry(PolyWithHoles accessibleArea)
    : _accessibleAreaPolygon(accessibleArea)
```

Because the parameter is by value, the member should probably be
move-constructed:

```cpp
: _accessibleAreaPolygon(std::move(accessibleArea))
```

As written, callers passing an rvalue still incur a copy from the parameter into
the member.

This is a performance issue, not a behavior bug.

However, if this is changed, subsequent constructor code should consistently use
`_accessibleAreaPolygon` rather than the moved-from parameter. The current code
uses both:

```cpp
_segments.reserve(CountLineSegments(accessibleArea));
ExtractSegmentsFromPolygon(accessibleArea.outer_boundary(), _segments);
for(const auto& hole : accessibleArea.holes()) {
```

So a move change requires a small refactor.

**Problem: `LineSegmentsInApproxDistanceTo()` Returns A Reference To Internal
Storage**

This function returns:

```cpp
const std::vector<LineSegment>&
```

When there is no grid cell, it returns a reference to a static empty vector:

```cpp
static const std::vector<LineSegment> empty{};
return empty;
```

This is efficient, but callers must not retain the reference beyond the lifetime
of the geometry unless it is the static empty vector. The const reference
prevents mutation, but lifetime still matters.

This is normal C++ API behavior, but worth documenting.

**Problem: Names And Comments Have Typos**

Examples:

```cpp
/// Cells are defined on the intervalls
/// Cells are always alligned
/// Creates all cells that are trouched
/// Do not call constructor drectly
/// Will perfrom
```

These are minor, but this is a public/internal header and the comments explain
non-trivial spatial indexing behavior. The typos reduce confidence and should be
cleaned up when touching the file.

**Potential Issue: Boundary Semantics Of `InsideGeometry()`**

`InsideGeometry()` treats boundary points as inside:

```cpp
oriented_side(...) != CGAL::ON_NEGATIVE_SIDE
```

That means `ON_POSITIVE_SIDE` and `ON_ORIENTED_BOUNDARY` both return true.

For pedestrian simulation, treating points exactly on walls as inside can be
reasonable, because numerical movement may place agents on boundaries. But if
callers use this to validate collision-free placement, boundary-as-inside may be
too permissive.

This is not necessarily a bug, but the semantics should be intentional and
tested.

**Potential Issue: Polygon Validity Assumptions**

The constructor assumes `accessibleArea` is valid and non-empty.

`ExtractSegmentsFromPolygon()` calls:

```cpp
boundary.back()
boundary.front()
```

If the polygon boundary is empty, this is invalid.

GeometryBuilder likely enforces valid polygons, but `CollisionGeometry` itself
does not.

The header comment says:

```cpp
/// Do not call constructor drectly use 'GeometryBuilder'
```

But the constructor is public and used directly in tests and Python bindings. If
the class relies on prevalidated input, that should be clearer, or validation
should happen here.

**Potential Issue: `dist()` Is Too Generic**

There is a free function:

```cpp
double dist(LineSegment l, Point p);
```

It takes the line segment by value and simply calls:

```cpp
return l.DistTo(p);
```

This should probably take `const LineSegment&`:

```cpp
double dist(const LineSegment& l, Point p);
```

Also, the name `dist` is very generic for a global function in a header.

It exists to decouple `DistanceQueryIterator` from `LineSegment`, but since
`DistanceQueryIterator` is only used for `LineSegment` here, the helper is
unnecessary or should be private/detail-scoped.

**Performance Characteristics**

Construction cost:

- O(number of polygon edges)
- plus cost of assigning each segment to touched cells
- plus cost of assigning each segment to approximate cells

Memory cost:

- `_segments` stores every wall segment once
- `_grid` stores copies of segments per exact touched cell
- `_approximateGrid` stores copies of segments per approximate candidate cell

`IntersectsAny()` expected good case:

- O(number of cells touched by query segment + candidates in those cells)

Worst case:

- very long query lines traverse many cells
- dense geometry in a cell can produce many candidate tests
- poor cell size relative to geometry scale can reduce acceleration benefits

`LineSegmentsInDistanceTo()`:

- O(total number of segments) in the worst case and usually also in practice,
  because it scans `_segments` lazily until iteration completes
- no grid acceleration

`LineSegmentsInApproxDistanceTo()`:

- O(1) average map lookup
- returns all precomputed candidates for that cell

`CELL_EXTEND = 4` is therefore a central performance/correctness tuning
parameter.

**Most Important Fixes**

If I were improving this, I would prioritize:

1. Fix `cellsFromLineSegment()` to avoid division by zero and NaN intersections
   for vertical, horizontal, and zero-length line segments.

2. Add direct tests for `IntersectsAny()`, especially vertical, horizontal,
   diagonal, negative-coordinate, boundary-aligned, and hole cases.

3. Add tests for `InsideGeometry()` with holes and boundary points.

4. Replace double-valued `Cell = Point` with a distinct integer cell/index type,
   or at least make `Cell` a real struct.

5. Change `std::hash<Cell>` so it does not accidentally specialize
   `std::hash<Point>`.

6. Make the approximate-search radius explicit and tie it to `CELL_EXTEND` if
   they are intended to be the same.

7. Fix `DistanceQueryIterator` traits to use `const T*` and `const T&`.

8. Consider storing segment indices in grids instead of copying `LineSegment`s
   into every cell.

9. Clean up comments and clarify exact versus approximate query semantics.

10. Consider moving the constructor parameter into `_accessibleAreaPolygon` to
    avoid a copy, while refactoring constructor logic to use the member
    afterward.

**Verdict**

The core design is good: `CollisionGeometry` builds static spatial acceleration
structures for wall intersection and nearby-wall candidate queries, while using
CGAL for robust polygon containment. That is an appropriate shape for pedestrian
simulation geometry.

The implementation is also covered by some useful grid tests, especially around
negative coordinates and approximate candidate lookup.

But there are real concerns:

- `cellsFromLineSegment()` can generate NaNs through division by zero.
- `IntersectsAny()` correctness depends entirely on `cellsFromLineSegment()`
  being perfect, but `IntersectsAny()` itself appears untested.
- `Cell = Point` means cells are double-valued hash keys and `std::hash<Cell>`
  is really `std::hash<Point>`.
- approximate distance behavior is hardcoded and not clearly documented.
- copy behavior may be expensive and ID semantics are ambiguous.
- the iterator traits are technically wrong for a const iterator.

The most glaring issue is the division-by-zero/NaN behavior in
`cellsFromLineSegment()`. The highest-value verification gap is the lack of
direct `IntersectsAny()` tests.
