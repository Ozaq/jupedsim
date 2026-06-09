`libsimulator/src/NeighborhoodSearch.hpp` implements a simple uniform-grid
spatial index for agents/items with a `.pos` and `.id`.

It is conceptually fine and likely fast enough for many use cases, but there are
a few concerning correctness issues. The biggest glaring bug is negative
coordinate handling: grid indices are computed with truncation toward zero
instead of mathematical floor, which gives wrong cells for positions with
negative `x`/`y`.

**What It Does**

`NeighborhoodSearch<Value>` stores pointers to `Value` objects in a 2D hash
grid.

Core idea:

`cell index = position / cellSize`

Each grid cell maps to:

`std::vector<const Value*>`

So internally:

`std::unordered_map<Grid2DIndex, std::vector<const Value*>>`

A query checks all grid cells near the query point, then filters by exact
Euclidean distance squared.

The relevant pieces:

- `Grid2DIndex` is a pair of `int32_t` cell coordinates.
- `std::hash<Grid2DIndex>` enables use as an `unordered_map` key.
- `AddAgent()` inserts one item pointer into the appropriate cell.
- `RemoveAgent()` searches all cells and erases the item with matching `id`.
- `Update()` clears and rebuilds the whole grid from an `AgentContainer<Value>`.
- `GetNeighboringAgents()` returns copies of all agents within `radius`.

**How Lookup Works**

For a query:

`GetNeighboringAgents(pos, radius)`

it:

1. Computes the grid cell containing `pos`.
2. Computes how many neighboring cells to inspect:

`offset = ceil(radius / _cellSize)`

3. Iterates over the square of cells:

`[posIdx.idx - offset, posIdx.idx + offset] [posIdx.idy - offset, posIdx.idy + offset]`

4. For each stored item in those cells, checks:

`DistanceSquared(item->pos, pos) <= radius * radius`

5. Returns matching agents by value.

This is a standard broad-phase spatial search: cheap grid lookup first, exact
distance check second.

**What Is Good**

The overall algorithm is simple and appropriate.

For roughly uniform agent distributions, this avoids scanning every agent for
every query. If `cellSize` is close to the expected query radius or interaction
range, lookup cost is approximately proportional to local density rather than
total agent count.

The exact distance filter is correct in principle.

Even though the grid search checks a square of cells, it avoids false positives
by using `DistanceSquared(...) <= radiusSquared`.

Using squared distance is good because it avoids unnecessary `sqrt`.

The hash key type is compact.

`struct Grid2DIndex { std::int32_t idx; std::int32_t idy; };`

The code is easy to understand and has low abstraction overhead.

`Update()` rebuilds the grid from authoritative state, which is often the right
approach for dynamic simulations where many agents move every tick.

**Glaring Bug: Negative Coordinates**

This is the most serious issue:

```cpp
const int32_t idx = static_cast<int32_t>(pos.x / _cellSize);
const int32_t idy = static_cast<int32_t>(pos.y / _cellSize);
```

Casting floating-point to integer truncates toward zero in C++.

That is not the same as grid flooring.

Examples with `_cellSize = 1.0`:

```text
pos.x =  0.9  ->  0
pos.x =  1.1  ->  1
pos.x = -0.1  ->  0   // problematic
pos.x = -0.9  ->  0   // problematic
pos.x = -1.1  -> -1
```

Mathematically, grid cells usually should be:

```cpp
floor(-0.1) == -1
floor(-0.9) == -1
floor(-1.1) == -2
```

With the current code, the cell interval around zero is asymmetric:

`[-0.999..., 0.999...] -> cell 0`

while positive cells have width 1 and negative cells are shifted.

This can cause missed neighbors near negative cell boundaries.

Example with `cellSize = 1.0`, `radius = 0.2`:

- Query position: `x = -1.05`
- Neighbor position: `x = -0.95`
- Actual distance: `0.10`, so it should be found.

Current indexing:

```text
getIndex(-1.05) -> -1
getIndex(-0.95) -> 0
offset = ceil(0.2 / 1.0) -> 1
```
This one is still found because offset 1 checks both cells.

But with smaller radius and multi-dimensional edge cases, truncation makes the
cell model geometrically inconsistent. More importantly, the grid cell
assignment itself is wrong for negative coordinates and can under-cover required
cells because the query cell is not the floor cell.

The correct implementation should likely be:

```cpp
const auto idx = static_cast<std::int32_t>(std::floor(pos.x / _cellSize));
const auto idy = static_cast<std::int32_t>(std::floor(pos.y / _cellSize));
```

This is the first thing I would fix.

**Likely Bug: `RemoveAgent()` Pointer/Value Confusion**

This code looks suspicious:

```cpp
const auto iter = std::find_if(std::begin(agents), std::end(agents),
                               [item](auto& agent) { return agent.id == item.id; });
```

But `agents` is:

`std::vector<const Value*>`

So each `agent` in the lambda is a pointer, not a `Value`.

This line:

`return agent.id == item.id;`

should probably be:

`return agent->id == item.id;`

As written, this should not compile if `RemoveAgent()` is instantiated.

Because this is a class template, the method body may not be compiled until
used. So the project may build if `RemoveAgent()` is unused.

This is a glaring issue if `RemoveAgent()` is intended to work.

Also, the lambda captures `item` by value:

`[item]`

That copies the whole `Value`. It should probably capture by reference:

`[&item]`

or just the id:

`[id = item.id]`

**Potential Bug: Invalid Pointers / Lifetime Assumptions**

The grid stores raw pointers:

`std::vector<const Value*>`

That is efficient, but it assumes the referenced `Value` objects outlive the
grid entries and do not move in memory.

This is safe only if:

- the underlying `AgentContainer<Value>` has stable references, or
- `Update()` is called after every container mutation that may relocate
  elements, and
- `AddAgent()` only receives objects with stable lifetime.

If `Value` objects are stored in a `std::vector` and more agents are appended
after pointers are stored, reallocation could invalidate all pointers in the
grid.

This header does not enforce or document that requirement.

Given this is a simulation, it may be acceptable if the surrounding
architecture rebuilds the grid every tick from a stable container. But as a
standalone class, it is fragile.

**Potential Bug: No Validation Of `cellSize`**

Constructor accepts any `double`:

```cpp
explicit NeighborhoodSearch(double cellSize) : _cellSize(cellSize) {};
```
No check prevents:

`cellSize == 0 cellSize < 0 NaN inf`

If `_cellSize == 0`, this divides by zero:

`pos.x / _cellSize radius / _cellSize`

If `_cellSize < 0`, indexing and `offset` become nonsensical:

`ceil(radius / negativeCellSize)`

This can produce negative offsets and broken query bounds.

A defensive constructor should reject non-positive or non-finite values.

**Potential Bug: Radius Is Not Validated**

`GetNeighboringAgents(Point pos, double radius)` accepts any `radius`.

If `radius < 0`:

`offset = ceil(radius / _cellSize) radiusSquared = radius * radius`

A negative radius gets squared into a positive number, but `offset` may be
negative. That can make:

`xMin = posIdx.idx - offset xMax = posIdx.idx + offset`

inverted, causing no cells to be searched, even though `radiusSquared` is
positive.

A negative radius should probably either throw or be treated as invalid.

`NaN` radius is also problematic because conversion from NaN to integer is
undefined/implementation-specific territory and comparisons with `NaN` distance
thresholds are always false.

**Potential Bug: Integer Overflow / Undefined Behavior**

This conversion is risky:

`static_cast<int32_t>(pos.x / _cellSize)`

If coordinates are very large, outside `int32_t` range, converting a
floating-point value to an integer type outside representable range is undefined
behavior.

Likewise:

```cpp
const auto offset = static_cast<int32_t>(std::ceil(radius / _cellSize));
```

can overflow if `radius / _cellSize` is too large.

In normal simulation domains this may not matter, but it is still a correctness
hole.

**Potential Bug: Query Bounds Overflow**

These can overflow:

```cpp
const int32_t xMin = posIdx.idx - offset;
const int32_t xMax = posIdx.idx + offset;
```

If `posIdx.idx` is near `INT32_MIN` or `INT32_MAX`, this is signed integer
overflow, which is undefined behavior.

Again, probably unlikely in realistic geometry, but worth noting.

**Problem: `GetNeighboringAgents()` Returns Copies**

This function returns:

`std::vector<Value>`

and appends with:

`result.emplace_back(*item);`

So every neighbor is copied.

That may be fine if `Value` is small, but for agent objects this can be
expensive and semantically surprising. A neighborhood search usually returns
references, pointers, ids, or lightweight handles.

Potential downsides:

- unnecessary copying on every query
- copied agents can be mistaken for live mutable agents
- requires `Value` to be copyable
- copies may include stale state if consumers expect identity/reference
  behavior

If callers only need read-only snapshots, this is okay. But from a design
standpoint, returning `std::vector<const Value*>` or
`std::vector<std::reference_wrapper<const Value>>` would be more natural.

**Problem: `RemoveAgent()` Is O(number of agents)**

`RemoveAgent()` scans every cell:

`for(auto& [_, agents] : _grid)`

This is linear in all stored agents in the worst case.

It could compute the item’s current cell and only search there:

`auto index = getIndex(item.pos);`

But that assumes `item.pos` matches the stored position. If agents move without
updating the grid, searching all cells is more robust.

So this is a tradeoff:

- scanning all cells is slower but tolerant of stale positions
- searching only the computed cell is faster but stricter

Given the method is named `RemoveAgent`, and `Update()` exists for full
rebuilds, the current implementation may be intentionally defensive. But it is
still expensive.

**Problem: Empty Cells Are Left Behind**

When an agent is removed:

`agents.erase(iter); return;`

If the vector becomes empty, the map entry remains.

This can accumulate empty cells over time if agents are frequently
added/removed in many locations without `Update()` clearing the grid.

If `Update()` is called regularly, this is irrelevant.

A cleanup would be:

`if (agents.empty()) { _grid.erase(cellIterator); }`

But that requires iterating with a non-structured-binding iterator.

**Problem: Global `as_ptr()` Helpers Are Unused**

These helpers:

```cpp
template <typename T> T* as_ptr(T* t)

template <typename T> const T* as_ptr(const T* t)

template <typename T> T* as_ptr(T& t)

template <typename T> const T* as_ptr(const T& t)
```

are not used anywhere in this file.

They also live in the global namespace, which is not ideal for a header. Any
globally defined template helper in a widely included header increases the
chance of name collisions.

If they are unused project-wide, they should be removed. If used elsewhere
through this include, that is hidden coupling and should be made explicit
somewhere else.

**Problem: Specializing `std::hash` In A Header**

This is legal for user-defined types, so it is not inherently wrong:

`template <> struct std::hash<Grid2DIndex> {`

But it is global and affects every translation unit including this header.

Given `Grid2DIndex` itself is also global, this is acceptable but not
beautifully contained. A local hasher type inside `NeighborhoodSearch` would
reduce global namespace pollution.

Also, modern style is usually:

`namespace std { template <> struct hash<Grid2DIndex> { ... }; }`

The current syntax is accepted by compilers, but explicit specialization using
`struct std::hash<...>` at global scope is less idiomatic and can be
controversial.

**Problem: Header Pollutes Global Namespace**

Everything is in the global namespace:

`Grid2DIndex NeighborhoodSearch as_ptr`

Given this is a library project, that is not ideal.

Maybe this project uses global namespace internally in `libsimulator`, but if
not, this should be in a project namespace.

**Problem: Assumes `Value` Shape Without Concept/Documentation**

`Value` must have:

`item.pos item.id`

and `item.pos` must be a `Point`.

That is template duck typing, which is common, but there is no concept/static
assertion documenting the requirement.

This means errors can be cryptic if someone instantiates `NeighborhoodSearch`
with a wrong type.

A C++20 concept would make this clearer, though that may be more change than
needed.

**Possible Semantics Issue: Includes The Query Agent Itself**

`GetNeighboringAgents(pos, radius)` returns all agents within radius, including
any agent located exactly at `pos`.

If the caller queries neighbors for an existing agent by passing its position,
the result will include that same agent unless the caller filters it out
elsewhere.

That may be intended, but many “neighboring agents” APIs exclude self.

The method cannot exclude self because it only receives `Point`, not an agent
id. This may be fine depending on callers.

**Potential Precision Issue At Cell Boundaries**

Using floating-point division and `ceil(radius / _cellSize)` is generally fine.

However, boundary cases can over-search or under-search if floating-point error
produces an offset that is one too small. Example:

`radius / cellSize`

could theoretically compute as `0.9999999999999999` instead of `1.0`, and
`ceil(...)` still gives `1`, so that case is safe.

If it computes as `1.0000000000000002`, `ceil(...)` gives `2`, which
over-searches but remains correct.

The bigger precision risk is not here; it is the `int32_t` cast and lack of
`floor`.

**Performance Characteristics**

Expected good case:

- `Update()` is O(n)
- `AddAgent()` is expected O(1)
- `GetNeighboringAgents()` is O(number of agents in nearby cells)
- exact filtering keeps results correct

Worst case:

- all agents in one cell: query is O(n)
- huge radius: query can scan many grid cells
- very small `cellSize`: query checks many cells
- very large `cellSize`: many agents per cell

The choice of `_cellSize` is critical.

Typically you want `cellSize` around the common interaction radius. If
`cellSize` is much smaller than `radius`, the nested cell loop grows as:

`(2 * ceil(radius / cellSize) + 1)^2`

If `cellSize` is much larger than `radius`, each inspected cell contains too
many unrelated agents.

**Most Important Fixes**

If I were improving this, I would prioritize:

1. Fix `RemoveAgent()` so it actually dereferences stored pointers:

`return agent->id == item.id;`

2. Use `std::floor` for grid indexing:

`static_cast<std::int32_t>(std::floor(pos.x / _cellSize))`

3. Validate `cellSize > 0` and finite.

4. Validate `radius >= 0` and finite.

5. Consider whether the API should return pointers/references/ids instead of
copies.

6. Remove unused `as_ptr()` helpers if they are truly unused.

7. Document pointer lifetime requirements.

**Verdict**

The core design is good: a simple hashed uniform grid with exact distance
filtering. That is an appropriate and understandable neighborhood-search
structure for pedestrian simulations.

But there are real issues:

- `RemoveAgent()` appears broken if instantiated.
- negative coordinates are indexed incorrectly.
- no validation exists for invalid `cellSize` or `radius`.
- raw pointer lifetime assumptions are undocumented and potentially dangerous.
- returning neighbors by value may be expensive and semantically surprising.

The first two are the most glaring bugs.

---

Planning conversation log: append future planning notes for
`NeighborhoodSearch.hpp` here.
