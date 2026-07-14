# [6] Local map — the target every scan is measured against

The accumulated world geometry that registration aligns to. It is both the **output**
of the pipeline (aligned scans go in) and the **input** to it (the next scan is
registered against it), which is what makes it the most dangerous object in the
system.

Code: [`local_map.cpp`](../src/lio/local_map.cpp),
[`local_map.hpp`](../include/glasslio/local_map.hpp).
Self-check: [`test_local_map.cpp`](../test/test_local_map.cpp).

---

## 6.1 Scan-to-map, not scan-to-scan

**This single choice dominates drift.**

Scan-to-scan registration aligns each cloud to the previous one and compounds every
alignment error — a random walk with nothing pulling it back. Errors only accumulate;
nothing ever corrects them.

Scan-to-**map** anchors you against structure seen over *many* frames. The wall you
saw 50 scans ago still constrains you now. Errors don't compound in the same way,
because you keep re-measuring against the same persistent geometry.

The price is the feedback loop in §6.6, and it is a real price.

## 6.2 The data structure: a voxel hash

```cpp
std::unordered_map<VoxelKey, Voxel, VoxelKeyHash>
```

Key = the integer voxel coordinate. Value = the points in that cell, plus its cached
plane.

Hashed with the **Teschner** spatial hash — three large primes, xor-combined:

```cpp
(x·73856093) ^ (y·19349669) ^ (z·83492791)
```

Chosen because a naive hash of three small integers clusters badly (nearby voxels →
nearby buckets → collision chains). The large primes decorrelate the axes.

### `floor`, never an `int` cast

```cpp
static_cast<std::int32_t>(std::floor(p.x() / voxel_size_))
```

> `int(−0.3) == int(0.3) == 0`. A C++ cast **truncates toward zero**, so the two voxels
> straddling each axis origin **fold into one**. Everything on both sides of `x = 0`
> lands in the same cell.
>
> This is invisible until you drive backwards through the origin, at which point your
> map quietly develops a seam. `std::floor` rounds toward −∞ and gets it right.
> Pinned by [`test_local_map.cpp`](../test/test_local_map.cpp).

## 6.3 Insert — the density cap *is* the downsampling

```
insert(cloud_world):
    key = floor(p / voxel_size)
    if the bucket has room      -> append
    else                        -> RESERVOIR-SAMPLE against everything seen so far
    mark the voxel dirty
```

The scan is transformed into the world frame first: `p_world = T_wl · p_sensor`.

Two consequences of the cap (`max_points_per_voxel: 20`):

- **No separate downsampling pass over the map, ever.** Density is bounded structurally,
  by the data structure itself.
- **No re-quantization.** If you voxel-downsampled the map each frame, every stored point
  would be *nudged* a little every cycle — and registration would be chasing a target
  that is quietly moving underneath it. Here a point's **position** is never altered.

Insert is **O(1) per point**: hash, bounds-check, append or one RNG draw.

### ⚠️ The cap must SAMPLE, not TRUNCATE

The obvious policy — *"keep the first 20, drop the rest"* — is **not spatially neutral**,
and it silently manufactures fake planes.

Points arrive in whatever order the sensor emits them, so the retained subset inherits
that order's bias. Feed a **raster-ordered** cloud and a voxel fills from the first two
or three scan lines: the kept points then span the full voxel in two axes but a *sliver*
in the third.

**PCA cannot tell that sliver apart from a genuinely thin surface.** It dutifully reports
the sliver's axis as the normal — and the planarity gate **passes it**, because a thin
slab really does look planar. The voxel now holds a confident plane whose normal is
*perpendicular to the actual geometry*.

The victims are the **mixed-surface voxels** (a wall/floor corner), which `fitPlane` is
supposed to *reject* as non-planar. Truncated to a slab, they pass instead. In a corridor
test this handed the LiDAR **1.2e6 of spurious stiffness in the one axis it was supposed
to be blind to** — the estimator stopped being degenerate in a direction where it
genuinely had no information, which is the most dangerous possible way to be wrong.

**Reservoir sampling** (Vitter's Algorithm R) keeps a *uniform random* subset of every
point the voxel has ever seen. The retained points are then representative of the actual
surface, so `fitPlane`'s planarity gate can do its job: a corner looks like a corner, and
gets refused.

> The real Livox dodged this **by accident**. Its non-repetitive scan pattern delivers
> points in scattered order, so the first 20 in a voxel happened to be representative.
> Any raster-ordered sensor would have walked straight into it. Pinned by
> [`test_local_map.cpp`](../test/test_local_map.cpp).

The RNG is **deterministically seeded**: a map that reshuffles itself differently on every
run is not reproducible, and an estimator you cannot reproduce is one you cannot debug.

## 6.4 The cached plane — why registration is fast

Each voxel caches a **plane** (centroid + unit normal), fitted by PCA over its own
points.

```
cov  = (1/n) Σ (pᵢ − c)(pᵢ − c)ᵀ
normal = eigenvector of the SMALLEST eigenvalue
```

The smallest-eigenvalue eigenvector is the direction of *least* spread — which, for
points lying on a surface, is the surface normal.

### The planarity gate

Not every voxel contains a plane. A voxel full of foliage is a blob; a voxel on a
building corner is an edge. Fitting a "plane" to either yields a normal that is pure
noise — and a noisy normal is worse than no correspondence, because ICP will weight it
equally with a good one.

So two gates:

```
n  >= map.min_points_for_plane (5)      enough points to fit anything
λ₀ <= map.planarity_ratio · λ₁ (0.1)    the spread ACROSS the surface must be
                                        tiny compared with the spread ALONG it
```

Fail either and `plane_ok = false`, and the voxel is skipped during association.

**Both are config parameters**, not compiled-in constants, because the right value is
genuinely environment-dependent:

- **Tighten** `planarity_ratio` (e.g. 0.05) in vegetation or clutter — blobs are otherwise
  sneaking through the gate and injecting garbage normals.
- **Loosen** it (e.g. 0.2) in sparse indoor scenes — too few planes are forming and the
  solve is starved of constraints.

It is the knob to reach for when the `rmse` looks fine but the pose feels mushy.

### This is the ~50× over GICP

The voxel **is** the neighbourhood. So:

- correspondence search needs **no KD-tree** — hash the query point, look at the 27-cell
  neighbourhood, done ([5-registration.md §3.2](5-registration.md));
- planes are refitted **only for voxels an insert actually dirtied** — and refitted
  *lazily*, on first query. Cost is **O(changed), not O(map)**.

PCL's GICP recomputed per-point covariances over the **entire map** every time the
target changed — O(map), every scan, forever. It is built for *pairwise* alignment,
where you pay that cost once. Used scan-to-map, you pay it every frame. That was the
whole 35×, and it was **algorithmic**: no amount of parallelism would have fixed it.

## 6.5 Prune — bounding the map

```
prune(origin): erase voxels whose centre is > max_range from the pose
```

Geometry 200 m behind you cannot constrain your current alignment. Dropping it bounds
memory and keeps the hash small.

**Honest complexity:** prune is O(1) per voxel but **sweeps the whole map**, so it is
**O(map) per call** — the one O(map) step left in the per-scan path. It is cheap (a
distance compare per voxel, no PCA) and nowhere near the GICP cliff, but it is not
free, and it is the next thing to attack if the scan rate ever gets tight.

This is also what makes the system **odometry, not SLAM**: the map *forgets*. Revisit a
place and you get no loop closure, because the geometry that would have recognised it
was pruned. That is a deliberate scope choice, not an oversight.

## 6.6 What gets inserted — and the bootstrap exception

**Only a scan whose pose we trust** goes into the map (see
[5-registration.md §3.6](5-registration.md)). A wrong pose inserted here poisons the map
*permanently*, and the map is what every future scan is measured against — the error
does not fade, it compounds.

**Except while bootstrapping.** Consider the deadlock:

```
empty map  →  no voxel has ≥ 5 points  →  no planes exist
           →  ICP is under-constrained  →  registration fails
           →  scan is NOT inserted      →  the map stays empty
           →  forever
```

So until the map has supported **one** successful registration (`map_bootstrapped_`),
we dead-reckon on the IMU prior and keep feeding the map anyway. After that, the trust
rule applies strictly.

Note also that the map is fed the **dense deskewed cloud**, not the downsampled one —
see [4-downsample.md §4](4-downsample.md). Feed it the downsampled cloud and you get
~1 point per map voxel, five are needed for a plane, and you land straight back in the
deadlock above.

## 6.7 The acceptance test — how you know the pose is right

This is the most useful diagnostic in the whole system, and it needs no ground truth.

With a **correct** pose, the same physical geometry re-observed from a new viewpoint
lands in the **same voxels**. So:

- **voxel count plateaus** — you are re-observing the same world, not inventing new one;
- **points-per-voxel climbs** toward the cap.

With a **drifting** pose, the same wall gets re-inserted at slightly wrong places every
scan, each spawning fresh voxels. **Voxel count climbs without bound.**

That is exactly how the identity-pose bug was caught: **316 k voxels** instead of a
plateau. The per-scan log line reports `map N vox` for precisely this reason — watch it,
and you are watching the health of the pose.
