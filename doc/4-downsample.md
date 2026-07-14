# [2] Downsample

`pcl::VoxelGrid`, leaf `voxel_leaf_size` = 0.5 m, applied to the deskewed cloud.

Code: `LioEstimator::downsample()` in [`lio_estimator.cpp`](../src/lio/lio_estimator.cpp).

Measured: **~20 000 points → ~5 000** (≈25% kept).

The shortest stage in the pipeline and the one with the best effort-to-payoff ratio.
It is also more interesting than "make it smaller", because of *what* gets thrown
away and *which* cloud is thrown away from.

---

## 1. Why it pays

Registration's cost is dominated by the correspondence search, which is **linear in
the source cloud size** — one hash lookup per point, per ICP iteration:

```
cost ≈ (points in source) × (ICP iterations)
```

With up to 30 iterations, cutting the source 4× cuts the dominant term 4×. There is
no cheaper large win available: the map side is already O(1) per lookup (see
[local_map.md](6-local-map.md)), so the source side is what's left.

## 2. Why a voxel grid, and not "keep every 4th point"

Decimation (every Nth point) preserves the *sampling pattern* of the sensor, which
is not what you want. A Livox's non-repetitive scan pattern is dense in some regions
and sparse in others; decimation keeps that imbalance, so you spend correspondences
where the sensor happened to look a lot, not where the geometry is.

A voxel grid enforces **spatial uniformity**: one point per 0.5 m cell, wherever
those points came from. Each surviving point represents a distinct piece of the
world, so each contributes an *independent* constraint to the solve. That is what
the normal equations actually want.

(`pcl::VoxelGrid` returns the **centroid** of each cell, not an arbitrary member.
Slightly better than picking one: the centroid averages down the per-point range
noise.)

## 3. The leaf size is a real trade

`voxel_leaf_size` is not a magic number, and it is not "as large as you can bear."

- **Too coarse** and you erase the thin structures — poles, railings, door frames,
  table legs. Those are precisely the features that **constrain** the alignment,
  because they are the ones with geometry in more than one direction. Big flat walls
  are abundant and cheap; they constrain only their own normal. Lose the thin stuff
  and the fit goes mushy exactly where you needed it to bite.
- **Too fine** and you pay linearly for redundant points that a plane fit was going
  to average together anyway.

The failure mode of "too coarse" is the sneaky one: you don't get an error, you get a
pose that is *slightly* free to slide, and a trajectory that drifts a little more
than it should.

## 4. The subtlety: the map is fed the DENSE cloud

This one is easy to get backwards, and getting it backwards deadlocks the pipeline.

```
deskewed (dense, ~20 000) ──┬──► downsample (~5 000) ──► ICP source
                            │
                            └──────────────────────────► local map insert
```

Downsampling is an **ICP source** optimisation. The map is the **plane-fitting
target**, and plane fitting needs *density*: a voxel needs `map.min_points_for_plane` points (default 5) before PCA will
fit a plane to it ([local_map.md](6-local-map.md)).

Do the arithmetic. Feed the map the *downsampled* cloud at a 0.5 m leaf, and each
1.0 m map voxel receives roughly **one point**. Five is required. **No plane ever
forms**, ICP is permanently under-constrained, and the estimator never gets off the
ground.

So the map gets the dense cloud, and only ICP's source is decimated. The two clouds
serve different masters.

## 5. Relationship to `map.voxel_size`

These two parameters are coupled, and the constraint is one-directional:

```
map.voxel_size  >  voxel_leaf_size
   (1.0 m)          (0.5 m)
```

The map voxel must be **comfortably coarser** than the downsample leaf, for exactly
the reason in §4 — enough points must land in a map voxel to fit a plane. Set them
equal and you get the degenerate case above.

(`map.voxel_size` has a second constraint from the other direction — it must also be
≥ `max_correspondence_distance`, so the 27-cell neighbourhood search actually covers
the search radius. See [registration.md](5-registration.md).)
