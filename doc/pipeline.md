# The LIO pipeline

How a Livox scan becomes a pose. Every stage, the math it runs, and why.

Everything below happens inside **one node** ([`glasslio_node.cpp`](../src/glasslio_node.cpp)).
The intermediate clouds are never consumed outside it, so publishing them between
separate nodes would buy a serialize/deserialize round trip and nothing else. They
*are* exposed on debug topics, but only for RViz.

```
 /livox/imu  ─┐
              ├─► [0] sync ──► [1] deskew ──► [2] downsample ──► [3] register ──► /odom + TF
 /livox/lidar ┘       ▲                                              │
                      │                                              ▼
                 [I] IMU init                               [4] insert into map
                 (gate: nothing                                      │
                  runs until done)                                   └──► local map
                                                                            │
                                                                            └─(target)─┘
```

The loop closes on itself: registration needs the map, the map needs the pose that
registration produces. Stage [3] resolves that by aligning against the map built
from *previous* scans, and the first scan bootstraps it by definition.

**Frames.** `livox_frame` is the sensor. `odom` is the world — gravity-aligned at
init, so +Z really is up. `pose_` is the sensor's pose in the world:
`T_wl ∈ SE(3)`.

---

## [I] IMU initialization — [`imu_init.cpp`](../src/lio/imu_init.cpp)

**Runs once, before anything else. No scan is processed until it completes.**

### The units trap

The Livox driver publishes `linear_acceleration` in **g**, not m/s², contrary to
the `sensor_msgs/Imu` spec. Measured on the test bag: `|a|` at rest = **0.997**
(it would be 9.81 in SI). Everything is scaled on ingest:

```
a_SI = a_raw · 9.80665            (config: imu.accel_in_g)
```

Get this wrong and every acceleration is 9.81× too small — invisible through
deskew (gyro-only), catastrophic the moment acceleration is integrated.

### Static detection

Accumulate `N` samples (200 = 1 s at 200 Hz), then check **both**:

```
max‖ω_i‖  <  max_gyro       (0.1 rad/s)   → not rotating
max_axis σ(a) < max_accel_sd (0.5 m/s²)   → not translating
```

The second check is not redundant. A gyro-only test passes happily while the
sensor is being carried in a straight line — and that motion would be folded into
"gravity".

If either check fails the window is **discarded** and we wait for a quiet one. A
bad init is worse than no init, because nothing ever reports it.

> **Known limitation — and it bit us.** Neither check can distinguish *rest* from
> *constant velocity*: both give zero rotation and zero acceleration variance.
> On our own test bag the robot is **already cruising at ~1.5 m/s** when the
> recording starts, and the check happily reports "static". We spent real time
> reading a correctly-tracked 1.6 m/s trajectory as "drift" because of it.
>
> Init is still *valid* here — constant velocity means no acceleration, so gravity
> is uncorrupted. But **never read "static window detected" as "the robot stopped".**

### What it extracts

```
b_g = (1/N) Σ ω_i                     gyro bias      (rad/s)
g   = (1/N) Σ a_i                     gravity        (m/s², IMU frame)
R_wi = FromTwoVectors( ĝ , +Z )       gravity alignment
```

`R_wi` is the *minimal* rotation taking measured "up" onto world +Z. Yaw is
**unobservable from gravity alone**, so leaving it at zero is exactly right, not
a shortcut.

The initial sensor pose applies the extrinsic, because gravity was measured in the
**IMU** frame but `pose_` tracks the **lidar**:

```
R_wl = R_wi · R_il
```

Measured on the bag: bias ≈ `[+0.003, −0.001, +0.003]` rad/s, `|g|` = 9.781 m/s²,
mount **tilted 7.33°** from vertical. That tilt is real — skip the alignment and
flat ground renders as a slope.

---

## [0] Sync — pairing a scan with the IMU that spans it

A scan is only released once the IMU buffer **brackets it on both sides**:

- an IMU sample **before** `t₀`, to interpolate ω exactly at the scan start;
- IMU coverage **past** the scan end, or the tail of the scan under-corrects.

The node can't know `t₁` before parsing the cloud, so it waits for coverage past
`header.stamp + scan_guard_sec` (0.12 s = a 100 ms scan plus margin).

Consumed IMU is **not** eagerly dropped: samples inside one scan's window also
bracket the *next* scan's start. Only samples strictly older than the current scan
start are pruned.

---

## [1] Deskew — [`data_process.cpp`](../src/lio/data_process.cpp), [`gyr_int.cpp`](../src/lio/gyr_int.cpp)

Full derivation in **[deskew.md](deskew.md)**. In brief:

A scan is not a snapshot — it's a ~100 ms sweep, and each point is measured with
the sensor at a different orientation. Uncorrected, error at range `r` is `≈ r·Δθ`;
on this bag `Δθ` hits 2.8° within one scan, which is **~2 m of smear at 40 m**.

Per-point time comes from the `timestamp` field (absolute **nanoseconds** — not
from `intensity`, which is genuine reflectivity here).

Integrate the bias-corrected gyro into orientation knots on **SO(3)**:

```
Δθ_k = Δt · ½(ω_k + ω_{k−1}) − b_g·Δt        trapezoidal, in the tangent space so(3)
R_k  = R_{k−1} · Exp(Δθ_k)                   compose ON the manifold, never add
```

Rotations don't commute and don't live in a vector space, so you cannot sum angles.
`Exp: ℝ³ → SO(3)` is the bridge: do linear things in `so(3)`, compositional things
in `SO(3)`. Composition is closed, so `R_k` is exactly a rotation — no
re-orthonormalization.

Between knots (20 gyro samples vs 20 000 points), **SLERP** along the geodesic —
constant angular velocity, always on the manifold.

Express in the lidar frame via the extrinsic (a change of basis / conjugation):

```
R_L(t) = R_il⁻¹ · R_I(t) · R_il
```

Then compensate every point into the **scan-end** frame:

```
p_end = R_L(t₁)⁻¹ · R_L(t_i) · p_i
```

Sanity check: at `t_i = t₁` the terms cancel to identity — the last point doesn't
move, which is correct, it already *is* the reference.

**Rotation only.** Translational deskew needs a velocity estimate we don't trust
yet (see [3]).

Output drops to `pcl::PointXYZI`: once deskew has consumed the per-point time,
`timestamp`/`tag`/`line` are dead weight, and the plain type unlocks PCL's
precompiled filters.

---

## [2] Downsample

`pcl::VoxelGrid`, leaf 0.5 m. Keeps ~22–33% of points (20 000 → ~5–6 000).

Registration does nearest-neighbour work proportional to the source size, ~10×
per scan (once per ICP iteration). This is the cheapest large win available.

The leaf size is a genuine trade, not a magic number: too coarse erases the thin
structures (poles, railings, door frames) that *constrain* the alignment, and the
fit goes mushy precisely where you need it.

---

## [3] Register — the pose

### Predict

ICP is a **local** optimizer with a small basin of convergence. Hand it a guess a
few degrees off and it will lock onto the wrong wall and report a confident,
low-error, completely wrong fit. The prior's whole job is keeping the guess inside
the basin.

```
R_guess = R_pose · ΔR_imu        ΔR_imu = gyro rotation across the scan
t_guess = t_pose                 (+ v·Δt only if use_constant_velocity)
```

Rotation comes from the gyro and is accurate. **Translation has no prior by
default** — see the runaway below.

### Associate — nearest plane, not nearest point

ICP breaks a chicken-and-egg (you need the pose to find correspondences, and the
correspondences to find the pose) by **alternating**, guessing correspondences from
the current pose:

```
repeat (max 30):
    1. ASSOCIATE  for each scan point p:  q = T·p, find the nearest map plane to q
    2. SOLVE      T ← argmin Σ (point-to-plane residual)²
```

Correspondence is a **hash lookup, not a KD-tree query**. The map caches one plane
per voxel, so we hash `q` to its voxel, scan the 27-cell neighbourhood, and take
the nearest valid plane within `max_correspondence_distance`. Constant work per
point.

### Solve — point-to-plane, Gauss-Newton on SE(3)

**Point-to-plane, not point-to-point.** A LiDAR never re-samples the same physical
points; it hits the same *surface* in different spots. Forcing point A onto point B
when both are arbitrary samples of one flat wall injects error. Penalising only the
distance **along the normal** lets points slide freely **across** the surface —
which is exactly what a wall does and does not constrain.

Residual for correspondence `i`, with plane `(c_i, n_i)`:

```
r_i(T) = n_iᵀ (T·p_i − c_i)          signed distance to the plane
```

Linearise about the current `T` with a **left perturbation** `ξ = [ρ; φ] ∈ se(3)`
(`T ← Exp(ξ)·T`). Using `Exp(ξ)·q ≈ q + ρ + φ × q`:

```
r(ξ) ≈ r + n·ρ + n·(φ × q)
     = r + nᵀρ + (q × n)ᵀφ
```

so the 1×6 Jacobian is

```
J_i = [ n_iᵀ , (q_i × n_i)ᵀ ]
```

Stack into the normal equations and solve the 6×6 system (LDLT — `H` is symmetric
PSD), then step **on the manifold**:

```
H = Σ w_i J_iᵀ J_i ,   b = −Σ w_i J_iᵀ r_i
ξ = H⁻¹ b
T ← Exp(ξ) · T
```

`w_i` is a **Huber** weight: quadratic near zero, linear in the tail
(`w = δ/|r|` for `|r| > δ`). Moving objects and newly-seen geometry produce a few
large residuals, and without robust weighting those squared terms would dominate
the whole solve.

Converged when `‖ρ‖ < eps_translation` and `‖φ‖ < eps_rotation`.

### Accept or coast

```
if (!converged || rmse > max_rmse)  →  keep the prediction, flag DIVERGED
if (correspondences < min_correspondences) → refuse; under-constrained
```

On failure we **coast on the prediction** rather than accept a bad alignment: a
wrong pose gets inserted into the map and poisons it permanently, whereas a
slightly stale pose recovers on the next scan.

### ⚠️ The constant-velocity runaway (why the prior is OFF)

Enabling `use_constant_velocity` made the pose accelerate to ~26 m/s and end 133 m
from the start — **while the fit still looked healthy**. The feedback loop:

```
guess is too far ahead
   → correspondence distance (1.0 m) is loose enough that ICP "converges" near it
   → that mis-registered scan is INSERTED INTO THE MAP
   → the map itself drifts
   → next scan aligns to the drifted map, v grows
   → runaway
```

The map closing the loop is what makes this vicious: the estimator and its
reference drift *together*, staying self-consistent, so **no residual complains**.
A low error is not evidence against it. Only re-enable once the correspondence
distance is tight enough to *reject* a bad guess.

---

## [4] Local map — [`local_map.cpp`](../src/lio/local_map.cpp)

The accumulated world geometry that each scan is registered against.

**Scan-to-map, not scan-to-scan.** Scan-to-scan compounds every alignment error
like a random walk with nothing to pull it back. A map anchors you against
structure seen over many frames — the wall from 50 scans ago still constrains you.
This single choice dominates drift.

**Voxel hash**: `unordered_map<VoxelKey, vector<Vector3f>>`, spatial hash
(Teschner: three large primes, xor-combined).

```
insert(cloud_world) : key = floor(p / voxel_size); append if bucket < max_points_per_voxel
prune(origin)       : erase voxels whose centre is > max_range from the pose
closestPlane(p)     : nearest cached plane in the 27-cell neighbourhood -- O(1)
target()            : flatten to a cloud for RViz; cached, rebuilt only when dirty
```

The scan is transformed into the world frame first: `p_world = T_wl · p_sensor`.

Three properties fall out of the choice of key:

- The **density cap *is* the downsampling** — no separate filter pass over the map.
- Each voxel caches a **plane** (PCA over its own points): the voxel *is* the
  neighbourhood, so registration needs no KD-tree, and planes are refitted only
  where an insert dirtied them — **O(changed), not O(map)**. This is the ~50×.
- **No re-quantization.** Voxel-downsampling the map each frame would nudge
  already-stored points every cycle, so registration would chase a target that is
  quietly moving. Here a point's fate is decided once, at insert.
- Insert and prune are **O(1)** per point/voxel.

> `floor`, **never** an `int` cast: `int(−0.3) == int(0.3) == 0` folds the two
> voxels straddling each axis origin into one. Invisible until you drive backwards.
> Pinned by [`test_local_map.cpp`](../test/test_local_map.cpp).

### The acceptance test

With a correct pose, **voxel count plateaus** (the same geometry re-observed lands
in the same voxels) while points-per-voxel climbs toward the cap. Voxel count
climbing without bound means the same wall is being re-inserted at slightly wrong
places — i.e. the pose is drifting. That is exactly how the identity-pose bug was
diagnosed: 316 k voxels instead of a plateau.

---

## [5] Output

- `/glasslio_node/odom` — `nav_msgs/Odometry`, pose in `odom`, twist in `livox_frame`.
- **TF** `odom → livox_frame`, broadcast by the node.
- Debug clouds: `~/deskewed`, `~/downsampled` (sensor frame), `~/local_map` (world).
  All skip serialization when nobody is subscribed.

The pose is re-orthonormalized every update — repeated float↔double round trips
through PCL slowly erode the rotation block away from SO(3).

---

## Current status & known problems

| Stage | State |
|---|---|
| IMU init | ✅ Working. Validated against bag (|g| = 9.781, tilt 7.33°). |
| Sync | ✅ Working. |
| Deskew | ✅ Working. 2.8° intra-scan rotation corrected. |
| Downsample | ✅ Working. ~25% kept. |
| Register | ✅ **Point-to-plane ICP.** 8.6 Hz at a 100 m map (need 10). |
| Local map | ✅ Working. Self-checked. |

**Independent validation of the pose.** From the raw cloud alone — no registration
involved — a surface on the robot's right recedes 53.0 → 59.7 m over 4.5 s, i.e.
the platform is moving **1.48 m/s in +Y**. ICP independently reports ~1.6 m/s in
+Y. Those agree, so registration is tracking real motion.

### Performance: PCL GICP → hand-rolled point-to-plane

| | GICP (100 m map) | GICP (20 m map) | point-to-plane (100 m map) |
|---|---|---|---|
| throughput | ~0.3 Hz | 4.4 Hz | **8.6 Hz** |

**~50× at the same map size.** PCL's GICP recomputed per-point covariances over the
*entire* map every time the target changed — O(map), every scan, forever. It is
built for *pairwise* alignment, where you pay that once. Caching one plane per
voxel and refitting only the voxels an insert dirtied makes the cost **O(changed)**.

Still ~15% short of 10 Hz, and 52/474 scans diverge. Next levers: cap ICP
iterations, tighten `voxel_leaf_size`, or move registration off the callback thread
(below).

### 🚧 Concurrency defect

`tryProcess()` — and therefore the whole registration — runs **inside the IMU
callback while holding `buf_mutex_`**, on a single-threaded executor. A slow scan
blocks sensor intake entirely, converting a latency problem into data loss. Fix:
move registration to a worker thread consuming the synced-measurement queue.

This is the threading that is *justified* (decoupling I/O from compute).
Parallelizing the math was never the answer to the 35× — that gap was algorithmic,
and no number of cores substitutes for deleting the wasted work.

### Not implemented

- **Translational deskew** — needs a trustworthy velocity.
- **Tight coupling** (Phase 3) — IMU and LiDAR are still fused loosely: register,
  then use IMU only as a prior. An iterated-EKF would fuse them in one estimator.
- **Loop closure** — this is odometry, not SLAM. The local map forgets, so drift is
  never corrected on revisit. Deliberate.
- The extrinsic **translation** `t_il` is parsed and stored but unused (deskew is
  rotation-only).

---

## Parameters

All in [`config/livox_mid_360.yaml`](../config/livox_mid_360.yaml). The ones that
actually bite:

| Param | Why it matters |
|---|---|
| `imu.accel_in_g` | **true for Livox.** Wrong → every acceleration 9.81× off. |
| `registration.max_correspondence_distance` | Too small: fast motion never converges. Too large: matches the wrong wall, and enables the runaway. |
| `registration.use_constant_velocity` | **Keep false** until the above is tight. See the runaway. |
| `map.max_range` | Memory / prune radius. No longer the speed cliff it was under GICP. |
| `map.voxel_size` | Sets plane resolution. Should be >= `max_correspondence_distance` so the 27-cell search covers the radius. |
| `scan_guard_sec` | Must exceed the scan period. |

## Running

```bash
./scripts/run_bag.sh          # node + bag + RViz, on an isolated ROS domain
./scripts/run_bag.sh -n       # headless
colcon test --packages-select glasslio
```

The domain isolation is not cosmetic: a nav2 stack on domain 0 floods DDS
discovery, wedges the `ros2` CLI daemon, and presents as "the node is hung".

> Tests are `assert`-based with `-UNDEBUG` forced in CMake. **Release builds define
> `NDEBUG`, which compiles `assert()` out entirely** — without that flag both suites
> pass while checking nothing.
