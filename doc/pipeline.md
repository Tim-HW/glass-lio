# The LIO pipeline

How a Livox scan becomes a pose. This is the **spine**: the shape of the pipeline, how
the stages chain, and the cross-cutting concerns (frames, threading, status). Each
stage has its own doc, linked below and named in **execution order**.

Everything here happens inside **one node**
([`glasslio_node.cpp`](../src/glasslio_node.cpp)). The intermediate clouds are never
consumed outside it, so publishing them between separate nodes would buy a
serialize/deserialize round trip and nothing else. They *are* exposed on debug topics,
but only for RViz.

```
 /livox/imu  ─┐
              ├─► [2] sync ──► [3] deskew ──► [4] downsample ──► [5] register ──► /odom + TF
 /livox/lidar ┘       ▲                                              │
                      │                                              ▼
                 [1] IMU init                              [6] insert into map
                 (gate: nothing                                      │
                  runs until done)                                   └──► local map
                                                                            │
                                                                            └─(target)─┘
```

**The loop closes on itself.** Registration needs the map; the map needs the pose that
registration produces. Stage [5] resolves it by aligning against the map built from
*previous* scans, and the first scan bootstraps it by definition. That loop is also the
system's central hazard — see [6-local-map.md §6.6](6-local-map.md).

---

## The stages, in execution order

| # | Stage | Doc | Code |
|---|---|---|---|
| **1** | **IMU initialization** — gyro bias, gravity, world frame. *A gate: nothing runs until it completes.* | [1-imu-init.md](1-imu-init.md) | [`imu_init.cpp`](../src/lio/imu_init.cpp) |
| **2** | **Sync** — pair a scan with the IMU that brackets it | [2-sync.md](2-sync.md) | `syncMeasure()` |
| **3** | **Deskew** — undo intra-scan rotation on SO(3) | [3-deskew.md](3-deskew.md) | [`data_process.cpp`](../src/lio/data_process.cpp), [`gyr_int.cpp`](../src/lio/gyr_int.cpp) |
| **4** | **Downsample** — voxel grid, 0.5 m leaf | [4-downsample.md](4-downsample.md) | `pcl::VoxelGrid` |
| **5** | **Register** — point-to-plane ICP → **the pose** | [5-registration.md](5-registration.md) | [`registration.cpp`](../src/lio/registration.cpp) |
| **6** | **Local map** — insert the aligned scan; it is the next scan's target | [6-local-map.md](6-local-map.md) | [`local_map.cpp`](../src/lio/local_map.cpp) |

**Companions** (not pipeline stages):

- [gauss-newton.md](gauss-newton.md) — the generic manifold solver stage 5 calls. The
  optimization core, deliberately split out so it knows nothing about LiDAR.
- [7-tight-coupling.md](7-tight-coupling.md) — folding the IMU into stage 5's *own* normal
  equations, instead of letting it only propose a guess. **Implemented, math verified, and
  currently OFF by default** (`imu_prior_weight: 0`) because it diverges on the real bag
  for a structural reason worth reading about.

> **Why IMU init is [1] and not somewhere later.** It is not a "setup step" you can
> reorder — it is a hard gate. Scans arriving before it completes are *dropped*, not
> buffered, because the IMU samples that would deskew them were consumed by the init
> window. Deskew cannot run without the gyro bias; the world frame cannot exist without
> gravity.

---

## Frames

- **`livox_frame`** — the sensor. Deskewed and downsampled clouds live here.
- **`odom`** — the world. **Gravity-aligned at init**, so +Z really is up. The local map
  and the odometry output live here.
- **`pose_`** — the sensor's pose in the world, `T_wl ∈ SE(3)`. This is what
  registration produces.
- **`R_il`** — the extrinsic, lidar → IMU. Genuinely identity on the Mid-360, but
  written out everywhere it belongs — see [3-deskew.md §5](3-deskew.md).

The pose is **re-orthonormalized every update**: repeated float↔double round trips
through PCL slowly erode the rotation block away from SO(3). (The solver's own updates
don't need this — composition on the manifold is exact. See
[gauss-newton.md §5](gauss-newton.md).)

---

## Threading

Registration runs on a **worker thread**, not in the subscription callbacks. It used to
run inside the IMU callback while holding the buffer mutex, which meant a slow scan
blocked sensor intake outright — turning a latency problem into **data loss**. This is
the threading that is *justified*: decoupling I/O from compute.

```
  callbacks              queue              worker
  ─────────         ─────────────         ────────
  buffer + sync  ──►  MeasureGroup  ──►  deskew → downsample → register → map
  (buf_mutex_)        (bounded, 3)        (owns the estimator)
```

The hand-off is a **bounded queue** (`max_queue_size`). If the worker falls behind, the
**oldest scan is dropped** rather than letting latency grow without bound — a stale pose
is useless. Persistent "worker behind" warnings mean registration is too slow.

### Ownership is the invariant

- The **estimator** — `pose_`, `map_`, `imu_proc_`, `velocity_`, `state_` — is touched
  **only by the worker**.
- The **sensor buffers** are touched only under `buf_mutex_`.
- The **queue is the single hand-off point.**

This is why a **bag restart** (a backwards time jump > 1 s, e.g. `run_bag.sh -l`) does
*not* reset the estimator from the callback that detects it. The worker may already have
popped a scan and be mid-`processOne`, so freeing the map under it would be a
**use-after-free** — and clearing the queue does not help, because the in-flight scan is
already out of the queue.

Instead the callback only **requests** a reset; the worker applies it to itself, in
order, between scans. The initial pose produced by IMU init is handed over the same way,
for the same reason.

> Parallelising the *math* was never the answer to anything here. The 35× GICP gap was
> **algorithmic** — see [6-local-map.md §6.4](6-local-map.md). No number of cores
> substitutes for deleting the wasted work.

---

## [7] Output

- **`/glasslio_node/odom`** — `nav_msgs/Odometry`. Pose in `odom`, twist in `livox_frame`
  (per the message spec, twist is expressed in `child_frame_id`).
- **TF** `odom → livox_frame`, broadcast by the node.
- **Debug clouds** — `~/deskewed`, `~/downsampled` (sensor frame), `~/local_map` (world).
  All skip serialization entirely when nobody is subscribed.

---

## Current status

| Stage | State |
|---|---|
| [1] IMU init | ✅ Working. Validated against bag (`\|g\|` = 9.781, tilt 7.33°). |
| [2] Sync | ✅ Working. |
| [3] Deskew | ✅ Working. 2.8° intra-scan rotation corrected. |
| [4] Downsample | ✅ Working. ~25% kept. |
| [5] Register | ✅ **Point-to-plane ICP.** Holds real time (10 Hz). |
| [6] Local map | ✅ Working. Self-checked. |

Measured on the test bag at `--rate 1`: **zero scans dropped, zero diverged**, `rmse`
steady at ~0.13 m, and the worker queue never backs up — i.e. registration sustains the
sensor's full 10 Hz. The bag is 2 772 scans / 277 s.

> The absolute *scan count* of a run is not a property of the estimator — it is just how
> long you let the bag play. What matters is the **drop rate (zero)** and that the queue
> never grows, which together say the worker is keeping pace with the sensor.

**Independent validation of the pose.** From the raw cloud alone — no registration
involved — a surface on the robot's right recedes 53.0 → 59.7 m over 4.5 s, i.e. the
platform is moving **1.48 m/s in +Y**. ICP independently reports ~1.6 m/s in +Y. Those
agree, so registration is tracking real motion.

### Performance: PCL GICP → hand-rolled point-to-plane

| | GICP (100 m map) | GICP (20 m map) | point-to-plane (100 m map) |
|---|---|---|---|
| throughput | ~0.3 Hz | 4.4 Hz | **≥ 10 Hz (real time)** |

Two orders of magnitude at the same map size. The cause, and why it was algorithmic
rather than a tuning problem: [6-local-map.md §6.4](6-local-map.md).

---

## Not implemented

- **Tight coupling** (Phase 3) — **built, verified, and switched off.** The whole 15-DoF
  joint solve exists (`R, p, v, b_g, b_a`), with on-manifold IMU preintegration, and every
  Jacobian pinned against finite differences. On a synthetic corridor it recovers the axis
  the LiDAR literally cannot see (0.40 m → 0.00 m error). On the **real bag it slowly
  diverges**, for a *structural* reason: `x_i` is held fixed and infinitely certain, and
  gravity is not a state — so a tilt error in the world frame can never be corrected, and
  the accel bias is pinned too hard to absorb it. **What we built is a factor, not a
  filter.** The fix is an 18-DoF state plus marginalisation of `x_i`. Full write-up, and
  the two real bugs found and fixed along the way, in
  [7-tight-coupling.md](7-tight-coupling.md).
- **Translational deskew** — needs a trustworthy velocity, which tight coupling would
  produce (and it would retire `use_constant_velocity` with it). See
  [3-deskew.md §7](3-deskew.md).
- **Loop closure** — this is odometry, not SLAM. The local map *forgets*, so drift is
  never corrected on revisit. Deliberate.
- The extrinsic **translation** (`extrinsic.lidar_to_imu.xyz`) is **validated but not
  stored** — deskew is
  rotation-only and nothing reads it. The config key
  (`extrinsic.lidar_to_imu.xyz`) still exists and is still checked, so a malformed
  extrinsic fails loudly at startup; the value is simply not carried around. It returns
  with translational deskew, or with a non-identity extrinsic under tight coupling.

---

## Parameters

All in [`config/livox_mid_360.yaml`](../config/livox_mid_360.yaml). The ones that
actually bite:

| Param | Why it matters |
|---|---|
| `imu.accel_in_g` | **true for Livox.** Wrong → every acceleration 9.81× off, and it fails *silently*. [1-imu-init.md §2](1-imu-init.md) |
| `registration.max_correspondence_distance` | Too small: fast motion never converges. Too large: matches the wrong wall, and re-opens the runaway. |
| `registration.use_constant_velocity` | **On.** Without it the guess has no translation at all. First thing to turn off if the pose accelerates away with a healthy `rmse`. [5-registration.md §3.5](5-registration.md) |
| `registration.max_rmse` | The coast threshold — above it we keep the prediction and refuse to insert the scan. |
| `map.voxel_size` | Must be **coarser** than `voxel_leaf_size` (a voxel needs ≥ 5 points before PCA fits a plane) **and ≥** `max_correspondence_distance` (so the 27-cell search covers the radius). |
| `map.max_range` | Memory / prune radius. No longer the speed cliff it was under GICP. |
| `max_queue_size` | Worker backlog. Persistent "worker behind" warnings mean registration is too slow. |
| `scan_guard_sec` | Must exceed the scan period. [2-sync.md §2](2-sync.md) |

---

## Running

```bash
./scripts/run_bag.sh          # node + bag + RViz, on an isolated ROS domain
./scripts/run_bag.sh -n       # headless
./scripts/run_bag.sh -l       # loop the bag (exercises the estimator reset path)
colcon test --packages-select glasslio
```

The domain isolation is not cosmetic: a nav2 stack on domain 0 floods DDS discovery,
wedges the `ros2` CLI daemon, and presents as "the node is hung".

> Tests are `assert`-based with **`-UNDEBUG` forced in CMake**. Release builds define
> `NDEBUG`, which compiles `assert()` out entirely — without that flag the suites pass
> while checking nothing at all.
