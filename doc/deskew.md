# Deskewing (motion compensation)

Phase 1 of the LIO pipeline. Removes the *rotational* distortion a moving LiDAR
bakes into every scan, using the IMU gyroscope.

Code: [`src/lio/data_process.cpp`](../src/lio/data_process.cpp) (deskew),
[`src/lio/gyr_int.cpp`](../src/lio/gyr_int.cpp) (gyro integration),
[`src/glasslio_node.cpp`](../src/glasslio_node.cpp) (buffering / sync).

Deskew is **stage 1** of the pipeline, not a standalone node. For the whole
pipeline see [pipeline.md](pipeline.md).

---

## 1. The problem

A LiDAR scan is **not a snapshot**. A Livox at 10 Hz spends ~100 ms sweeping,
emitting points continuously. Each point is measured at a different instant, in
the sensor frame *as it was oriented at that instant*.

The driver hands you all ~20 000 points in one `PointCloud2` as if they shared a
single frame. They don't. If the sensor rotated during the sweep, that lie
smears the cloud: a straight wall bends, a pole doubles.

How bad? Distortion of a point at range `r` under intra-scan rotation `Δθ` is
roughly the arc length:

```
error ≈ r · Δθ
```

Measured on our bag during a turn: `Δθ ≈ 2.8°` per scan (`0.049 rad`). At
`r = 40 m`:

```
error ≈ 40 · 0.049 ≈ 2.0 m
```

Two metres of smear. Registration (Phase 2) cannot recover from that — it would
be matching a warped cloud against a warped map. **Deskew must come first.**

## 2. The idea

If we knew the sensor's orientation `R(t)` at every instant, we could take each
point, rotate it out of the frame it was measured in, and into one **common
reference frame**. All points then agree on a single pose, and the cloud becomes
the snapshot it pretended to be.

The gyroscope gives us exactly that: angular velocity `ω(t)`, at ~200 Hz —
20 samples per scan. Integrate it and you have `R(t)`.

> We compensate **rotation only**. Translational smear needs a velocity estimate,
> which we don't have until the registration loop exists (Phase 2). See §7.

## 3. Getting per-point time

Everything hinges on knowing *when* each point was measured. Our Livox
`PointCloud2` carries it per point (`point_step = 26`):

| field | type | meaning |
|---|---|---|
| `x, y, z` | float32 | position in the lidar frame *at time of measurement* |
| `intensity` | float32 | **reflectivity** — genuinely intensity, not time |
| `tag`, `line` | uint8 | return type, laser line index |
| `timestamp` | **float64** | **absolute acquisition time, in NANOSECONDS** |

Two traps, both of which we hit:

1. **Not `intensity`.** The original LOAM-Horizon–style code this was ported from
   packed time into the fractional part of `intensity` (`intensity = ring + dt`).
   This driver does *not*. Reading time from `intensity` there yields garbage.
2. **Nanoseconds, not seconds.** IMU header stamps are in seconds. Mixing the two
   makes the point times land ~10⁹ away from the gyro knots, every lookup clamps
   to the range endpoint, and the deskew silently becomes a **no-op**. The
   tell-tale symptom was a reported scan duration of `~1e8 s`.

Hence, in `data_process.cpp`:

```cpp
auto pt_sec = [](const LivoxPoint & p) { return p.timestamp * 1e-9; };
```

The scan's time span is then read straight off the data — we don't assume 100 ms:

```
t0 = min over points of timestamp     (scan start)
t1 = max over points of timestamp     (scan end)
```

## 4. Integrating the gyro → R(t)

`GyrInt` (in [`gyr_int.cpp`](../src/lio/gyr_int.cpp)) turns discrete gyro samples
into a continuous orientation function.

### Why SO(3) and not Euler angles

Orientation lives on the rotation group **SO(3)** — a curved manifold, not a
vector space. You cannot just add up angle increments; rotations don't commute,
and Euler angles gimbal-lock. `Sophus::SO3d` handles this properly via the
exponential map, which converts a rotation *vector* (axis × angle, an element of
the tangent space `so(3)`) into a rotation:

```
Exp : ℝ³ → SO(3)
```

### The integration

Anchor identity at scan start, then step through the IMU samples. Between two
consecutive samples we use the **trapezoidal rule** (average the two angular
velocities — second-order accurate, versus first-order for a naive forward step,
and free):

```
ω_k  = ω_raw,k − b_g                 ← gyro bias, from ImuInit (see pipeline.md)
Δθ_k = Δt · ½·(ω_k + ω_{k-1})        ← rotation vector for this step
R_k  = R_{k-1} · Exp(Δθ_k)           ← compose on the manifold
```

The bias `b_g` (~0.003 rad/s here) is negligible over one 0.1 s scan — about 0.02°
— but it is subtracted anyway, because the same integrator feeds the cross-scan
rotation prior that registration depends on, where it would accumulate.

with `R(t0) = I` by construction. This produces a set of **knots** — timestamped
orientations, one per IMU sample:

```
(t0, I), (t_1, R_1), (t_2, R_2), ... , (t_n, R_n)
```

Note `R_k` is right-multiplied: `Exp(Δθ_k)` is expressed in the *body* frame at
step `k`, so it composes on the right. Getting this side wrong inverts the
correction.

### Interpolating between knots

Points fall *between* IMU samples (20 gyro samples vs 20 000 points), so we need
`R(t)` at arbitrary `t`. `GetRotAt(t)` binary-searches the bracketing knots and
**SLERPs** (spherical linear interpolation) between them:

```cpp
ratio = (t - t_lo) / (t_hi - t_lo);
q = q_lo.slerp(ratio, q_hi);
```

SLERP walks the shortest arc on the quaternion sphere at constant angular
velocity — the geometrically correct way to blend two rotations. (Componentwise
lerp on quaternions would be both non-unit and non-uniform in angle.) Outside the
knot range it clamps rather than extrapolating.

> This is strictly better than the ported original, which computed a *single*
> rotation for the whole scan and scaled it linearly per point. That assumes
> constant angular velocity across the entire 100 ms. SLERP between real knots
> follows the actual gyro signal.

## 5. The extrinsic: IMU frame vs lidar frame

`R(t)` from the gyro is expressed in **IMU axes**. The points live in **lidar
axes**. These are two rigidly-attached but differently-oriented frames.

Let `R_il` be the rotation lidar → IMU. The *same physical rotation*, re-expressed
in lidar axes, is a change of basis (a **similarity transform** / conjugation):

```
R_L(t) = R_il⁻¹ · R_I(t) · R_il
```

Read right-to-left: take a lidar-frame vector → push it into IMU axes → apply the
rotation the gyro actually measured → pull the result back into lidar axes.

Conjugation preserves the rotation **angle** and merely re-expresses its **axis**.
So a wrong extrinsic doesn't scale your correction — it *tilts* it, correcting
about the wrong axis.

```cpp
auto lidar_rot_at = [&](double t) {
    return R_il_.inverse() * gyr_int_.GetRotAt(t) * R_il_;
  };
```

`R_il` is loaded from `extrinsic.lidar_to_imu.quat_xyzw` in the config and applied
via `ImuProcess::set_extrinsic(q, t)`.

For the **Mid-360 it is genuinely identity** — the internal IMU axes are aligned
with the lidar frame — so the expression degenerates to `R_L(t) = R_I(t)`. That is
correct for this sensor, not a placeholder we are getting away with. On an Avia or
an external IMU the conjugation does real work, and a wrong `R_il` would *tilt* the
correction rather than obviously break it — it would still look like a deskew.

(The extrinsic *translation* is parsed and stored but unused: deskew is
rotation-only.)

## 6. Compensating the points

We integrated relative to **scan start** (`R(t0) = I`), but we compensate to
**scan end** (`t1`) — the convention FAST-LIO and friends use, since the scan-end
pose is what the next scan continues from.

For a point `p_i` measured at time `t_i`:

```
p_end = R_L(t1)⁻¹ · R_L(t_i) · p_i
```

Read it in two steps:

1. `R_L(t_i) · p_i` — lift the point out of the frame it was measured in, into the
   common **scan-start** frame.
2. `R_L(t1)⁻¹ · (…)` — drop it from scan-start into the **scan-end** frame.

Composed, `R_L(t1)⁻¹ · R_L(t_i)` is precisely the rotation from *sensor-at-`t_i`*
to *sensor-at-`t1`*. Sanity check: for the last point, `t_i = t1`, the two terms
cancel to identity and the point is untouched — correct, it was already in the
reference frame. For the first point, `t_i = t0`, you get the full `R_L(t1)⁻¹`,
the largest correction. That gradient across the scan *is* the un-smearing.

`R_L(t1)⁻¹` is hoisted out of the loop — it's constant per scan, and this runs
20 000 times:

```cpp
const SO3d R_end_inv = lidar_rot_at(t1).inverse();
for (auto & pt : cloud->points) {
  const SO3d R_i = lidar_rot_at(pt_sec(pt));
  const Eigen::Vector3d pc = R_end_inv * (R_i * Eigen::Vector3d(pt.x, pt.y, pt.z));
  pt.x = pc.x(); pt.y = pc.y(); pt.z = pc.z();
}
```

## 7. What is deliberately *not* compensated

**Translation.** Full motion compensation would be an SE(3) transform:

```
p_end = T_L(t1)⁻¹ · T_L(t_i) · p_i        where T = (R, t)
```

Getting `t(t_i)` means double-integrating accelerometer data — which requires
knowing gravity's direction and the sensor's velocity, and which drifts quadratically
if either is off. Neither is available from a standalone deskew node.

This is not a large omission at typical speeds: at 5 m/s over a 100 ms scan the
sensor moves 0.5 m, and unlike rotational error (`r·Δθ`, which **grows with
range**) translational error is bounded by the displacement itself and matters
most for *near* points. Rotation is the dominant term.

Phase 2 (scan-to-map registration) produces a velocity estimate, at which point
translational deskew drops straight into the same loop.

## 8. Node plumbing

[`glasslio_node.cpp`](../src/glasslio_node.cpp) buffers both streams and pairs them.

A scan is only processed once the IMU buffer **brackets it on both sides**:

- an IMU sample **before** `t0` — needed to interpolate `ω` exactly at the anchor,
- IMU coverage **past** the scan end — otherwise `GetRotAt` would clamp for the
  tail of the scan and under-correct those points.

Since the node can't know `t1` before parsing the cloud, it waits for coverage
past `header.stamp + scan_guard_sec` (default `0.12 s`, i.e. a 100 ms scan plus
margin). Consumed IMU is *not* eagerly dropped — the samples inside a scan's window
also bracket the *next* scan's start, so only samples strictly older than the
current scan start are pruned.

Output: the deskewed cloud (scan-end frame) feeds straight into downsampling and
registration. It is also published on `~/deskewed` for inspection.

## 9. Verifying it

```bash
./scripts/run_bag.sh          # node + bag + RViz
```

The deskew log line is at DEBUG level (the per-scan INFO line now reports pose and
map state instead). Enable it with `--ros-args --log-level glasslio_node:=debug`:

```
deskew: 20064 pts, scan 0.100s, gyro rot [x,y,z] deg [0.17, -0.24, 2.68]
```

Three things to check, in order:

1. **`scan ≈ 0.100s`** — per-point timestamps are being parsed in the right units.
   A value like `1e8` means the ns→s conversion is missing (§3).
2. **`gyro rot` ≈ 0 when stationary, non-zero when turning** — the gyro window
   actually lines up with the scan. Permanently `0.00` means the IMU and point
   clocks don't overlap and every lookup is clamping.
3. **The cloud itself** — overlay `~/deskewed` against the raw `/livox/lidar` in
   RViz while turning. Straight edges (walls, door frames) should straighten.
   (The RViz config ships both; the raw layer is off by default.)

On our bag: `2.68°` of yaw within one 100 ms scan through a turn — that's the
~2 m of smear at 40 m from §1, removed.
