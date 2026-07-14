# glasslio

**A transparent LiDAR-inertial odometry for Livox — written to be read.**

ROS 2 · C++17 · Sophus · Eigen · PCL

---

Most SLAM code is written to *run*. This one is written to be **understood**. It is a
working scan-to-map LiDAR-inertial odometry — deskew, voxel-plane mapping, point-to-plane
ICP solved by Gauss-Newton on SE(3) — and every non-obvious decision in it is written down
and justified, including the ones that turned out to be wrong.

The real subject is **Lie algebra and least squares on a manifold**, taught through a system
where getting them subtly wrong still produces plausible output.

> ### The thing that makes this domain hard
>
> **Every serious bug in this project produced plausible output. Not one of them crashed.**
>
> A sign-flipped Jacobian still converges. A Jacobian that has silently lost its
> first-order term still runs. A plane fitted *perpendicular* to the actual wall still gives
> ICP something to chew on. There is no stack trace, no NaN, no red text — just a
> trajectory that is quietly, confidently wrong.
>
> That single fact dictates the architecture, the tests, and the docs. See
> **[doc/testing.md](doc/testing.md)**.

---

## Quickstart

```bash
# build
colcon build --packages-select glasslio

# run on the bundled test bag (node + RViz, isolated ROS domain)
./scripts/run_bag.sh
./scripts/run_bag.sh -n        # headless
./scripts/run_bag.sh -l        # loop the bag (exercises the estimator reset path)

# the self-checks
colcon test --packages-select glasslio
```

Output: `/glasslio_node/odom` (`nav_msgs/Odometry`) plus a TF `odom → livox_frame`.

Configuration lives in **[`config/livox_mid_360.yaml`](config/livox_mid_360.yaml)**, which
is heavily commented — the parameters that actually bite are explained where they are set,
not in a table somewhere else.

## What it does

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

A Livox scan is a **~100 ms sweep, not a snapshot**. We integrate the gyro on SO(3) to
undistort it (*deskew*), voxel-downsample it, *register* it against a voxel-hash local map
to get a pose, then insert the aligned scan back into that map. The loop closes on itself —
which is both the reason it works and its central hazard.

**Status: it holds real time (10 Hz) on the test bag — zero scans dropped, zero diverged,
`rmse` steady at ~0.13 m.**

## Documentation

The docs are the point. Start with **[doc/pipeline.md](doc/pipeline.md)** — the spine — and
follow the stages in execution order.

| | Doc | What it covers |
|---|---|---|
| **1** | [IMU init](doc/1-imu-init.md) | Static-window detection, the **units trap** (accel in *g*), gravity alignment, and why yaw is deliberately left at zero |
| **2** | [Sync](doc/2-sync.md) | Bracketing a scan with the IMU that spans it, and why consumed IMU is *not* eagerly dropped |
| **3** | [Deskew](doc/3-deskew.md) | SO(3) gyro integration, SLERP between knots, the extrinsic **conjugation**, and the per-point timestamp traps |
| **4** | [Downsample](doc/4-downsample.md) | The leaf-size trade, and why the **map** is fed the dense cloud while ICP is fed the sparse one |
| **5** | [Register](doc/5-registration.md) | Predict → associate → solve → accept. Point-to-plane, the Jacobian, and the constant-velocity runaway |
| **6** | [Local map](doc/6-local-map.md) | Voxel hash, cached planes, `floor` vs `int`, and the acceptance test that tells you the pose is right |

**Companions:**

- **[gauss-newton.md](doc/gauss-newton.md)** — the solver. The normal equations *derived*,
  what the Gauss-Newton approximation throws away, LDLT, Huber, the **retraction**, and why
  we run with neither damping nor line search.
- **[7-tight-coupling.md](doc/7-tight-coupling.md)** — the IMU as a **residual in the same
  normal equations**, not merely a hint: on-manifold preintegration, the 15-DoF state,
  `J_r⁻¹`, and why it is currently **off by default**.
- **[testing.md](doc/testing.md)** — **how the bugs were actually found.** Finite-difference
  oracles, mutation testing, and why "all tests pass" is never the last step.

## The idea worth stealing

**The optimizer does not know what a point cloud is.**

[`gauss_newton.hpp`](include/glasslio/gauss_newton.hpp) owns the *generic* half — normal
equations, robust weighting, the LDLT solve, and the retraction back onto the manifold.
[`registration.cpp`](src/lio/registration.cpp) supplies only the LiDAR-specific half:
**association** (hash the point to its voxel, take the nearest plane) and the residual.

That split is not tidiness. Swap the residual and the same solver becomes a different
estimator — and it is exactly the seam the IMU prior plugs into:

$$
\mathbf{H} =
\underbrace{\sum_i \tfrac{1}{\sigma^2}\,\mathbf{J}_i^\top \mathbf{J}_i}_{\text{LiDAR}}
\;+\;
\underbrace{\mathbf{J}_{\text{imu}}^\top \boldsymbol{\Sigma}^{-1} \mathbf{J}_{\text{imu}}}_{\text{IMU}}
$$

**That sum *is* the sensor fusion.** No filter, no blending coefficient — just Jacobians
stacked into one linear system, each weighted by how much it actually knows. Where the
geometry is degenerate (a corridor), the LiDAR term has a **null space** and the IMU is the
only thing there, so it takes over exactly where it is needed, with no mode switch.

And the punchline: **loose coupling is tight coupling with $\boldsymbol{\Sigma}^{-1} = 0$.**
The two are the same estimator with one block zeroed, which is what lets a single parameter
select between them.

## Current state

| Stage | Status |
|---|---|
| IMU init, sync, deskew, downsample | ✅ Working, self-checked |
| Register (point-to-plane ICP on SE(3)) | ✅ Working, holds real time |
| Local map (voxel hash + cached planes) | ✅ Working, self-checked |
| **Tight coupling (15-DoF, preintegration)** | ⚠️ **Built and verified — but OFF by default** |

Tight coupling passes every unit test — preintegration matches brute-force integration to
1e-14, every Jacobian is pinned against finite differences, and on a synthetic corridor it
recovers the axis the LiDAR *cannot see* (0.40 m → 0.00 m error).

**Then it diverges on the real bag**, and the reason is structural rather than a typo:
`x_i` is held **fixed and infinitely certain**, and gravity is not a state — so a tilt error
in the world frame can never be corrected, and the accel bias is pinned too hard to absorb
it. **What we built is a factor, not a filter.** The write-up is in
[7-tight-coupling.md](doc/7-tight-coupling.md), and the failure is more instructive than the
success would have been.

**Not implemented:** loop closure (this is odometry, not SLAM — the map deliberately
forgets), and translational deskew (it needs a velocity we do not yet trust).

## Three landmines in this sensor set

Each cost real debugging time, and each produced *plausible output* rather than an error:

1. **Per-point `timestamp` is nanoseconds** — and the time is **not** in `intensity` (that
   field is genuine reflectivity here). Mix the units and every lookup clamps, and deskew
   silently becomes a no-op.
2. **`linear_acceleration` is in *g*, not m/s²** — a `sensor_msgs/Imu` spec violation.
   Measured at rest: `|a|` = 0.997. Get it wrong and every acceleration is 9.81× too small.
3. **Release builds define `NDEBUG`, which deletes every `assert()`.** The `assert`-based
   suites passed *while checking nothing at all* until `-UNDEBUG` was forced in CMake.

## Layout

```
include/glasslio/     the library headers — each one is the doc for its stage
  gauss_newton.hpp      generic manifold least squares (knows nothing about LiDAR)
  so3_jacobian.hpp      the SO(3) right Jacobian — the one bit Sophus does not give you
  preintegration.hpp    on-manifold IMU preintegration (Forster)
  nav_state.hpp         the 15-DoF state and its retraction
  local_map.hpp         voxel hash + cached per-voxel planes
src/lio/              the pipeline stages, ROS-free
src/glasslio_node.cpp the ROS shell: subscriptions, threading, publishing
test/                 assert-based self-checks, no framework
doc/                  the actual product
include/sophus/       vendored (Lie group primitives)
```

## Dependencies

ROS 2 (Jazzy), Eigen 3, PCL (common / io / filters), and a vendored Sophus. No Ceres, no
GTSAM — the whole solver is under 200 lines, and you are meant to read it.

## License

MIT.
