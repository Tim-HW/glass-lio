# glass-lio implementation guide

LiDAR-inertial odometry for Livox, built incrementally — and written to be **read**.
The point of this repo is the *why*: Lie algebra on SO(3)/SE(3), and least squares on a
manifold, in a system where getting them subtly wrong still produces plausible output.

## Start here

For the general theory and exercises, use the [HTML estimation course](../index.html).
This Markdown guide follows the glass-lio implementation and its measured failures.

**[pipeline.md](pipeline.md)** — the spine. The diagram, the frames, the threading, and
the current status. Read it first; it links to everything else.

## The stages, in execution order

| # | Doc | What it covers |
|---|---|---|
| **1** | [1-imu-init.md](1-imu-init.md) | Static-window detection, the **units trap** (accel in *g*), gravity alignment, and why yaw is deliberately left at zero. *A gate: nothing runs until it completes.* |
| **2** | [2-sync.md](2-sync.md) | Bracketing a scan with the IMU that spans it, `scan_guard_sec`, and why consumed IMU is **not** eagerly dropped. |
| **3** | [3-deskew.md](3-deskew.md) | The deep dive: SO(3) gyro integration, SLERP between knots, the extrinsic **conjugation**, and the per-point timestamp traps. |
| **4** | [4-downsample.md](4-downsample.md) | The leaf-size trade, and why the **map** is fed the dense cloud while ICP is fed the sparse one. |
| **5** | [5-registration.md](5-registration.md) | Predict → associate → solve → accept. Point-to-plane, the Jacobian, and the constant-velocity runaway — then the **tight** path ([§3.7–3.14](5-registration.md#37-tight-coupling--the-imu-inside-the-solve)): the IMU as a residual in the same normal equations, on-manifold preintegration, the 18-DoF state, `J_r⁻¹`. The densest Lie-algebra content in the repo. |
| **6** | [6-local-map.md](6-local-map.md) | The voxel hash, cached planes, `floor` vs `int`, and the acceptance test that tells you the pose is right. |

**Companions** (not pipeline stages):

- **[gauss-newton.md](gauss-newton.md)** — the generic manifold solver stage 5 calls. The
  normal equations *derived*, what the Gauss-Newton approximation throws away, LDLT, Huber,
  the **retraction**, and why we run with neither damping nor line search.
- **[benchmark.md](benchmark.md)** — glasslio vs FAST-LIO2 vs KISS-ICP on M3DGR's
  `Outdoor01` and `Corridor02`, with real ground truth (RTK / ArUco).
- **[testing.md](testing.md)** — **how the bugs were actually found.** Every serious defect
  in this project produced *plausible output* and none of them crashed. Finite-difference
  oracles, mutation testing, and why "all tests pass" is never the last step — ending with the
  full case study of making tight coupling work on the real bag
  ([§12](testing.md#12-case-study--making-tight-coupling-work-on-the-real-bag)). Arguably the
  most transferable thing here.

## The one-paragraph version

Each Livox scan is a ~100 ms sweep, not a snapshot. We integrate the gyro to undistort it
(**deskew**), voxel-**downsample** it, **register** it against a voxel-hash **local map**
to get a pose, then insert the aligned scan back into that map. The IMU is **initialized**
from a static window first, to estimate gyro bias and align the world frame to gravity.
Output is `/glasslio_node/odom` plus a TF.

## Status

**The full pipeline works and holds real time** (10 Hz) on the test bag: it keeps up with
every scan — **zero dropped, zero diverged** — with `rmse` steady at ~0.13 m.

Registration is hand-rolled **point-to-plane ICP** over the voxel map's cached planes,
solved by Gauss-Newton on SE(3). It replaced PCL's GICP, which was ~35× too slow because
it recomputed covariances over the whole map every scan.

The IMU is fused **tightly** by default (`imu_prior_weight: 1.0`) — one joint 18-DoF solve,
LiDAR and IMU in the same normal equations.

**Tight coupling** is an 18-DoF joint solve (gravity is a state) with on-manifold IMU
preintegration, every Jacobian verified against finite differences. It rescues an
unobservable axis on synthetic data, matches loose on the original test bag (429 m vs
434 m), and beats loose on real sustained degeneracy (M3DGR's `Outdoor01`: RPE 22.6 m →
0.29 m rmse). Deskew's own gyro bias is kept in sync with the one the solve refines every
scan, gated by how well the LiDAR constrains rotation that scan. Getting there, it *first*
diverged catastrophically (~500 km), and the documented diagnosis — "a factor, not a filter;
it needs a sliding window" — was itself the *plausible-but-wrong* story: the runaway was a
miswired gravity prior, and two one-line calibration fixes closed it. How it works:
[5-registration.md §3.7](5-registration.md#37-tight-coupling--the-imu-inside-the-solve).
How it was made to work: [testing.md §12](testing.md#12-case-study--making-tight-coupling-work-on-the-real-bag).

## Three landmines this sensor set, all of which cost real time

1. Per-point `timestamp` is **nanoseconds**, and the time is **not** in `intensity`
   (that's genuine reflectivity).
2. The IMU's `linear_acceleration` is in **g**, not m/s² — a `sensor_msgs/Imu`
   spec violation.
3. Release builds define `NDEBUG`, which **deletes every `assert()`** — the test
   suites passed while checking nothing until `-UNDEBUG` was forced.

All three produce *plausible-looking output* rather than an error. That is the defining
hazard of estimator code, and the reason each stage carries a runnable self-check.
