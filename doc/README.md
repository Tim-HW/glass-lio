# glasslio docs

LiDAR-inertial odometry for Livox, built incrementally — and written to be **read**.
The point of this repo is the *why*: Lie algebra on SO(3)/SE(3), and least squares on a
manifold, in a system where getting them subtly wrong still produces plausible output.

## Start here

**[pipeline.md](pipeline.md)** — the spine. The diagram, the frames, the threading, and
the current status. Read it first; it links to everything else.

## The stages, in execution order

| # | Doc | What it covers |
|---|---|---|
| **1** | [1-imu-init.md](1-imu-init.md) | Static-window detection, the **units trap** (accel in *g*), gravity alignment, and why yaw is deliberately left at zero. *A gate: nothing runs until it completes.* |
| **2** | [2-sync.md](2-sync.md) | Bracketing a scan with the IMU that spans it, `scan_guard_sec`, and why consumed IMU is **not** eagerly dropped. |
| **3** | [3-deskew.md](3-deskew.md) | The deep dive: SO(3) gyro integration, SLERP between knots, the extrinsic **conjugation**, and the per-point timestamp traps. |
| **4** | [4-downsample.md](4-downsample.md) | The leaf-size trade, and why the **map** is fed the dense cloud while ICP is fed the sparse one. |
| **5** | [5-registration.md](5-registration.md) | Predict → associate → solve → accept. Point-to-plane, the Jacobian, and the constant-velocity runaway. |
| **6** | [6-local-map.md](6-local-map.md) | The voxel hash, cached planes, `floor` vs `int`, and the acceptance test that tells you the pose is right. |

**Companions** (not pipeline stages):

- **[gauss-newton.md](gauss-newton.md)** — the generic manifold solver stage 5 calls. The
  normal equations *derived*, what the Gauss-Newton approximation throws away, LDLT, Huber,
  the **retraction**, and why we run with neither damping nor line search.
- **[7-tight-coupling.md](7-tight-coupling.md)** — the IMU as a **residual in the same
  normal equations**, not merely a hint: preintegration on the manifold, the 15-DoF state,
  `J_r⁻¹`, and why the whole thing is currently **off by default**. The densest Lie-algebra
  content in the repo.
- **[testing.md](testing.md)** — **how the bugs were actually found.** Every serious defect
  in this project produced *plausible output* and none of them crashed. Finite-difference
  oracles, mutation testing, and why "all tests pass" is never the last step. Arguably the
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

The IMU is fused **loosely** by default — it predicts, then ICP solves and the IMU gets no
further vote.

**Tight coupling is built** (15-DoF joint solve, on-manifold IMU preintegration, every
Jacobian verified against finite differences) and **switched off**. It rescues an
unobservable axis on synthetic data but slowly diverges on the real bag, because `x_i` is
held infinitely certain and gravity is not a state — *a factor, not a filter*. The failure
is more instructive than the success: [7-tight-coupling.md](7-tight-coupling.md).

## Three landmines this sensor set, all of which cost real time

1. Per-point `timestamp` is **nanoseconds**, and the time is **not** in `intensity`
   (that's genuine reflectivity).
2. The IMU's `linear_acceleration` is in **g**, not m/s² — a `sensor_msgs/Imu`
   spec violation.
3. Release builds define `NDEBUG`, which **deletes every `assert()`** — the test
   suites passed while checking nothing until `-UNDEBUG` was forced.

All three produce *plausible-looking output* rather than an error. That is the defining
hazard of estimator code, and the reason each stage carries a runnable self-check.
