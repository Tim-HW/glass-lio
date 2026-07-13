# glasslio docs

LiDAR-inertial odometry for Livox, built incrementally.

- **[pipeline.md](pipeline.md)** — start here. Every stage of the pipeline, the
  math it runs, why it's built that way, and what is currently broken.
- **[deskew.md](deskew.md)** — deep dive on motion compensation: SO(3) gyro
  integration, SLERP, the extrinsic conjugation, and the per-point timestamp traps.

## The one-paragraph version

Each Livox scan is a ~100 ms sweep, not a snapshot. We integrate the gyro to
undistort it (**deskew**), voxel-**downsample** it, **register** it against a voxel-hash
**local map** to get a pose, then insert the aligned scan back into that map. The
IMU is **initialized** from a static window first, to estimate gyro bias and align
the world frame to gravity. Output is `/glasslio_node/odom` plus a TF.

## Status

Deskew, initialization, downsampling and the local map all work and are verified.
**Registration is correct but ~35× too slow** — PCL's GICP recomputes covariances
over the whole map every scan. Replacing it with point-to-plane ICP over our own
voxel map is the next step. See the status section of [pipeline.md](pipeline.md).

## Three landmines this sensor set, all of which cost real time

1. Per-point `timestamp` is **nanoseconds**, and the time is **not** in `intensity`
   (that's genuine reflectivity).
2. The IMU's `linear_acceleration` is in **g**, not m/s² — a `sensor_msgs/Imu`
   spec violation.
3. Release builds define `NDEBUG`, which **deletes every `assert()`** — the test
   suites passed while checking nothing until `-UNDEBUG` was forced.

All three produce *plausible-looking output* rather than an error. That is the
defining hazard of estimator code, and the reason each stage carries a runnable
self-check.
