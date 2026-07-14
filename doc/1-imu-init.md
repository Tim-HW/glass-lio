# [I] IMU initialization

Estimates the **gyro bias** and the **direction of gravity** from a static window of
IMU samples, and from gravity derives the initial gravity-aligned orientation.

Code: [`imu_init.cpp`](../src/lio/imu_init.cpp), [`imu_init.hpp`](../include/glasslio/imu_init.hpp).
Self-check: [`test_imu_init.cpp`](../test/test_imu_init.cpp).

**This is a gate.** No scan is deskewed, registered, or mapped until it completes.
Scans arriving before it are *dropped*, not buffered — the IMU samples that would
deskew them have already been consumed by the init window.

---

## 1. Why it has to happen first

Two things downstream are undefined without it.

**The gyro bias `b_g`.** A MEMS gyro reports a non-zero rate while perfectly still.
Integrate that unremoved and orientation walks away linearly in time. Over one 0.1 s
scan the bias here contributes ~0.02° — negligible. But the *same* integrator feeds
the cross-scan rotation prior that registration starts from, and there it
accumulates. Bias is subtracted at the source, in `GyrInt`, so nothing downstream
has to think about it.

**The world frame.** `odom` is supposed to be gravity-aligned — +Z genuinely up.
Nothing but the accelerometer can tell us which way that is. Skip this and "up" is
wherever the sensor happened to be bolted, so flat ground renders as a slope and
every height you report is measured along a tilted axis.

---

## 2. The units trap

The Livox driver publishes `linear_acceleration` in **g**, *not* m/s² — a direct
violation of the `sensor_msgs/Imu` spec, which mandates m/s².

Measured on the test bag, `|a|` at rest = **0.997**. In SI that number would be 9.81.

Everything is scaled on ingest:

$$
\mathbf{a}_{\text{SI}} = \mathbf{a}_{\text{raw}} \times 9.80665
\qquad \text{(config: \texttt{imu.accel\_in\_g})}
$$

Get this wrong and every acceleration is **9.81× too small**. Note how quietly this
fails: deskew is gyro-only, so it is completely unaffected, and initialization still
produces a perfectly plausible *direction* for gravity (scaling a vector doesn't
rotate it). The estimator looks fine. It only detonates the moment somebody
integrates acceleration — i.e. exactly when you add tight coupling.

The node guards it explicitly: if `|g|` lands more than 1 m/s² away from 9.80665
after scaling, it logs an ERROR naming the parameter.

---

## 3. Static detection

Accumulate `N` samples (`num_samples: 200` = 1 s at 200 Hz), then require **both**:

$$
\max_i \lVert \boldsymbol{\omega}_i \rVert < \omega_{\max}
\quad (0.1\ \text{rad/s})
\qquad \Longrightarrow \quad \text{not rotating}
$$
$$
\max_{\text{axis}} \; \sigma(\mathbf{a}) < \sigma_{a,\max}
\quad (0.5\ \text{m/s}^2)
\qquad \Longrightarrow \quad \text{not translating}
$$

**The second check is not redundant**, and it is the one people leave out. A
gyro-only test passes happily while the sensor is carried in a straight line at
increasing speed — and that acceleration would be summed into the mean and reported
as part of "gravity", permanently tilting the world frame.

Measured on the bag: static windows peak at `‖ω‖ ≈ 0.04` rad/s; a turn hits 0.4. The
0.1 threshold sits in the gap.

If **either** check fails, the window is **discarded entirely** and we wait for a
quiet one (`rejected_windows()` counts these). We do not initialize from a partial
or marginal window.

> A bad init is **worse than no init**. No init blocks the pipeline loudly — you see
> "waiting for IMU initialization" and you go fix it. A bad init produces a running
> estimator that is silently, permanently wrong, with no error anywhere to chase.

### The limitation, and it bit us

Neither check can distinguish **rest** from **constant velocity**. Both give zero
angular rate and zero acceleration variance — that is what "no acceleration" *means*.

On our own test bag the robot is **already cruising at ~1.5 m/s** when the recording
starts. The check reports "static". It is not lying: constant velocity means zero
acceleration, so gravity is uncorrupted and the init is *valid*.

But we then spent real time reading a correctly-tracked 1.6 m/s trajectory as
"drift", because we'd read the log line as meaning the robot was stopped.

**Never read "static window detected" as "the robot is stationary."** It means "the
robot is not accelerating."

---

## 4. What it extracts

$$
\begin{aligned}
\mathbf{b}_g &= \frac{1}{N}\sum_i \boldsymbol{\omega}_i
&&\text{gyro bias (rad/s)} \\
\mathbf{g} &= \frac{1}{N}\sum_i \mathbf{a}_i
&&\text{gravity (m/s}^2\text{, IMU frame)} \\
\mathbf{R}_{wi} &= \mathrm{FromTwoVectors}\!\left( \hat{\mathbf{g}},\; +\mathbf{Z} \right)
&&\text{gravity alignment}
\end{aligned}
$$

### Why yaw is left at zero

`R_wi` is the **minimal** rotation carrying measured "up" onto world +Z, which is
what `Eigen::Quaterniond::FromTwoVectors` gives you.

Gravity is a single vector. It pins down two degrees of freedom — roll and pitch —
and says **nothing whatsoever** about the third. Rotate the sensor about the gravity
axis and the accelerometer reads exactly the same. **Yaw is unobservable from
gravity alone.**

So leaving yaw at zero is not a shortcut or a TODO: it is the correct response to an
unobservable quantity. Inventing a value would be worse than admitting we don't have
one. (This is why odometry frames are conventionally defined *up to* yaw, and why
recovering absolute yaw needs a magnetometer or a map.)

### Applying the extrinsic

Gravity was measured in the **IMU** frame, but `pose_` tracks the **lidar**. The
initial pose must convert:

$$
\mathbf{R}_{wl} = \mathbf{R}_{wi} \cdot \mathbf{R}_{il}
$$

Forget this and the whole trajectory is tilted by the mount angle between the two
sensors. (On the Mid-360 `R_il` is genuinely identity, so this term does nothing
*here* — see [deskew.md §5](3-deskew.md). It is still written out, because the moment
anyone runs this on an Avia or an external IMU it stops being identity, and a bug
that only appears on someone else's hardware is the worst kind.)

---

## 5. Measured on the test bag

```
gyro bias : [+0.003, −0.001, +0.003] rad/s
|g|       : 9.781 m/s²                      (vs 9.80665 standard)
mount tilt: 7.33° from vertical
windows rejected for motion: 0
```

The **7.33° tilt is real** — the sensor is genuinely not mounted level. That is
precisely the quantity `R_wi` exists to remove, and skipping the alignment would
render flat ground as a 7° slope.

`|g|` = 9.781 vs 9.80665 is a 0.26% discrepancy: accelerometer scale-factor error,
well within spec for a MEMS part, and a reminder that `accel_in_g` is a *unit*
conversion, not a calibration. A tightly-coupled estimator would estimate the accel
bias online rather than trusting this number forever.

---

## 6. Threading note

`ImuInit` lives on the **callback** side (it is fed from `imuCallback`). Its result —
the initial pose and the gyro bias — is *handed to the worker thread through the
queue*, never written directly into the estimator. See
[pipeline.md](pipeline.md#threading): the worker owns `pose_`, and a callback writing
it while a scan is in flight would be a data race.
