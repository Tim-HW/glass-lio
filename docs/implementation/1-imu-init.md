# [I] IMU initialization

Estimates the **gyro bias** and the **direction of gravity** from a static window of
IMU samples, and from gravity derives the initial gravity-aligned orientation.

Code: [`imu_init.cpp`](../glass_core/src/imu_init.cpp), [`imu_init.hpp`](../glass_core/include/glass_core/imu_init.hpp).
Self-check: [`test_imu_init.cpp`](../glass_core/test/test_imu_init.cpp).

> **This is a gate.** No scan is deskewed, registered, or mapped until it completes.
> Scans arriving before it are *dropped*, not buffered — the IMU samples that would
> deskew them have already been consumed by the init window.

---

## 1. Why it has to happen first

Two things downstream are undefined without it.

**The gyro bias `b_g`.** A MEMS gyro reports a non-zero rate while perfectly still.
Integrate that unremoved and orientation walks away linearly in time. Over one 0.1 s
scan the bias here contributes ~0.03° — negligible. But the *same* integrator feeds
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

| Check | Threshold | Sensitive to |
|---|---|---|
| `maxᵢ ‖ωᵢ‖` | < 0.1 rad/s | rotation above the threshold |
| `max_axis σ(a)` | < 0.5 m/s² | large changes in measured specific force |

**The second check is not redundant:** it can catch changing linear acceleration
that a gyro-only test misses. It cannot detect a steady push. Constant horizontal
acceleration has low standard deviation, passes both gates, and is included in the
mean specific force used to align gravity. These are stationarity heuristics, not
proof that the platform was at rest.

Measured on the bag: static windows peak at `‖ω‖ ≈ 0.04` rad/s; a turn hits 0.4. The
0.1 threshold sits in the gap.

If **either** check fails, the window **slides forward one sample** and re-checks —
it does *not* clear the whole buffer. That distinction is load-bearing. Clearing on
failure only ever tests **non-overlapping, aligned** blocks — `[0,N)`, `[N,2N)`, … — so a
genuine 1-second rest that straddles a block boundary gets split across two windows and
rejected by both, *even though the rest is right there*. A sliding window finds it wherever
it falls. (Measured: of three handheld apartment bags on a rigid 3-ToF rig, all three
contained a valid rest window, but the old clear-on-fail logic found only one — the other
two never initialised until the window was made to slide.) `rejected_windows()` counts the
slid samples. We still never initialize from a partial or marginal window — only from a full
window that passes **both** checks.

> A bad init is **worse than no init**. No init blocks the pipeline loudly — you see
> "waiting for IMU initialization" and you go fix it. A bad init produces a running
> estimator that is silently, permanently wrong, with no error anywhere to chase.

### The limitation, and it bit us

Neither check can distinguish **rest** from **constant velocity**. Both give zero
angular rate and low accelerometer variation.

On our own test bag the robot is **already cruising at ~1.5 m/s** when the recording
starts. The check reports "static". It is not lying: constant velocity means zero
acceleration, so gravity is uncorrupted and the init is *valid*.

But we then read the "static" log line as "the robot is stopped", and spent real time
mistaking a correctly-tracked 1.6 m/s trajectory for drift.

**Never read "static window detected" as "the robot is stationary."** It means
the samples passed the rotation-rate and acceleration-variation gates. Constant
velocity and steady acceleration can both pass; only the former leaves gravity
alignment uncorrupted.

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

Gravity is a single vector: it pins down roll and pitch, but rotate the sensor about
the gravity axis and the accelerometer reads exactly the same. **Yaw is unobservable
from gravity alone.** So `R_wi` is the *minimal* rotation carrying measured "up" onto
world +Z (what `Eigen::Quaterniond::FromTwoVectors` returns), and leaving yaw at zero
is not a shortcut — it is the correct response to an unobservable quantity. Inventing a
value would be worse than admitting we don't have one.

### Applying the extrinsic

Gravity was measured in the **IMU** frame, but `pose_` tracks the **lidar**. The
initial pose must convert:

$$
\mathbf{R}_{wl} = \mathbf{R}_{wi} \cdot \mathbf{R}_{il}
$$

Forget this and the whole trajectory is tilted by the mount angle between the two
sensors. On the Mid-360 `R_il` is identity so it is a no-op *here* (see
[deskew.md §5](3-deskew.md)), but it is written out for external-IMU setups where it
isn't — a bug that only appears on someone else's hardware is the worst kind.

---

## 5. Measured on the test bag

```
gyro bias : [−0.0026, −0.0004, −0.0043] rad/s
|g|       : 9.781 m/s²                      (vs 9.80665 standard)
mount tilt: 7.33° from vertical
windows rejected for motion: 0
```

The **7.33° tilt is real**, not an error — the sensor is genuinely not mounted level,
and `R_wi` is exactly what removes it.

`|g|` = 9.781 vs 9.80665 is a 0.26% discrepancy: accelerometer scale-factor and bias
error, well within spec for a MEMS part, and a reminder that `accel_in_g` is a *unit*
conversion, not a calibration.

### Gravity is not what the accelerometer reads

At rest an accelerometer does not measure $\mathbf{g}$ — it measures $\mathbf{g} + \mathbf{b}_a$,
gravity's specific force plus its own bias. A static window sees only the sum, and the two
cannot be separated until the platform **rotates**, because they live in different frames:

| | Fixed in which frame? | When the platform turns |
|---|---|---|
| **gravity** | the **world** | nothing — it still points down |
| **accel bias** | the **body** | it rotates with the platform |

So init takes only the measured **direction** (to level the world frame, §4) and gives world
gravity the **standard** magnitude, 9.80665 m/s² — not the measured 9.781. Fold the measured
sum into a "gravity" constant instead and the two agree at init by construction, then
**diverge the moment you turn**, injecting a spurious acceleration of order
$\lVert\mathbf{b}_a\rVert$ in an arbitrary direction. It happened here: a quadratic Z fall worth
~0.21 m/s², about 2% of $g$ — exactly a cheap MEMS bias
([testing.md §12](testing.md#12-case-study--making-tight-coupling-work-on-the-real-bag)). The
tightly-coupled path then estimates $\mathbf{b}_a$ online, as the body-frame state it is
([5-registration.md §3.8](5-registration.md#38-the-state--18-dof-one-curved-block)).

---

## 6. Threading note

`ImuInit` lives on the **callback** side (it is fed from `imuCallback`). Its result —
the initial pose and the gyro bias — is *handed to the worker thread through the
queue*, never written directly into the estimator. See
[pipeline.md](pipeline.md#threading): the worker owns `pose_`, and a callback writing
it while a scan is in flight would be a data race.
