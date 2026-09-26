# Testing an estimator that never crashes

The most transferable thing in this repo is not the LiDAR-inertial odometry — it is the
method used to find the bugs in it. Because **every serious bug here produced plausible
output** (see the [README](../README.md)): none crashed, NaN'd, or threw. A sign-flipped
Jacobian still converges; a plane fitted perpendicular to the wall still gives ICP
something to chew on. That is the defining hazard of estimator code, and it dictates
everything below.

---

## 1. The oracle principle

> **Never check a derivation against itself.**

The obvious way to test `pointToPlaneJacobian` is to re-derive it by hand and compare. This
is worthless: if your derivation was wrong, your *check* is wrong the same way. Unit tests
written by the same brain that wrote the bug inherit the bug.

An **oracle** is a second source of truth that shares *none* of the first one's
assumptions. We use three:

| Oracle | Checks | Shares nothing with |
|---|---|---|
| **Finite differences** | analytic Jacobians | the hand-derivation (it perturbs and *measures*) |
| **Brute-force integration** | preintegration | the clever frame-independent trick |
| **The real bag** | everything | all of the above |

Each is slow, approximate, or awkward — which is fine, because you run it once, offline,
against a fast exact implementation used in production. **Cheap oracle, expensive-but-exact
production path.** You get correctness *and* speed instead of trading one for the other.

## 2. Finite differences, on the manifold

The analytic Jacobian in production is closed-form and exact (for point-to-plane,
`[nᵢᵀ (qᵢ×nᵢ)ᵀ]` — derived in [5-registration.md §3.4](5-registration.md)).

The oracle perturbs the state and watches what *actually happens*:

$$
\mathbf{J}_{\text{num}}[i] \;=\; \frac{r\!\left(\boldsymbol{\xi} + h\,\mathbf{e}_i\right) - r\!\left(\boldsymbol{\xi} - h\,\mathbf{e}_i\right)}{2h}
$$

Two details that matter more than they look:

**Central differences, not forward.** The error is $O(h^2)$ rather than $O(h)$, which is
what lets us assert agreement to `1e-9` instead of squinting at `1e-4`.

**Perturb *through the retraction*, never by addition.** The state lives on a manifold, so
the perturbation must be applied the way the solver applies it —
$\mathrm{Exp}(h\,\mathbf{e}_i)$ — not by adding $h$ to a rotation matrix. In
[`test_nav_residual.cpp`](../glass_core/test/test_nav_residual.cpp) the numeric Jacobian goes through
`boxplus()`, the *same* retraction the optimizer uses. A finite-difference check that
perturbs the wrong way tests the wrong function.

**Why not use numeric Jacobians in production?** 12 extra residual evaluations per
correspondence per iteration (6 dims × central difference). The residual loop is the hot
path — thousands of correspondences × up to 30 iterations — so it would be ~12× slower, and
it carries step-size error besides. Exact and fast in production; approximate and honest in
the test.

## 3. Mutation testing — does the test have teeth?

**A test that has never failed is a test you have not verified.** Green means nothing until
you have seen it go red for the right reason.

So: deliberately break the code, and confirm the test catches it. Every significant check in
this repo was validated this way.

```
# Inject the exact bug the docs warn about:
sed -i 's/q.cross(normal)/normal.cross(q)/' include/glasslio/registration.hpp
colcon build && ./build/glasslio/test_jacobian
# => Assertion `err < 1e-7 && "analytic Jacobian disagrees with finite differences"' failed
git checkout include/glasslio/registration.hpp
```

This is also how you learn what each test is *for*. When the swapped cross-product was
injected, **both** `test_jacobian` and `test_registration` failed — but only one of them was
useful:

- `test_jacobian`: *"analytic Jacobian disagrees with finite differences"* → names the cause.
- `test_registration`: *"ICP failed to recover the transform"* → tells you something is
  broken, somewhere, in a pipeline with four candidate culprits.

**That gap is the entire argument for unit-level oracles.** Both catch the bug; only one
localises it. With just the end-to-end test you would be staring at a broken ICP wondering
whether it is the Jacobian, the association, the retraction, or your convergence threshold.

## 4. Tests that prove the *other* tests have teeth

A finite-difference check only proves the analytic Jacobian matches. It does **not** prove
your check is sensitive enough to notice if it didn't. So assert the failure explicitly:

**Negative assertions.** [`test_jacobian.cpp`](../test/test_jacobian.cpp) constructs the
*wrong* Jacobian — `(n × q)` instead of `(q × n)` — and asserts it **fails** the oracle:

```cpp
assert(err > 1e-3 && "the swapped cross product should NOT match; test 1 has no teeth");
```

**Load-bearing assertions.** Is the `J_r⁻¹` term in the IMU Jacobian actually doing work, or
would we pass with it stubbed out? [`test_nav_residual.cpp`](../glass_core/test/test_nav_residual.cpp)
replaces it with the identity matrix — the classic shortcut — and measures the damage:

```
Jr^-1 is load-bearing : with 5.5e-10, stubbed to I 2.5e-02      (7 orders of magnitude)
```

Without that check, you could ship the identity-matrix version and never know. It does not
NaN, it does not diverge — it just quietly mis-weights rotation, worst when you are rotating
hardest.

**Convergence-order assertions.** The bias *correction* is **first-order** (it keeps only the
linear term), so its residual **error** is $O(\delta b^2)$. Don't assert a magic tolerance —
assert the **order**: halve the bias offset and the error must fall ~4×.

```
bias first-order correction : err 6.88e-07, halving db -> 4.0x smaller
```

A loose tolerance passes for the wrong reasons. A convergence *rate* cannot.

**Structural assertions.** A LiDAR return carries *no information* about velocity or IMU
bias, so those Jacobian columns must be **exactly zero**:

```cpp
assert((J.block<1, 3>(0, kIdxVel).isZero()));
```

That is not pedantry — it is the observability structure of the problem, written down. If it
ever becomes non-zero, a residual is leaking into states it cannot possibly observe.

## 5. Your fixtures are also code, and they are also wrong

Two of the hardest bugs in this project were **in the tests**, and both taught more than they
cost.

**The corridor that wasn't degenerate.** The whole point of the corridor fixture is that X is
unobservable to the LiDAR. But the map and the scan both spanned ±25 m, so shifting the scan
*exposed the corridor's ends* — and an end cap is an X-facing surface. The LiDAR quietly
acquired **1.2e6 of stiffness** in the axis it was supposed to be blind to.

> **Near-degenerate is not degenerate.** If your fixture is supposed to have a null space,
> *verify the null space exists* — don't assume the geometry you drew has the property you
> intended.

**The test premise that could not be true.** "Strong geometry must overrule a lying IMU" —
except the closed room supplied ~2e5 of information in X, and a MEMS IMU over 0.5 s supplies
~2.4e5. **Evenly matched sensors produce a compromise, not a winner.** The test wasn't
failing because the code was wrong; it was failing because the scenario could not
demonstrate the claim. The fix was a scene with genuinely dominant geometry — not a weaker
solver.

> When a test fails, the third hypothesis (after "the code is wrong" and "the test is
> wrong") is **"the test is asking a question that has no right answer."**

## 6. When the fixture hides the bug

`LocalMap` kept the **first N points** per voxel. With a raster-ordered cloud, a voxel fills
from the first few scan lines, leaving a thin slab — and PCA then reports the *slab's* axis
as the surface normal, straight through the planarity gate.

This bug survived a working pipeline, a green test suite, and a real bag run. **Why?**
Because the real Livox has a *non-repetitive* scan pattern: points arrive scattered, so the
first 20 in a voxel happen to be representative. **The hardware was accidentally masking the
defect.**

It only surfaced when a *synthetic* raster-ordered fixture removed the accidental protection.

> Latent bugs masked by a benevolent input distribution are the ones that detonate when you
> change sensors. Synthetic fixtures are valuable precisely because they are **not** as kind
> as your hardware.

## 7. Unit tests do not replace driving the thing

Tight coupling passed **every** unit test — preintegration matched brute-force integration to
1e-14, every Jacobian matched finite differences, the corridor was rescued.

Then it diverged on the real bag, to +6 km.

The two bugs it hit are ones no unit test was ever going to find, because both are about how
the estimator meets *reality*:

- **Initial velocity was zero.** IMU init cannot distinguish rest from constant velocity, and
  the robot is already cruising when the bag starts. No synthetic fixture had that property,
  because we *built* the fixtures with a known initial velocity.
- **The preintegration window was wrong** (0.12 s of IMU against a 0.10 s pose delta). Every
  fixture handed the integrator a window that was correct by construction.

> Unit tests verify the math you wrote. **Only running the system verifies the math you
> needed.** Both bugs were about the interface between correct components — precisely the
> region unit tests are structurally blind to.

### The sequel, which proved the point twice over

Later, two *more* correct primitives were built to close that divergence — gravity promoted
to a state, and `x_i`'s uncertainty carried into the IMU factor
(§12.3). Both cleared the same bar: gravity's
Jacobian to `4.6e-10`, a Schur-marginalisation kernel exact to `2.1e-17`, every suite green.
The narrative was airtight — *the docs blamed exactly these two causes.*

It exploded to **~500 km** the first time it met the real bag.

That is the thesis at its purest: correct maths, green tests, a plausible story, and a system
still catastrophically wrong — caught only by a deterministic driver that *runs it on the data*.

**And then the thesis bit one level deeper.** The driver's *own* first diagnosis — "a factor,
not a filter; it needs a real sliding window" — was **itself** a plausible-but-wrong story, and
a sophisticated one (a careful, primitive-by-primitive argument). It was overturned only when
`tight_replay` was made to log the **state** (`v`, `b_a`, `g`) instead of the pose. The
fingerprint was instant: the runaway was **gravity**, wandering to magnitude `~25` because its
prior was anchored to its own moving estimate — no restoring force. Two lines (anchor to the
fixed init value, stiffen the prior) cut the divergence **~400×**. The pose said *"everything
is huge"*; the state said *"gravity, specifically."* **Instrument the quantity, not the
symptom** — and distrust even your sophisticated conclusions until the data confirms them. Full
story: [§12](#12-case-study--making-tight-coupling-work-on-the-real-bag).

Verified primitives are necessary; they are not the system. This is why `run_local.sh` and the
deterministic `tight_replay` exist, and why "all tests pass" is never the last step here.

## 8. The landmine under all of it

```cmake
target_compile_options(${t} PRIVATE -UNDEBUG)
```

Every test in this repo is `assert`-based — no framework, no fixtures, no mocking.

**Release builds define `NDEBUG`, which compiles `assert()` out entirely.** Without
`-UNDEBUG`, the suites pass in a Release build **while checking nothing at all**. They did,
for a while.

That is the same failure mode as every bug above: a green light that means nothing, and no
error anywhere to chase.

## 9. The checklist

For any non-trivial piece of estimator maths:

1. **Write the analytic version** (fast, exact, production).
2. **Check it against an independent oracle** — finite differences on the manifold, or a
   brute-force reference implementation.
3. **Mutate the code and watch the test fail**, for the *right reason*, with a message that
   names the cause.
4. **Assert the wrong version fails**, so the check provably has teeth.
5. **Assert the convergence order**, not a magic tolerance.
6. **Assert the structural zeros** — they encode observability.
7. **Verify your fixture actually has the property you claim** (the null space really is a
   null space).
8. **Then run it on real data**, because none of the above tests the interfaces.

## 10. Scoreboard — what this method actually caught

| Bug | Found by | Would it have crashed? |
|---|---|---|
| Jacobian rotation block `(n×q)` instead of `(q×n)` | finite differences | no — converges to the wrong pose |
| `kSmallAngle = 1e-8`: `1 − cos θ` cancels to zero in double | small-angle oracle check | no — silently drops the first-order term |
| `J_r⁻¹` stubbed to identity | load-bearing mutation | no — quietly overconfident |
| `LocalMap` truncating instead of sampling | synthetic raster fixture | no — fits planes perpendicular to walls |
| LiDAR residuals unweighted (σ = 1 m implied) | "lying IMU" scenario | no — IMU silently wins everything |
| Tight coupling: `v₀ = 0` | **the real bag** | no — 532/691 scans rejected, then dead-reckons away |
| Tight coupling: 0.12 s IMU window vs 0.10 s pose delta | **the real bag** | no — reads exactly like a gravity error |
| Accel bias baked into the "gravity" constant | **the real bag** + arithmetic | no — Z falls quadratically, looks like a tuning problem |
| "Fixing" it by loosening *only* the bias block | **the real bag** | no — it *converged*, and got **worse** |
| Gravity prior anchored to its own estimate (no restoring force) | **state fingerprint in `tight_replay`** | no — `g` random-walks to \|25\|, pose free-falls ~500 km |
| `lidar_sigma` = 0.05 m (LiDAR under-trusted vs its true ~2 cm noise) | **`‖v‖` fingerprint** | no — velocity ramps to ~40 m/s, tracks fine everywhere else |
| Tight on a ToF: preintegration window built from per-point time a snapshot sensor does not have | **a new sensor** (3-ToF rig) | no — every scan silently coasts; the pose freezes |

**Zero crashes. Twelve bugs. Every one of them produced plausible output** — and two of them
(the gravity anchor, the LiDAR-trust calibration) hid behind a *correct-sounding* diagnosis
until the **state**, not the pose, was logged. Together they took tight coupling from a 500 km
free-fall to parity with loose — two one-line fixes, no new architecture. The whole arc is §12.

## 11. The last one is the best one

The final row deserves its own note, because it is the only bug here that was introduced by
a **fix**.

The diagnosis was right: the accel bias was frozen (pinned ~50× harder than the data that
would move it), so it was given a carried covariance and allowed to move. That is a locally
sensible change, it compiles, it converges, and **rejections went from 266 to 579.**

The reason is that the bias was loosened while the previous state `x_i` remained *infinitely
certain* — so every error that belonged to `x_i` got shovelled into the only slack in the
system. **You cannot fix a filter by loosening one block of a factor.**

> A change that is locally correct, produces no crash, and makes the estimator **worse** is
> the purest form of the hazard this whole document is about. It is also the reason the last
> word belongs to the **real bag** and not the test suite: no unit test was ever going to
> tell you that a *more principled* bias model made things worse. Only running it did.

## 12. Case study — making tight coupling work on the real bag

Every principle above, in one feature, in the order it actually happened. The tight path
([5-registration.md §3.7](5-registration.md#37-tight-coupling--the-imu-inside-the-solve))
passed every unit test from its first day. Getting it to track the real bag took **seven
silent bugs and one confidently wrong diagnosis.** None of them crashed.

### 12.1 It diverges — and three real bugs

On the Livox bag the first tight run drifted off: Z falling quadratically, **532/691 scans
rejected**, the pose at +6 km.

**Initial velocity was zero.** IMU init cannot tell **rest** from **constant velocity**
([1-imu-init.md §3](1-imu-init.md)), and this robot is *already cruising* at ~1.5 m/s when
recording starts. Init duly reports "static" and seeds `v = 0`. Loose coupling shrugs that off —
velocity is only a prior there, and it self-corrects. Tight coupling cannot, because it holds
`x_i` at its estimate: a wrong `v_i` is treated as **certain**, and the IMU factor spends every
scan insisting the robot is stationary while the LiDAR insists it is not. **Fix:** run loose for
`tight_warmup_scans`, let ICP *measure* the velocity, then hand a correct state to the tight
solver. It recovers **1.73 m/s in +Y**, matching the 1.48–1.6 m/s derived from raw geometry.

**The preintegration window was wrong.** `meas.imu` spans ~0.12 s (a bracket sample plus guard
coverage — [2-sync.md](2-sync.md)), but consecutive poses are 0.10 s apart. Integrating the whole
group over-integrates ~20% every scan, compounding, and it reads exactly like a gravity error.
**Fix:** clip to `(prev_scan_end, scan_end]`. Rejections fell **266 → 3**.

**The accelerometer bias was baked into "gravity".** Init set the world gravity vector to the
*measured* magnitude — which is not $\mathbf{g}$ but $\mathbf{g} + \mathbf{b}_a$, two quantities
that live in different frames ([1-imu-init.md](1-imu-init.md#gravity-is-not-what-the-accelerometer-reads)).
The observed Z fall was ~380 m over a minute:

$$
a = \frac{2 \times 380}{60^2} \approx 0.21\ \text{m/s}^2
$$

— about 2% of $g$, far too large for gyro bias or numerics, and exactly the size of a cheap MEMS
accelerometer bias. **Fix:** the **standard** gravity magnitude in the world frame, and
$\mathbf{b}_a$ *estimated* as the body-frame state it actually is.

### 12.2 A principled fix that made it worse

The accel bias was also **frozen**: its prior information was
$1/(\sigma_{rw}^2\,\Delta t) \approx 10^7$ against the IMU factor's $\approx 2\times 10^5$ —
pinned ~50× harder than the data that would move it. So it was given a carried covariance,
starting loose. **Rejections went from 266 to 579.** The bias was made free while `x_i` stayed
infinitely certain, so every error that belonged to `x_i` got shovelled into the only slack in
the system. *You cannot fix a filter by loosening one block of a factor* — the whole of §11.

### 12.3 The structural diagnosis — careful, and wrong

The failures pointed at the formulation: `x_i` held fixed and infinitely certain, gravity not a
state — **"a factor, not a filter"**. So the pieces that argument called for were built, each
pinned against finite differences:

- **gravity as a state** — 18-DoF, `imuGravityJacobian` verified to `4.6e-10`;
- **`x_i`'s uncertainty carried into the IMU factor** — `Σ_eff = Σ_pre + J_i P_i J_iᵀ`, plus a
  Schur-marginalization kernel exact to `2.1e-17`;
- **the state-transition Jacobian `F`** — verified to `5.9e-09`.

Every suite was green. The narrative was airtight: *the docs blamed exactly these causes.*

| Variant | max ‖pose‖ | rejected |
|---|---|---|
| gravity-state, no `x_i` inflation | **1.9 M m** | 2575 |
| gravity-state **+** `x_i` inflation | **1.5 M m** | 2323 |

The runaway was **exponential** — `9.5 k → 367 k → 1.36 M m` — so the final number is set by how
long it runs, not by how wrong the code is (the earlier "+6 km" was a shorter run of the same
disease). The conclusion written at the time: the fixes were *necessary but not sufficient*;
the real fix is a fixed-lag window. **That conclusion was wrong.**

### 12.4 Log the state, not the pose

The break came from making `tight_replay` log the **state** — `‖v‖`, `‖b_a‖` and the gravity
vector — every scan, instead of only the pose. The fingerprint was unambiguous:

| scan | `g` (should be `[0,0,−9.8]`) | `‖v‖` | `‖b_a‖` |
|---|---|---|---|
| 0–7 | `[0, 0, −9.8]` ✅ | 0 | 0.00 |
| 199 | `[+15.8, −0.7, −4.5]` 💥 | 3.2 | 0.05 |
| 599 | `[−25.5, −3.9, −13.0]` 💥 | 5.4 | 0.05 |
| 799+ | garbage → pose free-falls | 344 → 5000 | 0.05 (fine!) |

**Gravity was the runaway** — not `x_i`, not the bias. `b_a` sat at a sensible `0.05` the whole
time; `‖v‖` exploding was a symptom. The root cause was **one line**: the gravity prior anchored
`g` to the *carried estimate* (`gravity_ = r.gravity` each scan) — a random walk with no
restoring force. Gravity is nearly unobservable over one 0.1 s scan, so the solver explained
every small pose error by tilting `g`, the anchor chased it, and it compounded. It is §12.2's
failure exactly, with gravity as the free variable instead of the bias.

**Fix:** anchor the prior to the **fixed** init gravity, and stiffen it (`gravity_sigma`
`0.5 → 0.05`). Trajectory: **527 000 m → 1 339 m**, gravity pinned, the map never lost.

### 12.5 The last drift — an under-trusted LiDAR

With gravity pinned, one drift remained: `‖v‖` ramped *smoothly* to ~40 m/s on a fast, bland
stretch and dragged the pose ~700 m off. The point-to-plane residual has **zero velocity
columns**, so the LiDAR constrains velocity only through position — and `lidar_sigma = 0.05`
declared a ~2 cm sensor to be a 5 cm one, so position was pinned too weakly for that indirect
channel to hold. **Fix:** calibrate `lidar_sigma` to `0.02`.

| | trajectory | scale-gate ratio | max `‖v‖` |
|---|---|---|---|
| Tight — original | 527 000 m | 3342× | 5000 |
| Tight — + gravity anchored to init | 1 339 m | 8.49× | 42 m/s |
| Tight — + `lidar_sigma` 0.05 → 0.02 | **429 m** | 2.72× | 7 m/s |
| Loose (the trusted baseline) | 434 m | 2.76× | — |

**Parity with loose.** Two one-line calibration fixes, **zero new architecture** — the opposite
of what §12.3's careful diagnosis prescribed.

### 12.6 A new sensor, a new silent freeze

On the 3-ToF rig ([`config/3lidars.yaml`](../config/3lidars.yaml)) tight did not diverge — it
**froze**: 0.3 m of trajectory, every scan coasting, not one warning. A ToF is a snapshot with no
per-point time, so the preintegration window, derived from per-point timestamps, collapsed to
`[0, 0]`; and because the scan-end time was only recorded *after* the empty-window guard, the
first coast left every later window stuck at zero. **Fix:** the header stamp as a snapshot's
acquisition instant, and the scan end recorded before the guard
([5-registration.md §3.9](5-registration.md#39-preintegration--why-it-exists)). Tight then ran —
and on that rig converged toward loose without beating it, because three fused fields of view
had already removed the degeneracy the IMU exists to rescue.

### 12.7 What it adds up to

> **The meta-lesson.** §12.3's diagnosis was not lazy — it was a careful, primitive-by-primitive
> argument, and it was *still* a plausible-but-wrong story. The dominant bug was not a missing
> filter; it was a prior anchored to the wrong thing. It was invisible in the pose (which just
> says "everything is huge") and obvious in the *state* (which says "gravity, specifically").
> **Instrument the quantity, not the symptom** — and distrust even your sophisticated
> conclusions until the data confirms them. This time the thesis caught the author twice: once
> in the code, once in the write-up.

The partial covariance and lack of a window remain design limits, described in
[5-registration.md §3.14](5-registration.md#current-state-limit).
