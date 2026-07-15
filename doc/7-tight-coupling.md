# [7] Tight coupling — the IMU as a residual, not a hint

**Status: implemented, math verified, and OFF by default (`imu_prior_weight: 0`).**
It works on synthetic data and slowly diverges on the real bag. Both facts are
instructive, and §7.8 explains exactly why.

Code: [`preintegration.hpp`](../glass_core/include/glass_core/preintegration.hpp),
[`nav_state.hpp`](../glass_core/include/glass_core/nav_state.hpp),
[`nav_residual.hpp`](../glass_core/include/glass_core/nav_residual.hpp),
[`tight_registration.cpp`](../src/lio/tight_registration.cpp),
[`so3_jacobian.hpp`](../glass_core/include/glass_core/so3_jacobian.hpp).
Tests: [`test_preintegration.cpp`](../glass_core/test/test_preintegration.cpp),
[`test_nav_residual.cpp`](../glass_core/test/test_nav_residual.cpp),
[`test_tight.cpp`](../test/test_tight.cpp).

---

## 7.1 What loose coupling actually costs

In the loose pipeline the IMU produces `guess`, and then ICP is free to ignore it.
Two consequences:

**Degenerate geometry has no fallback.** Point-to-plane in a long corridor constrains
you *across* the walls but not *along* them: that direction is a genuine **null space**
of `Σ JᵀJ`. ICP will slide along it, and the LDLT solve will not complain, because the
residuals honestly do not care. The IMU *knows* you are not accelerating. It has no vote.

**`use_constant_velocity` is a symptom.** You carry an accelerometer — a device that
measures the thing loose coupling *guesses* by finite-differencing consecutive poses.
Velocity is derived from the very poses it is meant to help produce. That is a feedback
loop by construction, and it is what made the runaway possible.

## 7.2 The one idea

**Tight coupling is not a new algorithm. It is one more residual block in the same
normal equations.**

**Loose** — LiDAR only:

$$
J(\mathbf{T}) = \sum_i \rho\!\left( \mathbf{n}_i^\top(\mathbf{T}\mathbf{p}_i - \mathbf{c}_i) \right)^2
$$

**Tight** — the IMU enters as a *prior residual on the same state*:

$$
J(\mathbf{x}) =
\underbrace{\lVert \mathbf{x} \boxminus \hat{\mathbf{x}} \rVert^2_{\boldsymbol{\Sigma}^{-1}}}_{\text{IMU}}
\;+\;
\underbrace{\sum_i \rho\!\left( r_i(\mathbf{x})^2 \right) / \sigma^2}_{\text{LiDAR}}
$$

which assembles into

$$
\mathbf{H} = \mathbf{J}_{\text{imu}}^\top \boldsymbol{\Sigma}^{-1} \mathbf{J}_{\text{imu}}
\;+\; \sum_i \frac{1}{\sigma^2}\,\mathbf{J}_i^\top \mathbf{J}_i
$$

**That sum *is* the fusion.** There is no filter, no blending coefficient, no mode
switch. Where the LiDAR is well-conditioned its term dominates. Where it is degenerate
(the corridor) its term contributes *nothing* in that direction, and `Σ⁻¹` is the only
thing there — so the IMU takes over precisely where it is needed. The information
matrices arbitrate, per-direction, per-iteration.

> **Loose coupling is this with `Σ⁻¹ = 0`.** Zero the IMU information and `H` collapses
> back to `Σ JᵀJ` — literally the 6-DoF solve. That is not an analogy; it is the same
> matrix with one block zeroed, and it is why `imu_prior_weight` can select between them.

## 7.3 The state grows to 15 DoF

$$
\mathbf{x} = \left( \mathbf{R},\; \mathbf{p},\; \mathbf{v},\; \mathbf{b}_g,\; \mathbf{b}_a \right)
$$

A quantity belongs in the state if a residual mentions it and we do not already know it.

- **`v` — velocity.** The IMU's `Δv` residual relates `v_j` to `v_i`. Without `v` in the
  state the solver has no variable to move when the IMU says "you are doing 1.5 m/s".
- **`b_g`, `b_a` — biases.** The preintegrated deltas were computed at *some assumed*
  bias. If the truth differs, the deltas are wrong. Estimate the bias and we can *shift*
  the deltas along their Jacobians instead of re-integrating.

The error state and its retraction (⊞):

$$
\delta\mathbf{x} = \left[\, \delta\boldsymbol{\phi},\; \delta\mathbf{p},\; \delta\mathbf{v},\; \delta\mathbf{b}_g,\; \delta\mathbf{b}_a \,\right] \in \mathbb{R}^{15}
$$

$$
\mathbf{R} \leftarrow \mathbf{R}\cdot\mathrm{Exp}(\delta\boldsymbol{\phi})
\quad \text{(manifold: compose, on the RIGHT)}
$$
$$
\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p}, \qquad
\mathbf{v} \leftarrow \mathbf{v} + \delta\mathbf{v}, \qquad \dots
\quad \text{(vector space: just add)}
$$

**Only the rotation is curved.** A 15-DoF manifold with exactly one non-trivial block.

> ⚠️ **The convention collision.** `optimizeSE3` perturbs on the **LEFT** (its correction
> lives in the world frame). This state perturbs `R` on the **RIGHT** (Forster's
> preintegration Jacobians are derived that way, and the body frame is where the gyro
> increment lives). Both are correct; **the Jacobians are not interchangeable.** The
> point-to-plane Jacobian therefore had to be *re-derived* — see
> `pointToPlaneJacobianNav`, which is a different matrix from the one in
> [5-registration.md](5-registration.md) for the very same residual. `test_nav_residual`
> asserts the two genuinely differ, so nobody "unifies" them later.

## 7.4 Preintegration — why it exists

To constrain two poses with the IMU you must integrate between them. Naively that
integration is expressed in the **world** frame, so it needs `R_i` to rotate each
acceleration sample. The moment the optimizer changes its estimate of `R_i`, all several
hundred samples must be re-integrated. Inside an iterative solve, that is ruinous.

**The trick:** integrate in the frame of the *first sample* instead.

$$
\begin{aligned}
\Delta\mathbf{R} &= \prod_k \mathrm{Exp}\!\left( (\boldsymbol{\omega}_k - \mathbf{b}_g)\Delta t \right) \\
\Delta\mathbf{v} &= \sum_k \Delta\mathbf{R}_{ik}\, (\mathbf{a}_k - \mathbf{b}_a)\, \Delta t \\
\Delta\mathbf{p} &= \sum_k \left[ \Delta\mathbf{v}_{ik}\,\Delta t + \tfrac{1}{2}\Delta\mathbf{R}_{ik}(\mathbf{a}_k - \mathbf{b}_a)\,\Delta t^2 \right]
\end{aligned}
$$

These depend only on the IMU measurements and the bias — and are **completely independent
of `R_i, p_i, v_i`**. The optimizer may move the poses freely; the delta never needs
recomputing.

**Gravity is deliberately *not* integrated here.** It is a known constant in the world
frame, so it is added analytically at residual time (§7.5). Folding it in would
re-introduce the `R_i` dependence we just worked to remove.

**The bias caveat.** The deltas *do* depend on the bias, and the bias *is* being
estimated. Re-integrating on every bias nudge would defeat the purpose — so we carry
`∂Δ/∂b` and apply a **first-order correction**. `test_preintegration` verifies this is
genuinely second-order accurate: halve the bias offset and the error falls **4×**.

## 7.5 The residual

With `i` = previous scan (held fixed), `j` = current, `Δt` the interval, `g` gravity:

$$
\begin{aligned}
\mathbf{r}_{\Delta R} &= \mathrm{Log}\!\left( \hat{\Delta\mathbf{R}}^\top \cdot \mathbf{R}_i^\top \mathbf{R}_j \right) \\
\mathbf{r}_{\Delta v} &= \mathbf{R}_i^\top\left( \mathbf{v}_j - \mathbf{v}_i - \mathbf{g}\,\Delta t \right) - \hat{\Delta\mathbf{v}} \\
\mathbf{r}_{\Delta p} &= \mathbf{R}_i^\top\left( \mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i\,\Delta t - \tfrac{1}{2}\mathbf{g}\,\Delta t^2 \right) - \hat{\Delta\mathbf{p}}
\end{aligned}
$$

Read each as **"what the state says happened" minus "what the IMU says happened."** Zero
when they agree.

`Log` on the rotation row, because you cannot subtract rotations: `ΔR̂ᵀ(R_iᵀR_j)` is the
*relative* rotation between prediction and state — identity if they agree — and `Log`
maps that discrepancy into ℝ³ so least squares can square it.

### The Jacobian that everyone gets wrong

$$
\frac{\partial \mathbf{r}_{\Delta R}}{\partial \delta\boldsymbol{\phi}_j}
\;=\; \mathbf{J}_r^{-1}\!\left( \mathbf{r}_{\Delta R} \right)
$$

`r_ΔR` is a `Log`, and `δφ_j` sits *inside* it. The identity
`Log(Exp(φ)·Exp(δ)) ≈ φ + J_r⁻¹(φ)·δ` says the perturbation does **not** pass through
unchanged — the manifold's curvature stretches it by `J_r⁻¹`.

Replace it with the identity matrix (the classic shortcut) and **nothing crashes**. The
solver simply mis-weights the rotation residual, and the error grows with `|r_ΔR|` — i.e.
it is worst during aggressive rotation, exactly when the IMU is most valuable.

`test_nav_residual` proves the term is load-bearing rather than decorative: stubbing it
to `I` moves the rotation-block error from **5.5e-10 to 2.5e-02**, seven orders of
magnitude.

> Sophus gives you `exp`, `log`, `hat`, `vee` — and **not** `J_r`. It is the one piece of
> Lie algebra we write ourselves ([so3_jacobian.hpp](../glass_core/include/glass_core/so3_jacobian.hpp)),
> because it is the piece every non-trivial derivative on SO(3) needs.

### The small-angle cliff

`J_r`'s closed form divides by `θ²` and `θ³`. The naive instinct is to set the Taylor
threshold as small as possible ("the closed form is more accurate, use it whenever we
can"). **That is exactly backwards.**

`(1 − cos θ)/θ²` at `θ = 1e-8`: mathematically ≈ 0.5, but `cos(1e-8)` **rounds to exactly
1.0** in double precision, so the numerator evaluates to **zero** and the term vanishes.
Relative error reaches 100% around `θ ≈ 1e-8`.

The branch is not protecting against a division by zero. It is protecting against
**subtracting two nearly-equal floats**. Cross over while the closed form is still
*accurate* (`θ ≈ 1e-4`) and carry a second-order Taylor term. And note `θ = 0` is not an
exotic input — it is the *most common* one: a gyro at rest, over a 5 ms step.

## 7.6 Weighting — the number that decides who wins

```
LiDAR:  each residual whitened by lidar_sigma  → information 1/σ²  per point
IMU:    the 9-vector weighted by Σ⁻¹           → from preintegration's own covariance
```

**In the loose path a global scale on the LiDAR residuals cancels out of `Hξ = b` and is
harmless.** The moment a second sensor enters the same normal equations, that scale stops
being arbitrary — **it is what decides which sensor is believed.**

Omit it (weight 1.0) and you are implicitly declaring each laser return accurate to **one
metre**, while the IMU's covariance claims millimetres. The IMU then wins every
disagreement, including the ones it should lose. (Measured, before the fix: with a lying
IMU against good geometry, the LiDAR had *essentially zero influence* — 0.900 m error.)

`Σ` **grows** the longer you integrate, so `Σ⁻¹` shrinks and the IMU is trusted less. No
heuristic decides that; the propagated uncertainty does. That is the self-tuning property
tight coupling buys you.

> **Noise densities are physics, not knobs.** MEMS-grade (`accel_noise ≈ 2e-2`) is what a
> Livox contains. Plug in navigation-grade numbers (`1e-3`) and the IMU's information over
> half a second genuinely *exceeds* a few thousand point-to-plane constraints — it will
> override good geometry, and it would be **right** to.

## 7.7 What it demonstrably buys (synthetic)

From [`test_tight.cpp`](../test/test_tight.cpp):

| Scene | Loose | Tight |
|---|---|---|
| **Corridor** — X is a null space of the LiDAR Jacobian | 0.400 m error (cannot fix it) | **0.000 m** |
| **Closed room** — all 6 DoF observable | fine | 0.012 m (not degraded) |
| **Lying IMU vs strong geometry** | — | 0.016 m (**LiDAR wins**) |

The corridor is the whole thesis: the IMU supplies the direction the geometry cannot,
*without* being able to steamroll the geometry when the geometry is good.

## 7.8 🚧 Why it is OFF on the real bag

On the test bag the tight solve **slowly diverges** (Z falls quadratically). Two bugs were
found and fixed along the way; a third is structural and remains.

### Fixed: the initial velocity was zero

IMU init cannot tell **rest** from **constant velocity** ([1-imu-init.md](1-imu-init.md)),
and this robot is *already cruising* at ~1.5 m/s when recording starts. Init duly reports
"static" and seeds `v = 0`.

Loose coupling shrugs that off — velocity is only a prior there, and it self-corrects.
**Tight coupling cannot**, because it holds `x_i` FIXED: a wrong `v_i` is treated as
**certain**, and the IMU factor spends every scan insisting the robot is stationary while
the LiDAR insists it is not. The two fight, correspondences collapse, and the estimator
free-falls. *(Observed: 532/691 scans rejected, pose to +6 km.)*

**Fix:** run **loose for `tight_warmup_scans`** (default 10), let ICP *measure* the
velocity, then hand a correct state to the tight solver. It recovers **1.73 m/s in +Y** —
matching the 1.48–1.6 m/s derived independently from raw cloud geometry.

### Fixed: the preintegration window was wrong

`meas.imu` deliberately over-covers the scan — a bracket sample before, guard coverage
after (~0.12 s; see [2-sync.md](2-sync.md)). But deskew compensates every point into the
**scan-end** frame, so consecutive **poses** are 0.10 s apart.

Integrate the whole group and the IMU factor asserts *"over 0.12 s you moved Δp"* against
a pose delta covering 0.10 s — a systematic **~20% over-integration, every scan,
compounding**. It reads exactly like a gravity error.

**Fix:** clip the integration to exactly `(prev_scan_end, scan_end]`. Rejections fell
**266 → 3**.

### Fixed: the accelerometer bias was baked into "gravity"

At init we set the world gravity vector to the **measured** magnitude. That is wrong, and
the arithmetic says how wrong.

The specific force an accelerometer reads at rest is not $\mathbf{g}$ — it is
$\mathbf{g} + \mathbf{b}_a$. Folding that sum into a "gravity" constant **conflates two
quantities that behave completely differently**:

| | Fixed in which frame? | What happens when the robot turns |
|---|---|---|
| **gravity** | the **world** frame | nothing — it still points down |
| **accel bias** | the **body** frame | it rotates with the robot |

They agree at init, by construction, and then **diverge the moment you turn** — injecting a
spurious acceleration of order $\lVert\mathbf{b}_a\rVert$ in an arbitrary direction.

The observed Z fall was ~380 m over a minute, i.e.

$$
a = \frac{2 \times 380}{60^2} \approx 0.21\ \text{m/s}^2
$$

— about 2% of $g$, far too large for gyro bias or numerics, and exactly the size of a cheap
MEMS accelerometer bias.

**Fix:** use the **standard** gravity magnitude in the world frame, and let $\mathbf{b}_a$
be *estimated* as the body-frame quantity it actually is.

> **Gravity and accel bias are indistinguishable at rest.** A static window sees only their
> sum. They only separate under **rotation**, which is why the accel bias is unobservable
> at init and must be discovered from data later — and why folding it into a constant is so
> tempting and so wrong.

### ❌ Failed: loosening the bias, and why it made things WORSE

Given the above, the obvious next step: the accel bias was also **frozen**. Its prior
information was $1/(\sigma_{rw}^2\,\Delta t) \approx 10^7$, against the IMU factor's
$\approx 2\times 10^5$ — **pinned 40× harder than the data that would move it.** So it was
given a carried covariance, starting loose and shrinking as the data constrained it.

**Rejections went from 266 to 579. It got worse.**

> ### The lesson, and it is the sharpest one in this repo
>
> **You cannot fix a filter by loosening one block of a factor.**
>
> The bias was made free while $\mathbf{x}_i$ remained **infinitely certain**. The IMU
> factor still asserts that the previous pose, velocity and attitude were *exact* — so the
> only slack anywhere in the system was the bias. Every error that genuinely belonged to
> $\mathbf{x}_i$ therefore got shovelled into $\mathbf{b}_a$, which poisoned the next
> prediction, which produced more error, which went back into the bias.
>
> **A free bias against an infinitely-stiff prior is worse than a frozen one.** The frozen
> version at least fails *honestly*.
>
> The covariance must be carried for the **whole state, or none of it**. Partial slack is
> not a partial fix — it is a new failure mode.

Note the shape of this bug: a locally sensible change, no crash, no NaN, and a **worse**
estimator. It is the thesis of this repository in one commit.

### Remaining: the formulation itself

What is left is structural, and the two failures above are symptoms of it:

- **$\mathbf{x}_i$ is held fixed and infinitely certain.** This is odometry, not a sliding
  window — so an error in $\mathbf{x}_i$ can **never** be corrected.
- **Gravity is not a state**, so a tilt error in the world frame (and init measured gravity
  while the robot was *already moving*) is permanent.
- **The biases cannot absorb any of it** — and, as shown above, letting them try in
  isolation makes it worse, not better.

> **This is why FAST-LIO is an iterated EKF and not what is built here.** Carrying
> $\mathbf{x}_i$'s covariance forward is not an implementation detail — it is the mechanism
> by which *yesterday's error can be corrected by today's measurement*. Declaring
> $\mathbf{x}_i$ exact makes every mistake permanent, and leaves the estimator no way to
> discover that its gravity vector is a few degrees off.
>
> **What is built here is a factor, not a filter.**

### 🚧 Where the boundary is, deliberately

This is a **stopping point, not an abandonment.** Three pieces of the real fix are already
in place — they are inert while `imu_prior_weight: 0`, and they are the seam to build on:

| Built | Why it is needed |
|---|---|
| **Standard gravity** in the world frame | Stops conflating $\mathbf{g}$ with $\mathbf{b}_a$. Correct regardless of what comes next. |
| **`TightResult::H`** — the solve returns its information matrix | Exactly what a posterior update consumes: $\mathbf{P} \leftarrow (\mathbf{P}^{-1} + \mathbf{H})^{-1}$. |
| **Caller-supplied bias information** | The API no longer hardcodes a random-walk prior — which was conceptually wrong anyway: *a random walk says how fast a bias may **drift**, never how wrong it might have been to **begin with**.* |

**What is missing** is the propagation of a full-state covariance:

1. **Propagate** $\mathbf{P}$ across the IMU interval:
   $\mathbf{P} \leftarrow \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{G}\mathbf{Q}\mathbf{G}^\top$.
   `ImuPreintegration` already accumulates the noise term; it would also need to accumulate
   the **state-transition Jacobian** $\mathbf{F}$.
2. **Prior on the whole state**, not one block:
   $\lVert \mathbf{x} \boxminus \hat{\mathbf{x}} \rVert^2_{\mathbf{P}^{-1}}$.
3. **Posterior**: $\mathbf{P} \leftarrow (\mathbf{P}^{-1} + \mathbf{H})^{-1}$ — the piece
   that already exists.

And note what falls out for free: with a proper $\mathbf{P}$, the **`tight_warmup_scans`
hack disappears.** You would simply initialise $\mathbf{P}_v$ large — *"we do not know the
velocity"* — and let the LiDAR discover it, instead of running loose for ten scans to
measure it by hand.

That is a real piece of work, not a patch. Everything it needs — preintegration, the
residuals, the Jacobians, and the finite-difference harness that verifies them — is already
built and tested.

## 7.9 Parameters

| Param | Meaning |
|---|---|
| `registration.imu_prior_weight` | **0 = loose** (6-DoF SE(3), the healthy default). `> 0` = tight (15-DoF joint solve). |
| `registration.lidar_sigma` | Point-to-plane noise (m). **Decides which sensor wins.** §7.6. |
| `registration.tight_warmup_scans` | Loose scans before engaging the IMU, to measure the velocity init cannot observe. |
| `imu.gyro_noise`, `imu.accel_noise` | Noise densities. Physics, from the datasheet — not knobs. |
| `imu.bias_rw_gyro`, `imu.bias_rw_accel` | How fast a bias may physically drift. Stops the biases absorbing real motion when they are unobservable. |
