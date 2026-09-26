# [3] Register — producing the pose

Scan-to-map **point-to-plane registration** produces `pose_`. The loose path solves
for a pose on SE(3); the tight path also estimates velocity, biases, and gravity.

It has **two solvers**. **Loose** (§3.1–3.6): the IMU proposes the guess, then
ICP solves the 6-DoF pose alone. **Tight** (§3.7–3.14, the default): the IMU becomes a residual in
the same normal equations and the solve grows to 18 DoF. The loop below is shared.

Code: [`registration.cpp`](../src/lio/registration.cpp),
[`registration.hpp`](../include/glasslio/registration.hpp).
The *solver* lives separately in [`gauss_newton.hpp`](../glass_core/include/glass_core/gauss_newton.hpp)
— see [gauss_newton.md](gauss-newton.md).
Self-check: [`test_registration.cpp`](../test/test_registration.cpp).

The loop, four steps:

```
PREDICT   →   ASSOCIATE   →   SOLVE   →   ACCEPT or COAST
   ↑              └──────────────┘
 (IMU)          repeat, max 30×
```

---

## 3.1 Predict — keeping the guess inside the basin

**ICP is a local optimizer.** It has a small basin of convergence, and outside it, it
does not fail loudly — it locks onto the *wrong wall* and reports a confident,
low-residual, completely wrong fit. The prior's entire job is to land the initial
guess inside that basin.

$$
\mathbf{R}_{\text{guess}} = \mathbf{R}_{\text{pose}} \cdot \Delta\mathbf{R}_{\text{imu}},
\qquad
\mathbf{t}_{\text{guess}} = \mathbf{t}_{\text{pose}} + \mathbf{v}\,\Delta t
$$

with $\Delta\mathbf{R}_{\text{imu}}$ the gyro rotation across the scan, and $\mathbf{v}$
from the previous pose delta.

**Rotation** comes from the integrated gyro and is genuinely good — the same
integration that drives deskew ([deskew.md](3-deskew.md)), reused. Rotation is the
dangerous DoF for ICP (a few degrees of error moves distant points metres), so this
is the prior that matters most.

**Translation** comes from a constant-velocity model — see §3.5, it has history.

## 3.2 Associate — nearest plane, not nearest point

ICP breaks a chicken-and-egg problem — *you need the pose to find correspondences,
and the correspondences to find the pose* — by **alternating**: assume the current
pose is right, find correspondences, solve, repeat.

```
repeat (max_iterations):
    1. ASSOCIATE   q = T·p ; find the nearest map plane to q
    2. SOLVE       T ← argmin Σ (point-to-plane residual)²
```

That alternation *is* the ICP loop. In this codebase it is literally the callback the
solver invokes each iteration — see [gauss_newton.md](gauss-newton.md).

**Correspondence is a hash lookup, not a KD-tree query.** The map caches one plane per
voxel, so we hash `q` to its voxel, scan the 27-cell neighbourhood, and take the
nearest valid plane within `max_correspondence_distance`. **Constant work per point.**
This is the single biggest reason registration is fast enough — see the GICP
comparison in [pipeline.md](pipeline.md#performance-pcl-gicp--hand-rolled-point-to-plane).

> **Why the 27 cells are enough:** with `map.voxel_size ≥ max_correspondence_distance`,
> any plane within the search radius of `q` must live in `q`'s own voxel or one of its
> 26 neighbours. Shrink `map.voxel_size` below the correspondence distance and the
> search silently becomes incomplete — it will miss valid correspondences that lie
> two cells away, and you will never see an error, only a worse fit.

## 3.3 The residual — why point-to-plane

**A LiDAR never re-samples the same physical point.** It hits the same *surface* in
different spots, every scan, forever. Point-to-*point* ICP asks "which scan point
corresponds to which map point?" — a question with **no correct answer**, because the
two clouds sample the same wall at different, arbitrary places. Forcing point A onto
point B injects an error equal to their spacing along the surface.

Point-to-**plane** asks a question that *does* have an answer: *how far is this point
from the surface?* Penalising only the distance **along the normal** lets points slide
freely **across** the surface — which is exactly what a wall does and does not
constrain.

For correspondence `i` with plane `(cᵢ, nᵢ)`:

$$
r_i(\mathbf{T}) \;=\; \mathbf{n}_i^\top \left( \mathbf{T}\,\mathbf{p}_i - \mathbf{c}_i \right)
\qquad \text{(signed distance to the plane)}
$$

One scalar per correspondence, not three. That is also why it converges faster: the
cost surface has no spurious minima from tangential mismatch.

## 3.4 The Jacobian — and the side you perturb on

Linearise about the current $\mathbf{T}$ with a **left perturbation**
$\boldsymbol{\xi} = [\boldsymbol{\rho};\, \boldsymbol{\phi}] \in \mathfrak{se}(3)$,
i.e. $\mathbf{T} \leftarrow \mathrm{Exp}(\boldsymbol{\xi})\,\mathbf{T}$.

To first order, $\mathrm{Exp}(\boldsymbol{\xi})\,\mathbf{q} \approx \mathbf{q} + \boldsymbol{\rho} + \boldsymbol{\phi} \times \mathbf{q}$, so

$$
\begin{aligned}
r(\boldsymbol{\xi})
&\approx r + \mathbf{n}^\top\boldsymbol{\rho} + \mathbf{n}^\top(\boldsymbol{\phi} \times \mathbf{q}) \\
&= r + \mathbf{n}^\top\boldsymbol{\rho} + (\mathbf{q} \times \mathbf{n})^\top \boldsymbol{\phi}
\qquad \text{[scalar triple product]}
\end{aligned}
$$

giving the $1\times 6$ Jacobian

$$
\mathbf{J}_i = \begin{bmatrix} \mathbf{n}_i^\top & (\mathbf{q}_i \times \mathbf{n}_i)^\top \end{bmatrix}
$$

> ⚠️ **The rotation block is `(q × n)`, not `(n × q)`.** Swap them and every rotational
> update flips sign — the solver walks *away* from the solution while still reporting a
> plausible residual. This is the class of bug that "still looks like a working
> estimator", and the reason the Jacobian is checked against finite differences rather
> than trusted.

**The perturbation side is not a free choice.** It is fixed by the frame the correction
lives in. Here the correction is expressed in the **world** frame, so it composes on
the **left**. In `GyrInt` the increment is measured in the **body** frame, so it
composes on the **right** (`R ← R·Exp(Δθ)`). Same manifold, same `Exp`, opposite side.
See [gauss_newton.md](gauss-newton.md#the-perturbation-convention).

## 3.5 The constant-velocity prior — a crutch with a rap sheet

`use_constant_velocity` is **on**. Without it, `t_guess = t_pose` — the guess carries
**no translation at all**, so if the worker ever drops a scan (see `max_queue_size`),
the robot has physically moved metres by the time the next scan is processed, and ICP
starts hopelessly far from the answer.

### The runaway it once caused

Enabling it made the pose accelerate to ~26 m/s and finish 133 m from the start —
**while the fit still looked healthy.**

```
guess is too far ahead
   → correspondence distance is loose enough that ICP "converges" near it
   → that mis-registered scan is INSERTED INTO THE MAP
   → the map itself drifts
   → the next scan aligns to the drifted map, v grows
   → runaway
```

**The map closing the loop is what made this vicious.** The estimator and its own
reference drifted *together*, staying mutually consistent — so **no residual ever
complained.** A low error was not evidence against it. This is the defining hazard of
estimator code: the failure is self-confirming.

### Why it is safe now

**The loop is broken at the insert.** A rejected scan is no longer added to the map
(§3.6), so a bad guess can no longer poison the reference it is measured against. The
feedback path simply doesn't exist any more.

If you ever see the pose accelerating away with a healthy `rmse`, this parameter is
still the first thing to turn off.

### It is a crutch for a missing state

On the loose path, velocity is **finite-differenced from the very poses it helps produce** —
a feedback loop by construction. You are carrying an accelerometer, a device that
measures acceleration directly. Only a *tightly-coupled* estimator can use it — and the tight
path does: velocity becomes a state driven by the accelerometer, and after its warm-up
`predictState` replaces the constant-velocity guess outright (§3.11).

## 3.6 Accept or coast

```
if (!valid || rmse > max_rmse)  →  keep the prediction, flag DIVERGED, do NOT insert
   where valid = (correspondences ≥ min_correspondences) && the solve was finite
                 && the translation eigenvalue ratio ≥ min_translation_eigenvalue_ratio
```

**Note what is *not* in that condition: `converged`.**

Hitting `max_iterations` is **not failure.** ICP routinely plateaus above `eps` while
sitting on a perfectly good fit. An earlier version treated `!converged` as a
rejection, threw away good poses, and froze the estimator. `valid` — enough
correspondences, finite solve — plus the residual is the trust signal. `converged` is
diagnostics.

On failure we **coast on the prediction**: keep the IMU-predicted pose and move on.
The asymmetry is deliberate —

- a **wrong pose inserted into the map** poisons it *permanently*, and the map is what
  every future scan is measured against;
- a **slightly stale pose** recovers on the very next scan.

So when in doubt, refuse. Under-constrained (`correspondences < min_correspondences`)
is refused for the same reason: the 6-DoF problem genuinely has no answer, and
inventing one is worse than admitting it.

### 3.6.1 Degenerate geometry — a healthy RMSE can still be a lie

`min_correspondences` and `max_rmse` both look at the SOLVE'S OUTPUT. Neither looks at
whether the geometry that produced it could support a unique answer in every
direction. A scene whose plane normals cluster into one or two directions — an open
outdoor stretch: mostly ground plus a wall — leaves one translation direction with
almost no normal support. Nothing in the data resists sliding along it, so
Gauss-Newton solves for whatever noise happens to be there, and the pose it hands back
can still report a plausible correspondence count and a low RMSE at the *wrong*
position, because locally the (now-shifted) fit still looks fine.

First caught on [M3DGR](https://github.com/sjtuyinjie/M3DGR)'s `Outdoor01` sequence
(RTK ground truth — glasslio's first run against real absolute GT, not just its own
registration residual): the estimate's path length came out to **38,657 m against a
true 346 m** — not drift, an oscillation. Per-scan pose jumps of 2–4 m sustained for
hundreds of scans, `rmse` sitting at its normal 0.10–0.16 the whole time, correspondence
counts never dropping below 300. `pose_trusted` never flagged a single one of them.

**The fix**: after the solve converges, re-run `associate()` once more at the
converged pose (§3.2's lambda, no extra Gauss-Newton iterations — see
`alignPointToPlane`) to get `H_t = sum(w_i · normal_i · normal_i^T)`, the translation
block of `H`. Its smallest eigenvalue divided by its trace is a scene-scale-invariant
measure of how well the weakest direction is constrained — the RAW eigenvalue is
useless here, since it scales with correspondence count and cannot tell
"under-constrained" from "just fewer points."

Calibrated empirically, not guessed: the indoor test bag's ratio never drops below
0.117 at the 5th percentile across ~180 scans; `Outdoor01`'s divergence window never
rises above 0.034 across ~110 scans. The two do not overlap — `min_translation_eigenvalue_ratio`
sits at 0.05, in the gap.

**This alone was not enough.** Gating out degenerate scans stops them being trusted,
but `registerScanLoose` coasts on `use_constant_velocity` (§3.5) when it refuses —
and `Outdoor01`'s degeneracy is not a brief patch, it is sustained. A rejected scan
means a stale map; a stale map means the next scan degenerates too; the cascade
free-integrates on the IMU alone with nothing ever correcting it, and the pose ran
away to hundreds of metres — the *exact* runaway §3.5 already documents, just
triggered by sustained degeneracy instead of a dropped-scan gap. **Tight coupling
(`imu_prior_weight: 1.0`) is what actually rescues `Outdoor01`**: the IMU becomes a
prior in the *same* solve rather than a fallback behind a rejection cliff, so it
takes over smoothly, scan by scan, exactly where the LiDAR term is weak. RPE went
from 22.6 m rmse (max 96.9 m) to **0.29 m rmse (max 1.9 m)**; path length from 111x
truth to 1.1x. Loose plus this gate alone is not the fix — tight coupling is what
the gate was clearing the way for.

A moderate APE remains even under tight coupling despite RPE being small — the
estimate is locally consistent scan-to-scan but accumulates global error over the
whole run. This is a separate mechanism from the translation degeneracy above:
[deskew](3-deskew.md) keeps its gyro bias in sync with the tight solver's own
continuously-refined estimate (`state_.bg`), rather than freezing it at the
init-window value, and that resync is itself gated by the LiDAR's rotational
conditioning — see [§3.13](#keeping-deskews-gyro-bias-in-sync-with-the-solve) for how the two
interact.

---

## 3.7 Tight coupling — the IMU inside the solve

Everything above is the **loose** path (`imu_prior_weight: 0`): the IMU proposes the guess
(§3.1), then ICP solves alone and the IMU gets no further vote. Sections 3.7–3.14 are the
**tight** path, selected by `imu_prior_weight > 0` — **the default** (`1.0`). Same stage, same map, same
point-to-plane residual — but the IMU becomes a **residual in the same normal equations**
instead of a hint.

Code: [`tight_registration.cpp`](../src/lio/tight_registration.cpp) (the solve),
[`nav_residual.hpp`](../glass_core/include/glass_core/nav_residual.hpp) (every residual and
Jacobian), [`preintegration.hpp`](../glass_core/include/glass_core/preintegration.hpp),
[`nav_state.hpp`](../glass_core/include/glass_core/nav_state.hpp),
[`so3_jacobian.hpp`](../glass_core/include/glass_core/so3_jacobian.hpp).
Self-checks: [`test_tight.cpp`](../test/test_tight.cpp),
[`test_nav_residual.cpp`](../glass_core/test/test_nav_residual.cpp),
[`test_preintegration.cpp`](../glass_core/test/test_preintegration.cpp).

> **Status: working, and ON by default.** Tight tracks the Livox test bag at **parity with
> loose** (429 m vs 434 m of trajectory), recovers the axis a corridor hides from the LiDAR
> (0.40 → 0.00 m), and on real sustained degeneracy — M3DGR's `Outdoor01`, RTK ground truth —
> it **beats** loose outright (RPE 22.6 m → 0.29 m rmse, §3.6.1). That result is what earned
> it the default (§3.13). Getting it to work took a string of silent bugs and one
> confidently wrong diagnosis; that story is
> [testing.md §12](testing.md#12-case-study--making-tight-coupling-work-on-the-real-bag).

### What loose coupling actually costs

In the loose pipeline the IMU produces `guess`, and then ICP is free to ignore it.
Two consequences:

**Degenerate geometry has no fallback.** Point-to-plane in a long corridor constrains
you *across* the walls but not *along* them: that direction is a genuine **null space**
of `Σ JᵀJ`. ICP will slide along it, and the LDLT solve will not complain, because the
residuals honestly do not care. The IMU predicts motion, but it has no vote in the
registration residual.

**`use_constant_velocity` is a symptom.** You carry an accelerometer — a device that
measures the thing loose coupling *guesses* by finite-differencing consecutive poses.
Velocity is derived from the very poses it is meant to help produce. That is a feedback
loop by construction, and it is what made the runaway of §3.5 possible.

### The one idea

**In the tight path, the IMU and LiDAR contribute residual blocks to the same
normal equations.**

**Loose** — LiDAR only, with a robust point-to-plane loss:

$$
\mathcal{L}_{\mathrm{loose}}(\mathbf{T})
= \sum_l \rho\!\left(r_l(\mathbf{T})/\sigma_l\right).
$$

**Tight** — LiDAR and the preintegrated IMU factor constrain the new state
together, with bias and gravity priors:

$$
\begin{aligned}
\mathcal{L}_{\mathrm{tight}}(\mathbf{x}_j,\mathbf{g})
={}& \sum_l \rho\!\left(r_l(\mathbf{x}_j)/\sigma_l\right)
+\|\mathbf{r}_{\mathcal I}(\mathbf{x}_i,\mathbf{x}_j,\mathbf{g})\|^2_{\Sigma_{\mathrm{eff}}}\\
&+\|\mathbf{b}_j-\mathbf{b}_i\|^2_{P_b}
+\|\mathbf{g}-\mathbf{g}_{\mathrm{init}}\|^2_{\sigma_g^2 I}.
\end{aligned}
$$

At one linearization, with correspondences and robust weights fixed, the
Gauss-Newton information has the shape

$$
H_{\mathrm{total}} =
J_{\mathcal I}^{\top}\Sigma_{\mathrm{eff}}^{-1}J_{\mathcal I}
+\sum_l \frac{w_l}{\sigma_l^2}J_l^\top J_l
+H_{\mathrm{bias}}+H_{\mathrm{gravity}}.
$$

**That sum is the fusion within the tight solve.** Its information matrices set
the relative influence of IMU and LiDAR in each direction. Where LiDAR geometry
is weak, its contribution along that direction is small, so IMU information can
carry the estimate. Section 3.11 shows the actual four-block system, including
robust weights and priors.

The loose path is a separate 6-DoF SE(3) solve selected when the IMU weight is zero.
Conceptually, it keeps the LiDAR terms and uses the IMU only to predict a starting
pose. Simply zeroing the IMU block of the 18-DoF tight system would leave velocity
unconstrained and make that system singular; it is not the implementation of loose mode.

## 3.8 The state — 18 DoF, one curved block

$$
\mathbf{x} = \left( \mathbf{R},\; \mathbf{p},\; \mathbf{v},\; \mathbf{b}_g,\; \mathbf{b}_a,\; \mathbf{g} \right)
$$

The 15-DoF navigation state (`NavState`) plus world-frame gravity. A quantity belongs in the
state if a residual mentions it and we do not already know it:

- **`v` — velocity.** The IMU's `Δv` residual relates `v_j` to `v_i`. Without `v` in the
  state the solver has no variable to move when the IMU says "you are doing 1.5 m/s".
- **`b_g`, `b_a` — biases.** The preintegrated deltas were computed at *some assumed*
  bias. If the truth differs, the deltas are wrong. Estimate the bias and we can *shift*
  the deltas along their Jacobians instead of re-integrating.
- **`g` — gravity.** Init measures "down" from a window that may not have been at rest (a
  static window cannot tell rest from constant velocity — [1-imu-init.md §3](1-imu-init.md)),
  so the world frame can carry a small tilt. Held as a constant, that tilt leaks a constant
  acceleration into every integration forever; as a state, the IMU's `Δv`/`Δp` residuals can
  drain it. It is anchored to its init value, for a reason §3.11 makes painfully clear.

The error state and its retraction (⊞):

$$
\delta\mathbf{x} = \left[\, \delta\boldsymbol{\phi},\; \delta\mathbf{p},\; \delta\mathbf{v},\; \delta\mathbf{b}_g,\; \delta\mathbf{b}_a,\; \delta\mathbf{g} \,\right] \in \mathbb{R}^{18}
$$

$$
\mathbf{R} \leftarrow \mathbf{R}\cdot\mathrm{Exp}(\delta\boldsymbol{\phi})
\quad \text{(manifold: compose, on the RIGHT)}
$$
$$
\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p}, \qquad
\mathbf{v} \leftarrow \mathbf{v} + \delta\mathbf{v}, \qquad \dots, \qquad
\mathbf{g} \leftarrow \mathbf{g} + \delta\mathbf{g}
\quad \text{(vector space: just add)}
$$

**Only the rotation is curved.** An 18-DoF manifold with exactly one non-trivial block. The
index order (`kIdxPhi`, `kIdxPos`, … in `nav_state.hpp`) is a contract: every Jacobian writes its
columns at those offsets, and one wrong offset applies the rotation correction to the
accelerometer bias — silently.

> ⚠️ **The convention collision.** `optimizeSE3` perturbs on the **LEFT** (its correction
> lives in the world frame). This state perturbs `R` on the **RIGHT** (Forster's
> preintegration Jacobians are derived that way, and the body frame is where the gyro
> increment lives). Both are correct; **the Jacobians are not interchangeable.** The
> point-to-plane Jacobian therefore had to be *re-derived* for the same residual, and
> `test_nav_residual` asserts the two genuinely differ, so nobody "unifies" them later.

The re-derivation, for a sensor-frame point $\mathbf{p}_s$ and
$r = \mathbf{n}^\top(\mathbf{R}\mathbf{p}_s + \mathbf{p} - \mathbf{c})$, with
$(\cdot)^\wedge$ the hat operator ($\mathbf{a}^\wedge\mathbf{b} = \mathbf{a}\times\mathbf{b}$):

$$
\mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol{\phi})\,\mathbf{p}_s
\;\approx\; \mathbf{R}\,(\mathbf{I} + \delta\boldsymbol{\phi}^\wedge)\,\mathbf{p}_s
\;=\; \mathbf{R}\mathbf{p}_s + \mathbf{R}\,(\delta\boldsymbol{\phi}\times\mathbf{p}_s)
\;=\; \mathbf{R}\mathbf{p}_s - \mathbf{R}\,\mathbf{p}_s^\wedge\,\delta\boldsymbol{\phi}
$$

so

$$
\frac{\partial r}{\partial \delta\boldsymbol{\phi}} = -\mathbf{n}^\top \mathbf{R}\,\mathbf{p}_s^\wedge,
\qquad
\frac{\partial r}{\partial \delta\mathbf{p}} = \mathbf{n}^\top
$$

against §3.4's $[\,\mathbf{n}^\top,\ (\mathbf{q}\times\mathbf{n})^\top\,]$ for the left
perturbation. Every velocity, bias and gravity column is **structurally zero** — a laser return
knows nothing about them. That zero matters twice: it is why loose coupling could never estimate
those states, and it is why the LiDAR can discipline velocity *only through position* (§3.12).

## 3.9 Preintegration — why it exists

To constrain two poses with the IMU you must integrate between them. A naive
world-frame integration rotates each acceleration sample using the estimated
starting orientation. If an optimizer changes that orientation, it must repeat
the integration. The cost scales with interval length and solver iterations:
glass-lio's 0.1 s scan spans about 20 samples at 200 Hz, while a longer-lived
window factor can span hundreds. Bias correction and computing covariance once
also make preintegration useful for short intervals.

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

**Gravity is deliberately *not* integrated here.** It is a world-frame quantity, so it is
added analytically at residual time (§3.10). Folding it in would re-introduce the `R_i`
dependence we just worked to remove.

**The bias caveat.** The deltas *do* depend on the bias, and the bias *is* being
estimated. Re-integrating on every bias nudge would defeat the purpose — so we carry
`∂Δ/∂b` and apply a **first-order correction**. `test_preintegration` verifies its error is
genuinely second order: halve the bias offset and the error falls **4×**.

### The interval is not the MeasureGroup

`meas.imu` deliberately over-covers the scan — a bracket sample before, guard coverage
after (~0.12 s; see [2-sync.md](2-sync.md)). But deskew compensates every point into the
**scan-end** frame, so consecutive **poses** are 0.10 s apart. Integrate the whole group and
the IMU factor asserts *"over 0.12 s you moved Δp"* against a pose delta covering 0.10 s — a
systematic **~20% over-integration, every scan, compounding**, which reads exactly like a
gravity error. So `buildPreintegration` clips to exactly `(prev_scan_end, scan_end]`,
including partial sample intervals.

**Snapshot sensors.** A ToF camera exposes every point at one instant and ships no per-point
time, so `t₀ = t₁ = 0` and that window collapses to `[0, 0]` — every tight scan then coasts
silently and the pose freezes. For a snapshot, the scan-end instant is the message **header
stamp** (deskew falls back to it when the per-point span is zero —
[3-deskew.md §3](3-deskew.md)), so the window runs header to header. The scan's end is also
recorded *before* the empty-window guard, so a first snapshot scan that coasts cannot leave
every later window stuck at zero.

## 3.10 The IMU residual — and the Jacobian everyone gets wrong

With `i` = previous scan (held at its estimate), `j` = current, `Δt` the interval, `g` the
gravity iterate:

$$
\begin{aligned}
\mathbf{r}_{\Delta R} &= \mathrm{Log}\!\left( \hat{\Delta\mathbf{R}}^\top \cdot \mathbf{R}_i^\top \mathbf{R}_j \right) \\
\mathbf{r}_{\Delta v} &= \mathbf{R}_i^\top\left( \mathbf{v}_j - \mathbf{v}_i - \mathbf{g}\,\Delta t \right) - \hat{\Delta\mathbf{v}} \\
\mathbf{r}_{\Delta p} &= \mathbf{R}_i^\top\left( \mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i\,\Delta t - \tfrac{1}{2}\mathbf{g}\,\Delta t^2 \right) - \hat{\Delta\mathbf{p}}
\end{aligned}
$$

Read each as **"what the state says happened" minus "what the IMU says happened."** Zero
when they agree. The hatted deltas are the preintegrated ones shifted to `x_j`'s *current*
bias estimate by the first-order correction of §3.9.

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

## 3.11 One tight solve, four residual blocks

Every iteration, `alignTightlyCoupled` builds one 18×18 system from four blocks:

| Block | Rows | Residual | Weight |
|---|---|---|---|
| LiDAR | 1 per correspondence | $\mathbf{n}^\top(\mathbf{R}\mathbf{p}_s + \mathbf{p} - \mathbf{c})$ (§3.8) | $1/\sigma^2$ from `lidar_sigma`, Huber |
| IMU factor | 9 | $[\mathbf{r}_{\Delta R};\ \mathbf{r}_{\Delta v};\ \mathbf{r}_{\Delta p}]$ (§3.10) | $\boldsymbol{\Sigma}_{\text{eff}}^{-1}\cdot$ `imu_prior_weight` |
| Bias prior | 6 | $[\mathbf{b}_{g,j} - \mathbf{b}_{g,i};\ \mathbf{b}_{a,j} - \mathbf{b}_{a,i}]$ | carried bias covariance⁻¹ |
| Gravity prior | 3 | $\mathbf{g} - \mathbf{g}_{\text{init}}$ | $\mathbf{I}/$`gravity_sigma`² |

then solves, retracts (⊞ on the nav state, plain addition on `g`), and re-associates — the
same loop as §3.2, over a bigger state.

**`x_i`'s uncertainty is carried into the IMU factor.** The previous state is not solved for;
it is held at its estimate. Held as *exact*, the IMU's information over 0.1 s is enormous — and
correctly so — so it would overrule every other measurement. Instead the factor's covariance is
inflated by whatever `x_i` was already unsure of:

$$
\boldsymbol{\Sigma}_{\text{eff}} = \boldsymbol{\Sigma}_{\text{pre}} + \mathbf{J}_i\,\mathbf{P}_i\,\mathbf{J}_i^\top
$$

the marginalization of an independent Gaussian perturbation of `x_i` out of this one linearized factor. `J_i` is
`imuJacobianI` (the `d/dx_i` block, verified against finite differences), and `P_i` is the
previous solve's local covariance — the nav block of the inverse of its full 18×18 information matrix `H`. The effective factor covariance is
recomputed every iteration, because `J_i` depends on the current `x_j`.

**The bias prior comes from a carried covariance**, not a random-walk constant: it grows by the
random walk over each interval and is replaced after an accepted solve by the bias block of
`H_total⁻¹`. The total information already includes the prior once; adding `P⁻¹` again would
double-count it. Inverting `H_bb` alone would instead give a conditional covariance, treating
other variables as certain. The implementation solves the full `H_total P = I` before
extracting the bias and nav blocks, and retains the carried covariances if that solve is invalid.
This is a local Gaussian approximation with the last linearization and robust weights;
selected covariance blocks and a fixed gravity prior do not constitute a full Bayesian filter. A random walk says how fast a bias may **drift** — never how wrong it
might have been to **begin with**, and an accel bias is exactly a quantity you start out wrong
about.

**The gravity prior is anchored to the fixed init value, not the running estimate.** Gravity is
nearly unobservable over a single 0.1 s scan. Anchored to its own carried estimate, it has no
restoring force: every small pose error gets explained by tilting `g`, the anchor chases it, and
it random-walks to `|g| ≈ 25` within 200 scans — which injected a fake acceleration that threw
the pose ~500 km. Anchored to init (and stiff, `gravity_sigma = 0.05`), it can drain a small tilt
but cannot wander. (That was the dominant real-bag bug — [testing.md §12](testing.md#12-case-study--making-tight-coupling-work-on-the-real-bag).)

**Warm-up.** IMU init cannot observe velocity, and the test bag starts with the robot already
cruising at ~1.5 m/s. Seed `v = 0` and the IMU factor insists the robot is stationary while the
LiDAR insists it is not; they fight until the estimator free-falls. So the first
`tight_warmup_scans` (10) run **loose**, ICP measures the velocity by finite difference, and the
tight solver starts from that — it recovers **1.73 m/s in +Y**, against 1.48–1.6 m/s derived
independently from raw cloud geometry.

**Accept or coast** — the §3.6 rule, unchanged. On rejection the estimator coasts on
`predictState`, which is now a real dead-reckon: velocity and biases are states driven by the
accelerometer, not a constant-velocity guess. The reported `rmse` is LiDAR-only, so it stays
directly comparable with the loose path's.

## 3.12 Weighting — the number that decides who wins

```
LiDAR:  each residual whitened by lidar_sigma  → information 1/σ²  per point
IMU:    the 9-vector weighted by Σ_eff⁻¹       → from preintegration's own covariance
```

**In the loose path a global scale on the LiDAR residuals cancels out of `Hξ = b` and is
harmless.** The moment a second sensor enters the same normal equations, that scale stops
being arbitrary — **it is what decides which sensor is believed.**

Omit it (weight 1.0) and you are implicitly declaring each laser return accurate to **one
metre**, while the IMU's covariance claims millimetres. The IMU then wins every
disagreement, including the ones it should lose. (Measured, before the fix: with a lying
IMU against good geometry, the LiDAR had *essentially zero influence* — 0.900 m error.)

**Calibrate it to the sensor, in both directions.** `lidar_sigma` is the point-to-plane noise,
and a Livox's real range noise is ~2 cm — hence the default `0.02`. At `0.05` the LiDAR was
declared *less* accurate than it is, and that bit through the structural zero of §3.8: the
LiDAR disciplines velocity only through position, so pinning position too weakly let the IMU
over-integrate velocity to ~40 m/s on a bland stretch. Calibrating collapsed the drift from
1 339 m to 429 m. The same knob, set too loose instead of too tight.

`Σ` **grows** the longer you integrate, so `Σ⁻¹` shrinks and the IMU is trusted less. No
heuristic decides that; the propagated uncertainty does. That is the self-tuning property
tight coupling buys you.

> **Noise densities are physics, not knobs.** MEMS-grade (`accel_noise ≈ 2e-2`) is what a
> Livox contains. Plug in navigation-grade numbers (`1e-3`) and the IMU's information over
> half a second genuinely *exceeds* a few thousand point-to-plane constraints — it will
> override good geometry, and it would be **right** to.

## 3.13 What it buys, and where it stands

**Synthetic**, from [`test_tight.cpp`](../test/test_tight.cpp):

| Scene | Loose | Tight |
|---|---|---|
| **Corridor** — X is a null space of the LiDAR Jacobian | 0.400 m error (cannot fix it) | **0.000 m** |
| **Closed room** — all 6 DoF observable | fine | 0.013 m (not degraded) |
| **Lying IMU vs strong geometry** | — | 0.001 m (**LiDAR wins**) |

The corridor is the whole thesis: the IMU supplies the direction the geometry cannot,
*without* being able to steamroll the geometry when the geometry is good.

**Real data**, measured deterministically with [`tight_replay`](../src/tight_replay.cpp) on the
Livox bag:

| | trajectory | scale-gate ratio | max `‖v‖` |
|---|---|---|---|
| Tight (`lidar_sigma` 0.02, gravity anchored to init) | **429 m** | 2.72× | 7 m/s |
| Loose (the trusted baseline) | 434 m | 2.76× | — |

- **Parity, not victory.** This bag has good geometry, so the IMU rarely has to rescue
  anything, and there is no ground truth to break the tie.
- **The scale gate is the wrong yardstick for a driving bag.** A large trajectory is *correct*
  when the car really travels, which is why even trusted loose "fails" at 2.76×. The gate was
  built for a bounded room; here "2.72×" means "tracks like loose", not "diverges".
- **On the 3-ToF rig** ([`config/3lidars.yaml`](../config/3lidars.yaml)) tight converged toward
  loose as `lidar_sigma` shrank and never beat it. Fusing three fields of view already makes the
  geometry observable; the IMU helps where geometry is *degenerate*, and that rig removes the
  degeneracy.

- **On genuinely degenerate real data it wins.** M3DGR's `Outdoor01` (RTK ground truth) is a
  long open stretch whose normals leave one translation axis unconstrained (§3.6.1). Loose
  oscillates there even with the degeneracy gate — coasting has no floor under *sustained*
  degeneracy — while tight takes over smoothly, scan by scan: RPE **22.6 m → 0.29 m rmse**,
  path length **111× → 1.1×** truth. Full comparison: [benchmark.md](benchmark.md).

That is the bar the old OFF default was waiting for, so tight is now **ON by default**
(`imu_prior_weight: 1.0`). Set it to `0` for loose — which, per the 3-ToF result above, is
still the right call where fused geometry has already removed the degeneracy.

### Keeping deskew's gyro bias in sync with the solve

`state_.bg` is a live optimizer variable, re-estimated every scan. [Deskew](3-deskew.md)'s
intra-scan compensation uses a gyro bias too, and it is kept in sync: after each tight solve,
`LioEstimator::registerScanTight()` calls `deskew_->set_gyro_bias(state_.bg)`, so the next
scan is deskewed with the current estimate rather than the one measured once at init.

The resync is gated:

```
if (rotation_eigenvalue_ratio >= min_rotation_eigenvalue_ratio)
    deskew_->set_gyro_bias(state_.bg)
```

`rotation_eigenvalue_ratio` is the smallest-eigenvalue/trace ratio of the LiDAR-only rotation
block of `H` — §3.6.1's translation check applied to rotation, snapshotted before the
IMU/bias/gravity terms enter the solve. It answers *"how well does the LiDAR alone constrain
rotation this scan?"*

When it barely does, the bias the solve settles on is mostly IMU dead-reckoning. The **pose**
is fine either way — §3.12's information arbitration handles that — but feeding an unverified
bias into deskew changes the geometry every *following* scan is built from, and over a
sustained rotationally-degenerate stretch that compounds: each scan's slightly-off bias
distorts the next scan's points, which distorts its bias estimate. On M3DGR's `Corridor02`
an ungated resync cascaded into total divergence. The gate keeps the resync to scans the LiDAR
itself backs up. `min_rotation_eigenvalue_ratio` defaults to `0.05`, the translation gate's
starting value, and has not been swept.

<a id="current-state-limit"></a>
## 3.14 The limit — a current-state solve with partial covariance

After each scan, the previous state is committed and never re-optimized. Its uncertainty
enters the next IMU factor through $\Sigma_{\mathrm{eff}}$, which marginalizes an
independent Gaussian perturbation of that state for this one linearized factor (§3.11).
The solver extracts the navigation and bias blocks from the full local posterior
$H_{\mathrm{total}}^{-1}$. It carries correlations among navigation variables, but
does not carry gravity cross-correlations or a complete joint history.

This is a current-state optimization with partial uncertainty propagation. Factors
can also be used inside filters and sliding windows; the distinction is which state
distribution and past states the estimator retains.

A full-state filter would:

1. Propagate a complete current-state covariance,
   $P^- = F P^+ F^\top + G Q G^\top$, across the IMU interval.
   ImuPreintegration::covariance() supplies the 9×9 uncertainty of the
   preintegrated deltas; it is not, by itself, the full state process-noise term.
2. Use the propagated uncertainty as a prior on the current state, retaining its
   cross-correlations through the LiDAR update.
3. Compute the joint local posterior from the **total** information, including each
   prior once: $P^+ \approx H_{\mathrm{total}}^{-1}$. Adding a prior information
   matrix again after it is already in $H_{\mathrm{total}}$ would double-count it.

A filter can update the *current* velocity and biases through their correlations with
the measured pose. It does not re-optimize a previously committed pose. A fixed-lag
window does retain recent poses for later scans to revise; removing the oldest while
preserving its linearized information requires Schur-complement marginalization
([marginalization.hpp](../glass_core/include/glass_core/marginalization.hpp)).

A fuller covariance might reduce the need for the loose warm-up, since a large
initial velocity variance would represent uncertainty in $v_0$. Whether it can
remove that warm-up without harming convergence requires a real-data test.

This limit was once believed to be the *cause* of the real-bag divergence. It was
not — two miscalibrated numbers were
([testing.md §12](testing.md#12-case-study--making-tight-coupling-work-on-the-real-bag)).
The current solve is at parity with loose on the original bag; a full filter or
window remains a separate design choice.

## 3.15 Parameters that bite

| Param | Why |
|---|---|
| `max_correspondence_distance` | **The key knob.** Too small: fast motion puts the true correspondence out of reach and it never converges. Too large: it matches the wrong wall, and re-opens the runaway. |
| `max_rmse` | The coast threshold (§3.6). |
| `huber_delta` | Robust threshold — see [gauss_newton.md](gauss-newton.md#7-robust-weighting). |
| `min_correspondences` | Below this the problem is under-constrained; refuse. |
| `min_translation_eigenvalue_ratio` | The degeneracy gate (§3.6.1). Below this, some translation direction has essentially no normal support; refuse regardless of RMSE. **Loose path only.** |
| `registration.min_rotation_eigenvalue_ratio` | Tight path's rotational counterpart — see [§3.13](#keeping-deskews-gyro-bias-in-sync-with-the-solve). Gates deskew's bias resync, not the pose solve. |
| `max_iterations` | Cost ceiling. Hitting it is not failure. |
| `use_constant_velocity` | §3.5. Consulted by the loose path and the tight warm-up only. |
| `imu_prior_weight` | `0` = loose (6-DoF SE(3)). `> 0` = tight (18-DoF `[R,p,v,b_g,b_a,g]`); **`1.0`, the default**, trusts the IMU exactly as far as its own covariance says. |
| `lidar_sigma` | Point-to-plane noise (m). **Decides which sensor wins** (§3.12). Calibrate it to the sensor — `0.02` for a Livox. |
| `tight_warmup_scans` | Loose scans before engaging the IMU, to measure the velocity init cannot observe (§3.11). |
| `gravity_sigma` | Stiffness of the gravity prior, anchored to the fixed init value (§3.11). Default `0.05` — an estimator default, not exposed as a ROS parameter. Loosen it and gravity wanders — the catastrophic-divergence bug. |
| `imu.gyro_noise`, `imu.accel_noise` | Noise densities. Physics, from the datasheet — not knobs. |
| `imu.bias_rw_gyro`, `imu.bias_rw_accel` | How fast a bias may physically drift. Stops the biases absorbing real motion when they are unobservable. |
| `imu.bias_sigma0_gyro`, `imu.bias_sigma0_accel` | How wrong the *starting* bias may be — a different question from the random walk. Too tight and the accel bias is frozen at zero. |
