# [3] Register — producing the pose

Scan-to-map **point-to-plane ICP**, solved by Gauss-Newton on SE(3). This is the
stage that produces `pose_`; everything else feeds it or consumes it.

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
comparison in [pipeline.md](pipeline.md#performance).

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

Velocity is currently **finite-differenced from the very poses it helps produce** — a
feedback loop by construction. You are carrying an accelerometer, a device that
measures acceleration directly. Only a *tightly-coupled* estimator can use it, and
doing so retires this parameter entirely. See [pipeline.md](pipeline.md#not-implemented).

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
conditioning — see [7-tight-coupling.md §7.9](7-tight-coupling.md) for how the two
interact.

## 3.7 Parameters that bite

| Param | Why |
|---|---|
| `max_correspondence_distance` | **The key knob.** Too small: fast motion puts the true correspondence out of reach and it never converges. Too large: it matches the wrong wall, and re-opens the runaway. |
| `max_rmse` | The coast threshold (§3.6). |
| `huber_delta` | Robust threshold — see [gauss_newton.md](gauss-newton.md#robust-weighting). |
| `min_correspondences` | Below this the problem is under-constrained; refuse. |
| `min_translation_eigenvalue_ratio` | The degeneracy gate (§3.6.1). Below this, some translation direction has essentially no normal support; refuse regardless of RMSE. **Loose path only.** |
| `registration.min_rotation_eigenvalue_ratio` | Tight path's rotational counterpart — see [7-tight-coupling.md §7.9](7-tight-coupling.md). Gates deskew's bias resync, not the pose solve. |
| `max_iterations` | Cost ceiling. Hitting it is not failure. |
| `use_constant_velocity` | §3.5. |
