# The solver — Gauss-Newton on a manifold

Iterative least squares on SE(3). This file has **never heard of a point cloud**.

Code: [`gauss_newton.hpp`](../include/glasslio/gauss_newton.hpp).

It is split out from [5-registration.md](5-registration.md) on purpose, and the split is
the lesson: **swap the residual and the same solver becomes a different estimator.** It is
also the seam the IMU prior plugs into ([7-tight-coupling.md](7-tight-coupling.md)).

> **Notation.** Formulas render as LaTeX on GitHub and in VS Code's preview. Fenced code
> blocks are reserved for *algorithms* — those stay copy-pasteable and legible in a plain
> terminal.

---

## 1. The problem

Given a state $\mathbf{T} \in SE(3)$ and a pile of scalar residuals $r_i(\mathbf{T})$,
find the state minimising the (robustly weighted) sum of squares:

$$
\mathbf{T}^\star = \arg\min_{\mathbf{T}} \; F(\mathbf{T}), \qquad
F(\mathbf{T}) = \tfrac{1}{2}\sum_i w_i \, r_i(\mathbf{T})^2
$$

The caller supplies **only** $r_i$ and its Jacobian. Everything below is generic.

Two things make this harder than the least squares in a statistics textbook:

1. $r_i$ is **nonlinear** in $\mathbf{T}$ — there is no closed form, so we must iterate.
2. $\mathbf{T}$ lives on a **manifold**, not in $\mathbb{R}^n$ — so we cannot even *add* a
   correction to it (§6).

## 2. Why not the obvious methods

**Gradient descent.** Follow $-\nabla F$ downhill. It works, and it takes thousands of
iterations, because the gradient tells you *which way* is downhill but says **nothing about
how far to step**. You are left guessing a step size, and the guess is worst in exactly the
ill-conditioned directions you care about most.

**Newton's method.** Use the true curvature:

$$
\Delta \mathbf{x} = -\left(\nabla^2 F\right)^{-1}\nabla F
$$

Correct, and quadratically convergent — but $\nabla^2 F$ needs the **second** derivatives of
every residual. With thousands of correspondences that is expensive to derive *and*
expensive to evaluate.

**Gauss-Newton is the deal in between:** it gets Newton-like curvature **for free**, out of
Jacobians you already computed. Here is exactly how.

## 3. Deriving the normal equations

Linearise each residual about the current estimate. Write $\boldsymbol{\xi}$ for a small
increment (for now, think of it as a vector; §6 makes it a tangent vector):

$$
r_i(\boldsymbol{\xi}) \;\approx\; r_i + \mathbf{J}_i \boldsymbol{\xi},
\qquad
\mathbf{J}_i = \frac{\partial r_i}{\partial \boldsymbol{\xi}} \in \mathbb{R}^{1\times n}
$$

Substitute into the cost:

$$
F(\boldsymbol{\xi}) \;\approx\; \tfrac{1}{2}\sum_i w_i \left( r_i + \mathbf{J}_i\boldsymbol{\xi} \right)^2
$$

This is now **quadratic in $\boldsymbol{\xi}$** — and a quadratic can be minimised exactly.
Differentiate, set to zero:

$$
\frac{\partial F}{\partial \boldsymbol{\xi}}
= \sum_i w_i \mathbf{J}_i^\top\left(r_i + \mathbf{J}_i \boldsymbol{\xi}\right) = \mathbf{0}
$$

Rearranged, that is the **normal equations**:

$$
\boxed{\;\mathbf{H}\,\boldsymbol{\xi} = \mathbf{b}\;}
\qquad
\mathbf{H} = \sum_i w_i \,\mathbf{J}_i^\top \mathbf{J}_i,
\qquad
\mathbf{b} = -\sum_i w_i \,\mathbf{J}_i^\top r_i
$$

That is the whole of Gauss-Newton's bookkeeping, and it is precisely what
`NormalEquations::addScalar` accumulates, one residual at a time.

### The shape is the point

However many residuals you stack — 3 000 correspondences, 20 000 — $\mathbf{H}$ stays
**$n \times n$** (6×6 for SE(3); 15×15 tightly coupled). Each residual contributes a
**rank-1 outer product** $\mathbf{J}_i^\top\mathbf{J}_i$, folded into a fixed-size
accumulator. Memory is constant; only the accumulation is linear in the number of points.

Note what the two terms *are*: $\mathbf{b}$ is (minus) the **gradient**, and $\mathbf{H}$ is
an approximate **curvature**. So $\boldsymbol{\xi} = \mathbf{H}^{-1}\mathbf{b}$ is a Newton
step taken through an approximate Hessian.

## 4. What the approximation throws away

The *true* Hessian of $F$ is

$$
\nabla^2 F \;=\; \underbrace{\sum_i w_i \mathbf{J}_i^\top \mathbf{J}_i}_{\mathbf{H}\ \text{— what we keep}}
\;+\; \underbrace{\sum_i w_i\, r_i \, \nabla^2 r_i}_{\text{what we drop}}
$$

**Gauss-Newton drops the second term**, and the condition for that to be harmless can be
read straight off the formula: the dropped term is **scaled by $r_i$**, so it vanishes when
the **residuals are small at the solution**.

That is exactly our situation. A converged ICP fit has residuals of a few centimetres
against geometry metres across, so $r_i \nabla^2 r_i$ is negligible beside
$\mathbf{J}_i^\top\mathbf{J}_i$ — and we get Newton's curvature for the price of a Jacobian.

**When it stops being true:** a *large-residual* problem — badly mis-associated
correspondences, or a genuinely poor fit — makes the dropped term matter, and Gauss-Newton
can then take a step that **increases** the cost. That is not hypothetical: it is the exact
regime a bad initial guess puts you in, and it is why §8 exists.

## 5. Solving the system

```cpp
Vec solve() const {return H_.ldlt().solve(b_);}
```

**LDLT, and it is the right choice, not a coincidence.**
$\mathbf{H} = \sum_i w_i \mathbf{J}_i^\top \mathbf{J}_i$ is symmetric positive
semi-definite **by construction** — a sum of outer products, and for any $\mathbf{v}$

$$
\mathbf{v}^\top \mathbf{J}_i^\top \mathbf{J}_i \mathbf{v}
= \lVert \mathbf{J}_i\mathbf{v}\rVert^2 \;\ge\; 0
$$

so $\mathbf{H}$ can never be indefinite. LDLT exploits exactly that, and is faster and more
stable than a general LU. A general solver would be paying for a generality the problem
cannot exhibit.

If the geometry is degenerate, $\mathbf{H}$ is singular and the solve returns non-finite
values. We check `xi.allFinite()` and **refuse to return a state** rather than propagate a
NaN pose into the map.

### A null space is information, not noise

$$
\mathbf{H}\mathbf{v} = \mathbf{0}
\iff
\mathbf{J}_i \mathbf{v} = 0 \;\; \forall i
\iff
\text{moving along } \mathbf{v} \text{ changes no residual}
$$

In a long corridor, sliding along the axis changes no point-to-plane distance, so
$\mathbf{H}$ is genuinely **rank-deficient** there. The geometry really *does not* constrain
that DoF, and the solver is right to be unable to determine it.

This is precisely the hole a tightly-coupled IMU prior fills: add $\boldsymbol{\Sigma}^{-1}$
to $\mathbf{H}$ and the null space is **spanned by the IMU** rather than collapsing.
See [7-tight-coupling.md](7-tight-coupling.md).

## 6. The retraction — the actual Lie-algebra content

Here is where a naive optimizer goes wrong.

$\boldsymbol{\xi}$ is a 6-vector. $\mathbf{T}$ is a rigid transform on a **curved manifold**.
**You cannot add them.** $SE(3)$ is a *group*, not a vector space: add a delta to a rotation
matrix and the result is no longer a rotation matrix.

The exponential map is the bridge:

$$
\operatorname{Exp} : \mathbb{R}^6 \longrightarrow SE(3)
$$

Do the **linear** things — accumulate $\mathbf{H}$ and $\mathbf{b}$, solve
$\mathbf{H}\boldsymbol{\xi} = \mathbf{b}$ — in the tangent space $\mathfrak{se}(3)$, where
linear algebra is legal. Then map back onto the manifold and **compose**:

$$
\mathbf{T} \;\leftarrow\; \operatorname{Exp}(\boldsymbol{\xi}) \cdot \mathbf{T}
$$

Composition is a *group operation*, so the result is **exactly** an element of $SE(3)$ — no
drift off the manifold, no re-orthonormalisation, ever. (Contrast the pose bookkeeping in
the node, which *does* re-orthonormalise, because it round-trips through PCL's floats.)

### The perturbation convention

This solver linearises about a **left** perturbation,
$\mathbf{T} \leftarrow \operatorname{Exp}(\boldsymbol{\xi})\,\mathbf{T}$, with
$\boldsymbol{\xi} = [\boldsymbol{\rho};\, \boldsymbol{\phi}]$ (translation; rotation).

**Which side is not a matter of taste.** It is fixed by the frame the correction lives in:

| | Increment measured in | Composes on |
|---|---|---|
| `optimizeSE3` (this file) | the **world** frame | the **left** |
| `GyrInt` ([3-deskew.md](3-deskew.md)) | the **body** frame (it is a gyro) | the **right** |
| `NavState` ([7-tight-coupling.md](7-tight-coupling.md)) | the **body** frame | the **right** |

Same manifold, same $\operatorname{Exp}$, opposite side. Get it backwards and the estimator
**still runs, still converges, and is wrong** — no exception, no NaN, just a quietly
incorrect trajectory.

Consequently: **every Jacobian handed to `optimizeSE3` must be
$\partial r / \partial \boldsymbol{\xi}$ under the left perturbation.** That is a contract,
and it is pinned by finite differences in [`test_jacobian.cpp`](../test/test_jacobian.cpp) —
not by a comment.

## 7. Robust weighting

$$
w(r) =
\begin{cases}
1, & |r| \le \delta \qquad \text{(quadratic — ordinary least squares)}\\[6pt]
\dfrac{\delta}{|r|}, & |r| > \delta \qquad \text{(linear — the tail)}
\end{cases}
$$

Squared error grows **quadratically**, so a residual ten times too large contributes a
**hundred** times the pull. Moving objects, newly-seen geometry and mis-associations all
produce a handful of gross residuals — and without robust weighting those few terms would
dominate a solve over thousands of good ones.

The **Huber** weight is quadratic near zero (so good correspondences behave exactly as in
ordinary least squares) and linear in the tail — so an outlier's *influence*, $w(r)\cdot r$,
**saturates at $\delta$** instead of exploding:

$$
\lim_{|r| \to \infty} w(r)\, r = \delta
$$

`test_jacobian` asserts that saturation explicitly, across six orders of magnitude.

Implemented as **iteratively reweighted least squares** (IRLS): recompute $w$ from the
current residual each iteration and fold it into $\mathbf{H}$ and $\mathbf{b}$. Note this
means the cost being minimised **changes between iterations** — one more reason the
convergence guarantees below are soft.

## 8. Convergence — and what we deliberately do *not* do

Near the solution, with small residuals, Gauss-Newton converges **quadratically** — the
error roughly squares each iteration. That is why the solver reaches
$\lVert \log \mathbf{T}\rVert \approx 10^{-22}$ in **3 iterations** in
[`test_jacobian.cpp`](../test/test_jacobian.cpp).

**Far from the solution there is no guarantee at all.** Gauss-Newton is a *local* method: it
trusts its quadratic model completely and jumps to that model's minimum. If the model is bad
(§4), the step can overshoot and *increase* the cost.

There are two standard safeguards, and this solver has **neither**:

| Safeguard | What it does | Why we skip it |
|---|---|---|
| **Levenberg–Marquardt** | Solve $(\mathbf{H} + \lambda \mathbf{I})\boldsymbol{\xi} = \mathbf{b}$. Large $\lambda$ → a short, safe, gradient-descent-like step; small $\lambda$ → the fast Gauss-Newton step. Adapt $\lambda$ by whether the cost actually fell. | Every rejected step costs a re-solve — **and** a full cost re-evaluation (i.e. another association pass over every point) merely to discover it was rejected. |
| **Line search** | Take $\alpha\boldsymbol{\xi}$, shrinking $\alpha$ until the cost decreases. | Same problem: each trial $\alpha$ needs another residual evaluation over the whole cloud. |

**The bet we make instead:** ICP is handed a *good initial guess* by the IMU
([5-registration.md §3.1](5-registration.md)), so we start inside the quadratic basin where
plain Gauss-Newton is well-behaved — and we spend the compute budget on **more
correspondences** rather than on step control. Huber already suppresses the gross residuals
that would most damage the quadratic model.

**When the bet fails, the estimator does not thrash — it refuses:**

- fewer than `min_correspondences` → `valid = false`, coast on the prediction;
- a non-finite solve → `valid = false`;
- `rmse > max_rmse` → the pose is rejected upstream, and the scan is **not** inserted into
  the map ([5-registration.md §3.6](5-registration.md)).

> **Levenberg–Marquardt is the first thing to add** if the prior ever gets worse — a higher
> scan-drop rate, or a platform with a poorer IMU. Note the damping term
> $\lambda\mathbf{I}$ also **regularises a rank-deficient $\mathbf{H}$**, so it would
> additionally paper over the corridor null space of §5. But papering over it is strictly
> worse than *filling* it with real IMU information, which is what tight coupling does.

### The convergence test

$$
\text{converged} \iff
\lVert \boldsymbol{\rho} \rVert < \varepsilon_{t}
\;\;\wedge\;\;
\lVert \boldsymbol{\phi} \rVert < \varepsilon_{r}
$$

**Both**, because they carry **different units** — metres and radians. A single combined
threshold on $\lVert\boldsymbol{\xi}\rVert$ would be adding them, which is meaningless.

> **`converged` is not a trust signal.** The solver routinely plateaus above $\varepsilon$
> while sitting on a perfectly good fit; hitting `max_iterations` is a *cost* outcome, not a
> *quality* one. `valid` (enough residuals + a finite solve) is the trust signal. Treating
> `!converged` as failure threw away good poses and froze the estimator once already.

## 9. The interface

```cpp
GaussNewtonResult optimizeSE3(Sophus::SE3d & T,
                              const GaussNewtonParams & params,
                              Accumulate && accumulate);
```

`accumulate(T, eq)` is the **only** problem-specific part. It is called **once per
iteration** with the current estimate, and must push each residual and its Jacobian into
`eq`.

Re-running it every iteration is what makes this *iterative* — and for ICP, that
re-invocation is exactly the **re-association** step:

```
repeat until converged or max_iterations:
    accumulate:  ASSOCIATE   re-find correspondences at the current T
                 build H, b  from residuals + Jacobians
    solve:       xi = H^-1 b                 (LDLT)
    retract:     T <- Exp(xi) * T            (on the manifold)
```

**The outer ICP loop and the Gauss-Newton loop are the same loop.** That is not an
implementation shortcut — alternating association with solving *is* what ICP is.

## 10. Two ways in

`NormalEquations` is `NormalEquationsN<N>`, templated on the state dimension, with two entry
points — because the two sensors contribute **differently**:

```cpp
addScalar(r, J_1xN, huber_delta);            // LiDAR
addBlock<M>(r_M, J_MxN, information_MxM);    // IMU factor, bias prior
```

- **`addScalar`** — one scalar residual, robustly weighted. The LiDAR produces *thousands*,
  each individually suspect (a moving car, a mis-association). Hence Huber.
- **`addBlock`** — one *vector* residual with a full information matrix. The IMU factor is
  9 **correlated** numbers $(\delta\boldsymbol{\phi},\, \delta\mathbf{v},\, \delta\mathbf{p})$
  whose uncertainties are coupled — they cannot be weighted one at a time, you need the full
  $\boldsymbol{\Sigma}^{-1}$. And no Huber: a preintegrated IMU delta is not an outlier
  candidate the way a single laser return is.

**Both fold into the same $\mathbf{H}$ and $\mathbf{b}$ — and that sum *is* the sensor
fusion:**

$$
\mathbf{H} =
\underbrace{\sum_i \frac{1}{\sigma^2}\,\mathbf{J}_i^\top \mathbf{J}_i}_{\text{LiDAR}}
\;+\;
\underbrace{\mathbf{J}_{\text{imu}}^\top \boldsymbol{\Sigma}^{-1} \mathbf{J}_{\text{imu}}}_{\text{IMU}}
\;+\;
\underbrace{\mathbf{J}_b^\top \boldsymbol{\Omega}_b \mathbf{J}_b}_{\text{bias prior}}
$$

No filter, no blending coefficient — just Jacobians stacked into one linear system, each
weighted by how much it actually knows.

`using NormalEquations = NormalEquationsN<6>` keeps the LiDAR-only path exactly as it was.

### The punchline

**Loose coupling is tight coupling with $\boldsymbol{\Sigma}^{-1} = \mathbf{0}$.** Zero the
IMU's information and $\mathbf{H}$ collapses back to $\sum_i \mathbf{J}_i^\top\mathbf{J}_i$ —
literally the 6-DoF ICP. The two are the same estimator with one block zeroed, which is what
lets `imu_prior_weight` select between them on the same bag.

The **LDLT solve, the retraction, and the convergence test are unchanged** by any of it.
That is the whole reason the solver was split out of the ICP in the first place.
