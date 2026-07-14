# Testing an estimator that never crashes

The most transferable thing in this repo is not the LiDAR-inertial odometry. It is the
method used to find the bugs in it.

**Every serious bug in this project produced plausible output.** Not one of them crashed,
NaN'd, or threw. A sign-flipped Jacobian still converges. A Jacobian that has silently lost
its first-order term still runs. A plane fitted perpendicular to the actual wall still
gives ICP something to chew on. A tightly-coupled estimator with a 3° gravity error still
tracks — for a while.

That is the defining hazard of estimator code, and it dictates everything below.

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

The analytic Jacobian in production is closed-form and exact:

$$
\mathbf{J}_i = \begin{bmatrix} \mathbf{n}_i^\top & (\mathbf{q}_i \times \mathbf{n}_i)^\top \end{bmatrix}
$$

The oracle perturbs the state and watches what *actually happens*:

$$
\mathbf{J}_{\text{num}}[i] \;=\; \frac{r\!\left(\boldsymbol{\xi} + h\,\mathbf{e}_i\right) - r\!\left(\boldsymbol{\xi} - h\,\mathbf{e}_i\right)}{2h}
$$

Two details that matter more than they look:

**Central differences, not forward.** The error is $O(h^2)$ rather than $O(h)$, which is
what lets us assert agreement to `1e-9` instead of squinting at `1e-4`.

**Perturb *through the retraction*, never by addition.** The state lives on a manifold, so
the perturbation must be applied the way the solver applies it —
$\operatorname{Exp}(h\,\mathbf{e}_i)$ — not by adding $h$ to a rotation matrix. In
[`test_nav_residual.cpp`](../test/test_nav_residual.cpp) the numeric Jacobian goes through
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
would we pass with it stubbed out? [`test_nav_residual.cpp`](../test/test_nav_residual.cpp)
replaces it with the identity matrix — the classic shortcut — and measures the damage:

```
Jr^-1 is load-bearing : with 5.5e-10, stubbed to I 2.5e-02      (7 orders of magnitude)
```

Without that check, you could ship the identity-matrix version and never know. It does not
NaN, it does not diverge — it just quietly mis-weights rotation, worst when you are rotating
hardest.

**Convergence-order assertions.** The bias correction claims to be *first-order accurate*
(error $O(\delta b^2)$). Don't assert a magic tolerance — assert the **order**: halve the
bias offset and the error must fall ~4×.

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

This is why `run_bag.sh` exists, and why "all tests pass" is never the last step here.

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

**Zero crashes. Nine bugs. Every one of them produced plausible output.**

## 11. The last one is the best one

The final row deserves its own note, because it is the only bug here that was introduced by
a **fix**.

The diagnosis was right: the accel bias was frozen (pinned 40× harder than the data that
would move it), so it was given a carried covariance and allowed to move. That is a locally
sensible change, it compiles, it converges, and **rejections went from 266 to 579.**

The reason is that the bias was loosened while the previous state `x_i` remained *infinitely
certain* — so every error that belonged to `x_i` got shovelled into the only slack in the
system. **You cannot fix a filter by loosening one block of a factor.**

> A change that is locally correct, produces no crash, and makes the estimator **worse** is
> the purest form of the hazard this whole document is about. It is also the reason the last
> word belongs to the **real bag** and not the test suite: no unit test was ever going to
> tell you that a *more principled* bias model made things worse. Only running it did.
