#!/usr/bin/env python3
"""Worked answers for exercises.html; run after attempting the tasks yourself."""

import math


def add(a, b):
    """Add two vectors."""
    return tuple(x + y for x, y in zip(a, b))


def scale(s, v):
    """Scale a vector."""
    return tuple(s * x for x in v)


def dot(a, b):
    """Take a dot product."""
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    """Take the right-handed cross product."""
    x, y, z = a
    u, v, w = b
    return (y * w - z * v, z * u - x * w, x * v - y * u)


def rotate(v, axis, angle):
    """Apply Rodrigues' formula for a unit axis."""
    c, s = math.cos(angle), math.sin(angle)
    return add(add(scale(c, v), scale(s, cross(axis, v))),
               scale((1 - c) * dot(axis, v), axis))


def near(actual, expected, tol=1e-10):
    """Check a worked vector result to the stated absolute tolerance."""
    assert len(actual) == len(expected)
    assert max(abs(a - b) for a, b in zip(actual, expected)) < tol, (actual, expected)


def imu_exercise():
    """Compare direct integration with gravity-free deltas on the same discretization."""
    dt, yaw = 0.1, 0.0
    p = v = dp = dv = (0.0, 0.0, 0.0)
    gravity, specific_force = (0.0, 0.0, -9.81), (1.0, 0.0, 9.81)
    axis = (0.0, 0.0, 1.0)
    expected = [((0.005, 0, 0), (0.1, 0, 0)), ((0.015, 0.005, 0), (0.1, 0.1, 0))]
    for ep, ev in expected:
        rotated_force = rotate(specific_force, axis, yaw)
        acceleration = add(rotated_force, gravity)
        p = add(p, add(scale(dt, v), scale(0.5 * dt * dt, acceleration)))
        v = add(v, scale(dt, acceleration))
        dp = add(dp, add(scale(dt, dv), scale(0.5 * dt * dt, rotated_force)))
        dv = add(dv, scale(dt, rotated_force))
        yaw += math.pi / 2
        near(p, ep)
        near(v, ev)
    near(dp, (0.015, 0.005, 0.1962))
    near(dv, (0.1, 0.1, 1.962))
    near(add(dp, scale(0.5 * (2 * dt)**2, gravity)), p)
    near(add(dv, scale(2 * dt, gravity)), v)
    print('1. IMU: p =', p, '; v =', v, '; direct and preintegrated agree')


def jacobian_exercise():
    """Check both retractions and show that the sign-flipped derivative fails."""
    p, t, n = (2, 1, 0.5), (0.5, -0.25, 1), (0, 1, 0)
    axes = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    q = add(rotate(p, axes[2], math.pi / 2), t)
    left = n + cross(q, n)
    right = (0, 0.5, -1)
    near(q, (-0.5, 1.75, 1.5))
    near(left, (0, 1, 0, -1.5, 0, -0.5))
    for eps in (1e-5, 1e-6, 1e-7):
        translation, left_rotation, right_rotation = [], [], []
        for axis in axes:
            translation.append((dot(n, add(q, scale(eps, axis))) -
                                dot(n, add(q, scale(-eps, axis)))) / (2 * eps))
            left_rotation.append((dot(n, rotate(q, axis, eps)) -
                                  dot(n, rotate(q, axis, -eps))) / (2 * eps))
            plus = add(rotate(rotate(p, axis, eps), axes[2], math.pi / 2), t)
            minus = add(rotate(rotate(p, axis, -eps), axes[2], math.pi / 2), t)
            right_rotation.append((dot(n, plus) - dot(n, minus)) / (2 * eps))
        near(translation + left_rotation, left, 1e-7)
        near(right_rotation, right, 1e-7)
        assert abs(left_rotation[0] - cross(n, q)[0]) > 2.9
    print('2. Jacobians: both retractions agree with finite differences; wrong sign detected')


def degeneracy_exercise():
    """Show that a full-rank translation block can coexist with unobservable rotation."""
    h_diag = (40, 60, 100)
    ratio = min(h_diag) / sum(h_diag)
    assert abs(ratio - 0.2) < 1e-12
    for q in ((2, 0, 0), (0, 2, 0), (0, 0, 2), (1, 2, 3)):
        n = scale(1 / math.sqrt(dot(q, q)), q)
        near(cross(q, n), (0, 0, 0))
    print('3. Translation ratio = 0.2, but sphere normals give zero rotational columns')


def covariance_exercise():
    """Invert the two-variable example and contrast marginal and conditional variance."""
    a, b, d = 4.0, 2.0, 3.0
    determinant = a * d - b * b
    marginal = d / determinant
    schur = 1 / (a - b * b / d)
    assert abs(marginal - schur) < 1e-12
    assert marginal == 3 / 8
    assert 1 / a == 1 / 4 < marginal
    prior_information = 1.0
    assert 1 / prior_information == 1.0
    assert 1 / (prior_information + prior_information) == 0.5
    print('4. Marginal variance = 3/8; conditional = 1/4; repeated prior invents certainty')


def noise_exercise():
    """Evaluate the counterexample by direct matrix multiplication."""
    a, p = ((0, -1), (1, 0)), ((4, 0), (0, 1))
    propagated = [[sum(a[i][k] * p[k][col] * a[j][col]
                       for k in range(2) for col in range(2))
                   + (0.1 if i == j else 0) for j in range(2)] for i in range(2)]
    near(propagated[0], (1.1, 0))
    near(propagated[1], (0, 4.1))
    assert propagated[0][0] < p[0][0]
    assert abs((0.02 / 0.05)**2 - 0.16) < 1e-12
    print('5. Propagated covariance = diag(1.1, 4.1); LiDAR information scales by 0.16')


if __name__ == '__main__':
    imu_exercise()
    jacobian_exercise()
    degeneracy_exercise()
    covariance_exercise()
    noise_exercise()
    print('All five worked examples passed.')
