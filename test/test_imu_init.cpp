// Self-check for ImuInit. Values are the ones actually measured from the test
// bag (see memory: livox-imu-units-g), so this fails if the physics changes.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>

#include "lidar_odom/imu_init.hpp"

using lidar_odom::ImuInit;
using lidar_odom::kGravity;

// Bag defaults: 200 samples (1 s @ 200 Hz), 0.1 rad/s, 0.5 m/s^2, accel in g.
static ImuInit makeInit() {return ImuInit(200, 0.1, 0.5, kGravity);}

/// Build an Imu msg. accel is in *g* (as the Livox driver publishes it).
static sensor_msgs::msg::Imu msg(
  double gx, double gy, double gz, double ax_g, double ay_g, double az_g)
{
  sensor_msgs::msg::Imu m;
  m.angular_velocity.x = gx;
  m.angular_velocity.y = gy;
  m.angular_velocity.z = gz;
  m.linear_acceleration.x = ax_g;
  m.linear_acceleration.y = ay_g;
  m.linear_acceleration.z = az_g;
  return m;
}

int main()
{
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, 0.002);

  // Measured from the bag's first static second.
  const double bias[3] = {0.0034, -0.0013, 0.0026};      // rad/s
  const double grav_g[3] = {0.0699, -0.1063, 0.9892};    // g

  // --- Static window: converges, and recovers the planted bias + gravity.
  {
    auto init = makeInit();
    bool done = false;
    for (int i = 0; i < 200; ++i) {
      done = init.add(
        msg(
          bias[0] + noise(rng), bias[1] + noise(rng), bias[2] + noise(rng),
          grav_g[0] + noise(rng), grav_g[1] + noise(rng), grav_g[2] + noise(rng)));
    }
    assert(done);
    assert(init.initialized());
    assert(init.rejected_windows() == 0);

    // Gyro bias recovered to within the noise.
    assert(std::abs(init.gyro_bias().x() - bias[0]) < 0.001);
    assert(std::abs(init.gyro_bias().y() - bias[1]) < 0.001);
    assert(std::abs(init.gyro_bias().z() - bias[2]) < 0.001);

    // THE UNITS CHECK: raw accel is ~1.0 (g), so gravity must come out ~9.81 m/s^2.
    // If the g->SI scaling is ever dropped, this magnitude collapses to ~1.0.
    const double g_mag = init.gravity().norm();
    assert(g_mag > 9.5 && g_mag < 10.1);

    // Gravity alignment: rotating measured gravity by R_wi must land on +Z.
    const Eigen::Vector3d up = init.initial_rotation() * init.gravity().normalized();
    assert(std::abs(up.x()) < 1e-6);
    assert(std::abs(up.y()) < 1e-6);
    assert(std::abs(up.z() - 1.0) < 1e-6);

    // The bag's sensor is tilted ~7.3 deg from vertical; the init must see that.
    const double tilt_deg =
      std::acos(grav_g[2] / std::sqrt(
        grav_g[0] * grav_g[0] + grav_g[1] * grav_g[1] + grav_g[2] * grav_g[2])) * 180.0 / M_PI;
    const double recovered =
      std::acos(init.gravity().normalized().z()) * 180.0 / M_PI;
    assert(std::abs(recovered - tilt_deg) < 0.5);
    assert(recovered > 6.0 && recovered < 9.0);   // ~7.3 deg
  }

  // --- Rotating: window must be REJECTED, not silently accepted.
  {
    auto init = makeInit();
    for (int i = 0; i < 200; ++i) {
      init.add(msg(0.4, 0.0, 0.0, grav_g[0], grav_g[1], grav_g[2]));  // 0.4 rad/s turn
    }
    assert(!init.initialized());
    assert(init.rejected_windows() == 1);
  }

  // --- Translating: gyro is quiet but accel is noisy. A gyro-only check would
  // wrongly accept this; the accel std-dev check must catch it.
  {
    auto init = makeInit();
    std::normal_distribution<double> shake(0.0, 0.2);   // 0.2 g of jolt
    for (int i = 0; i < 200; ++i) {
      init.add(
        msg(
          bias[0], bias[1], bias[2],
          grav_g[0] + shake(rng), grav_g[1] + shake(rng), grav_g[2] + shake(rng)));
    }
    assert(!init.initialized());
    assert(init.rejected_windows() == 1);
  }

  // --- Motion then stillness: rejects the moving window, then initializes.
  {
    auto init = makeInit();
    for (int i = 0; i < 200; ++i) {
      init.add(msg(0.4, 0.0, 0.0, grav_g[0], grav_g[1], grav_g[2]));   // moving
    }
    assert(!init.initialized());

    bool done = false;
    for (int i = 0; i < 200; ++i) {
      done = init.add(
        msg(
          bias[0] + noise(rng), bias[1] + noise(rng), bias[2] + noise(rng),
          grav_g[0] + noise(rng), grav_g[1] + noise(rng), grav_g[2] + noise(rng)));
    }
    assert(done);
    assert(init.initialized());
    assert(init.rejected_windows() == 1);
  }

  // --- A perfectly level sensor needs no rotation.
  {
    auto init = makeInit();
    for (int i = 0; i < 200; ++i) {
      init.add(msg(0, 0, 0, 0.0, 0.0, 1.0));
    }
    assert(init.initialized());
    assert(init.initial_rotation().log().norm() < 1e-6);   // identity
  }

  std::printf("test_imu_init: all assertions passed\n");
  return 0;
}
