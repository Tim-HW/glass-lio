// Self-check for the tightly-coupled solve.
//
// The claim under test is not "it runs" -- it is that stacking the IMU into the SAME
// normal equations buys something the LiDAR alone cannot have. So the centrepiece is a
// DEGENERATE scene (a corridor), where the LiDAR genuinely does not constrain motion
// along the axis, and the IMU must supply what the geometry cannot.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <random>

#include "glasslio/local_map.hpp"
#include "glasslio/registration.hpp"
#include "glasslio/tight_registration.hpp"

using namespace glasslio;
using namespace glass_core;
using Sophus::SO3d;

static const Eigen::Vector3d kG(0.0, 0.0, -9.80665);

/// Bias prior for the tests: tight enough that the biases stay put, since none of these
/// fixtures is trying to estimate them. (The node carries a real covariance instead.)
static Eigen::Matrix<double, 6, 6> biasInfo()
{
  return Eigen::Matrix<double, 6, 6>::Identity() * 1e8;
}

static void add(CloudXYZI & c, double x, double y, double z)
{
  pcl::PointXYZI p;
  p.x = static_cast<float>(x);
  p.y = static_cast<float>(y);
  p.z = static_cast<float>(z);
  p.intensity = 0.0f;
  c.push_back(p);
}

/// A CORRIDOR running along X: two side walls, a floor and a ceiling. Every surface
/// normal is perpendicular to X, so sliding along X changes no point-to-plane residual.
/// The LiDAR is blind to that one direction.
///
/// `x_from`/`x_to` matter more than they look. The MAP corridor must be strictly LONGER
/// than the scan's reach, or the scan can see the corridor's END -- and an end cap is an
/// X-facing surface, which makes X observable again and destroys the whole point of the
/// fixture. (This bit us: with map and scan both spanning +/-25 m, shifting the scan
/// exposed the ends, the terminating voxels fitted X-facing planes, and the LiDAR
/// acquired 1.2e6 of stiffness in the axis it was supposed to be blind to. Near-
/// degenerate is not degenerate.)
static CloudXYZI makeCorridor(double x_from, double x_to)
{
  CloudXYZI c;
  const double W = 2.0, H = 3.0, step = 0.15;
  for (double x = x_from; x <= x_to; x += step) {
    for (double z = 0.0; z <= H; z += step) {
      add(c, x, -W, z);      // left wall   (normal +/- Y)
      add(c, x, W, z);       // right wall
    }
    for (double y = -W; y <= W; y += step) {
      add(c, x, y, 0.0);     // floor       (normal +/- Z)
      add(c, x, y, H);       // ceiling
    }
  }
  return c;
}

/// A closed, CLUTTERED room: the corridor plus end caps plus a series of cross
/// partitions. Everything here is about the X-facing surface area.
///
/// The end caps alone are not enough, and the arithmetic is the lesson. Two caps supply
/// ~1100 X-facing points; at lidar_sigma = 0.05 that is ~2e5 of information in X --
/// almost exactly what a MEMS IMU supplies over half a second. Evenly matched sensors
/// produce a COMPROMISE, not a winner, so a scene with only end caps cannot demonstrate
/// "good geometry overrules a bad prior". It can only demonstrate a tie.
///
/// The partitions add ~11k more X-facing points, taking the LiDAR to ~20x the IMU's
/// information in that axis. NOW the geometry genuinely dominates, and the claim is
/// testable.
static CloudXYZI makeRoom()
{
  const double L = 25.0, W = 2.0, H = 3.0, step = 0.15;
  CloudXYZI c = makeCorridor(-L, L);

  for (double y = -W; y <= W; y += step) {
    for (double z = 0.0; z <= H; z += step) {
      add(c, -L, y, z);      // the end caps
      add(c, L, y, z);
    }
  }

  // Cross partitions every 5 m: dense X-facing surfaces. Spacing is kept well above
  // max_correspondence_distance so a partition cannot alias onto its neighbour.
  for (double x = -20.0; x <= 20.0; x += 5.0) {
    for (double y = -W; y <= W; y += 0.1) {
      for (double z = 0.0; z <= H; z += 0.1) {
        add(c, x, y, z);
      }
    }
  }
  return c;
}

/// SHUFFLE BEFORE INSERTING, and this is not cosmetic.
///
/// LocalMap caps each voxel at max_points_per_voxel and keeps the FIRST points that
/// arrive. Our corridor is generated in raster order (x outer), so an unshuffled insert
/// fills each voxel from only the first 2-3 x-slices: the retained points then span ~1 m
/// in y/z but only ~0.15 m in x. PCA duly reports the direction of least variance as X,
/// and the voxel's "plane" comes out with its normal along the corridor -- perpendicular
/// to the actual wall, and confidently through the planarity gate.
///
/// That gave the LiDAR 1.2e6 of spurious stiffness in the very axis this fixture needs
/// it to be blind to. A real Livox dodges this by accident: its non-repetitive scan
/// pattern delivers points in scattered order, so the first N in a voxel are roughly
/// representative. Any raster-ordered input hits it head-on.
static LocalMap buildMap(const CloudXYZI & cloud)
{
  CloudXYZI shuffled = cloud;
  std::mt19937 rng(1234);
  std::shuffle(shuffled.points.begin(), shuffled.points.end(), rng);

  LocalMap map(1.0, 20, 200.0);
  map.insert(shuffled);
  return map;
}

/// Synthesise the IMU an object would feel moving at constant velocity `v` (no rotation,
/// no acceleration). The accelerometer still reads the reaction to gravity: a = -g in
/// the body frame.
///
/// NOISE DENSITIES MATTER, and they are not free parameters to tune until the test goes
/// green. These are MEMS-grade -- the class of part actually inside a Livox. Plug in
/// navigation-grade numbers (accel ~1e-3) and the IMU's information over half a second
/// genuinely EXCEEDS a few thousand point-to-plane constraints, so it will win
/// disagreements against good geometry -- and it would be RIGHT to. Which sensor is
/// believed is decided by physics (the covariances), not by the estimator's preferences.
static ImuPreintegration constantVelocityImu(double duration)
{
  ImuPreintegration pre(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    1.7e-3,   // gyro noise  (rad/s/sqrt(Hz))   -- MEMS
    2.0e-2);  // accel noise (m/s^2/sqrt(Hz))   -- MEMS
  const double dt = 0.005;
  for (int k = 0; k < static_cast<int>(duration / dt); ++k) {
    pre.integrate(Eigen::Vector3d::Zero(), -kG, dt);   // specific force = -gravity
  }
  return pre;
}

// =================================================================================
// 1. THE POINT OF THE WHOLE EXERCISE.
//
//    Corridor. The robot really moved +1.0 m along X. The LiDAR cannot see that (X is a
//    null space of its Jacobian). Loose ICP is therefore free to leave the pose wherever
//    the guess put it. The IMU knows, and tight coupling must recover it.
// =================================================================================
static void testCorridorIsRescuedByImu()
{
  // The MAP corridor is long (+/-60 m); the SCAN only reaches +/-25 m. The sensor can
  // therefore never see an end, no matter how the pose slides -- X stays truly
  // unobservable to the LiDAR.
  const LocalMap map = buildMap(makeCorridor(-60.0, 60.0));
  const CloudXYZI visible = makeCorridor(-25.0, 25.0);

  const double dt = 0.5;
  const double true_dx = 1.0;               // travelled +1.0 m along the corridor
  const Eigen::Vector3d v(true_dx / dt, 0.0, 0.0);

  // The scan, as seen FROM the true pose (x = +1.0): the world, in sensor coordinates.
  CloudXYZI scan;
  for (const auto & pt : visible.points) {
    add(scan, pt.x - true_dx, pt.y, pt.z);
  }

  NavState xi;                              // previous state: at the origin, moving +X
  xi.v = v;

  const ImuPreintegration pre = constantVelocityImu(dt);

  // The IMU's own prediction: it integrates v*dt and lands at x = +1.0.
  const NavState guess = predictState(xi, pre, kG);
  assert(std::abs(guess.p.x() - true_dx) < 1e-6 && "prediction should already be right");

  // Now HANDICAP the guess: shove it 0.4 m back along the blind axis. The LiDAR cannot
  // pull it out (no residual changes); only the IMU factor can object.
  NavState bad = guess;
  bad.p.x() -= 0.4;

  TightParams tp;
  tp.imu_prior_weight = 1.0;
  const TightResult tight = alignTightlyCoupled(scan, map, xi, pre, kG, bad, biasInfo(), tp);
  assert(tight.valid);

  // --- The loose baseline, given the SAME handicapped guess.
  Eigen::Isometry3d loose_guess = Eigen::Isometry3d::Identity();
  loose_guess.linear() = bad.R.matrix();
  loose_guess.translation() = bad.p;
  RegistrationParams rp;
  const RegistrationResult loose = alignPointToPlane(scan, map, loose_guess, rp);
  assert(loose.valid);

  const double err_tight = std::abs(tight.state.p.x() - true_dx);
  const double err_loose = std::abs(loose.pose.translation().x() - true_dx);

  // The LiDAR is blind along X, so loose ICP simply keeps the 0.4 m error it was handed.
  assert(err_loose > 0.3 && "loose ICP should NOT be able to fix the blind axis");
  // The IMU can see it, so tight coupling must.
  assert(err_tight < 0.05 && "tight coupling failed to recover the unobservable axis");

  // And both must still nail the OBSERVABLE axes (the walls constrain Y and Z).
  assert(std::abs(tight.state.p.y()) < 0.02);
  assert(std::abs(tight.state.p.z()) < 0.02);

  std::printf(
    "  corridor (X unobservable) : loose err %.3f m -> tight err %.3f m   OK\n",
    err_loose, err_tight);
}

// =================================================================================
// 2. Tight coupling must not DAMAGE a well-conditioned scene. In a closed room the
//    LiDAR sees everything, and the fused answer should be at least as good.
// =================================================================================
static void testWellConditionedSceneStillWorks()
{
  const CloudXYZI room = makeRoom();
  const LocalMap map = buildMap(room);

  const double dt = 0.5;
  const double true_dx = 1.0;
  CloudXYZI scan;
  for (const auto & pt : room.points) {
    add(scan, pt.x - true_dx, pt.y, pt.z);
  }

  NavState xi;
  xi.v = Eigen::Vector3d(true_dx / dt, 0.0, 0.0);
  const ImuPreintegration pre = constantVelocityImu(dt);

  NavState bad = predictState(xi, pre, kG);
  bad.p.x() -= 0.3;
  bad.p.y() += 0.15;

  TightParams tp;
  const TightResult tight = alignTightlyCoupled(scan, map, xi, pre, kG, bad, biasInfo(), tp);

  assert(tight.valid);
  const double err = (tight.state.p - Eigen::Vector3d(true_dx, 0.0, 0.0)).norm();
  assert(err < 0.05 && "tight coupling degraded a well-conditioned scene");
  assert(tight.rmse < 0.15);

  std::printf(
    "  closed room (all observable): err %.3f m, rmse %.3f, %d iters   OK\n",
    err, tight.rmse, tight.iterations);
}

// =================================================================================
// 3. The IMU factor must not be free to override GOOD geometry. Feed a deliberately
//    WRONG IMU prediction into a well-conditioned room: the LiDAR should win, because
//    thousands of point-to-plane constraints carry far more information than one
//    9-vector.
// =================================================================================
static void testLidarWinsWhenGeometryIsStrong()
{
  const CloudXYZI room = makeRoom();
  const LocalMap map = buildMap(room);

  const double true_dx = 1.0;
  CloudXYZI scan;
  for (const auto & pt : room.points) {
    add(scan, pt.x - true_dx, pt.y, pt.z);
  }

  // A LYING IMU: it claims the robot barely moved (v = 0.2 m/s over 0.5 s = 0.1 m),
  // when it really moved 1.0 m.
  NavState xi;
  xi.v = Eigen::Vector3d(0.2, 0.0, 0.0);
  const ImuPreintegration pre = constantVelocityImu(0.5);

  const NavState guess = predictState(xi, pre, kG);
  TightParams tp;
  const TightResult tight = alignTightlyCoupled(scan, map, xi, pre, kG, guess, biasInfo(), tp);
  assert(tight.valid);

  // The geometry is unambiguous here, so it must dominate the bad prior.
  const double err = std::abs(tight.state.p.x() - true_dx);
  assert(err < 0.1 && "a wrong IMU prior overrode good geometry -- weighting is off");

  std::printf(
    "  lying IMU vs strong geometry: LiDAR wins, err %.3f m               OK\n", err);
}

int main()
{
  // Unbuffered: an assert() abort would otherwise discard the diagnostics we just
  // printed, which is precisely when you want to see them.
  std::setvbuf(stdout, nullptr, _IONBF, 0);

  std::printf("test_tight: tightly-coupled LiDAR-inertial solve\n");
  testCorridorIsRescuedByImu();
  testWellConditionedSceneStillWorks();
  testLidarWinsWhenGeometryIsStrong();
  std::printf("test_tight: all checks passed\n");
  return 0;
}
