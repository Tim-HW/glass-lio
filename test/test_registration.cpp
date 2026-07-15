// Self-check for point-to-plane ICP. Builds a synthetic room, perturbs the pose
// by a KNOWN transform, and asserts the solver recovers it.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>

#include "glasslio/local_map.hpp"
#include "glasslio/registration.hpp"

using namespace glasslio;

static void add(CloudXYZI & c, double x, double y, double z)
{
  pcl::PointXYZI p;
  p.x = static_cast<float>(x);
  p.y = static_cast<float>(y);
  p.z = static_cast<float>(z);
  p.intensity = 0.0f;
  c.push_back(p);
}

/// A closed box: floor, ceiling and four walls. Closed matters -- an open scene
/// leaves a direction unconstrained and ICP can slide along it.
static CloudXYZI makeRoom()
{
  CloudXYZI c;
  const double L = 10.0, H = 3.0, step = 0.15;
  for (double x = -L; x <= L; x += step) {
    for (double y = -L; y <= L; y += step) {
      add(c, x, y, 0.0);      // floor
      add(c, x, y, H);        // ceiling
    }
  }
  for (double x = -L; x <= L; x += step) {
    for (double z = 0.0; z <= H; z += step) {
      add(c, x, -L, z);       // wall
      add(c, x, L, z);
    }
  }
  for (double y = -L; y <= L; y += step) {
    for (double z = 0.0; z <= H; z += step) {
      add(c, -L, y, z);       // wall
      add(c, L, y, z);
    }
  }
  return c;
}

static CloudXYZI transformed(const CloudXYZI & in, const Eigen::Isometry3d & T)
{
  CloudXYZI out;
  out.reserve(in.size());
  for (const auto & p : in.points) {
    const Eigen::Vector3d q = T * Eigen::Vector3d(p.x, p.y, p.z);
    add(out, q.x(), q.y(), q.z());
  }
  return out;
}

static Eigen::Isometry3d makePose(double tx, double ty, double tz, double yaw_deg)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw_deg * M_PI / 180.0,
    Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(tx, ty, tz);
  return T;
}

int main()
{
  const CloudXYZI room = makeRoom();

  LocalMap map(0.5, 20, 1000.0);
  map.insert(room);                       // map is the room, in world coords
  assert(!map.empty());
  assert(map.num_planes() > 100);         // floor/walls must yield real planes

  RegistrationParams params;
  params.max_correspondence_distance = 1.0;
  params.max_iterations = 50;

  // --- Identity: a scan already in place must not move.
  {
    const auto r = alignPointToPlane(room, map, Eigen::Isometry3d::Identity(), params);
    assert(r.converged);
    assert(r.pose.translation().norm() < 0.02);
    assert(r.rmse < 0.05);
  }

  // --- Recover a known translation.
  // The sensor sits at `true_pose`; the scan it observes is the room expressed in
  // the SENSOR frame, i.e. true_pose^-1 * room. Registration must return true_pose.
  {
    const Eigen::Isometry3d truth = makePose(0.4, -0.3, 0.05, 0.0);
    const CloudXYZI scan = transformed(room, truth.inverse());

    const auto r = alignPointToPlane(scan, map, Eigen::Isometry3d::Identity(), params);
    assert(r.converged);
    const double err = (r.pose.translation() - truth.translation()).norm();
    assert(err < 0.05);
  }

  // --- Recover a known rotation + translation.
  {
    const Eigen::Isometry3d truth = makePose(0.3, 0.2, 0.0, 5.0);   // 5 deg yaw
    const CloudXYZI scan = transformed(room, truth.inverse());

    const auto r = alignPointToPlane(scan, map, Eigen::Isometry3d::Identity(), params);
    assert(r.converged);
    assert((r.pose.translation() - truth.translation()).norm() < 0.08);

    const Eigen::Matrix3d dR = r.pose.linear().transpose() * truth.linear();
    const double ang_err = std::abs(Eigen::AngleAxisd(dR).angle()) * 180.0 / M_PI;
    assert(ang_err < 1.0);   // within a degree
  }

  // --- A good guess must be preserved, not wrecked.
  {
    const Eigen::Isometry3d truth = makePose(0.5, 0.5, 0.0, 3.0);
    const CloudXYZI scan = transformed(room, truth.inverse());
    const auto r = alignPointToPlane(scan, map, truth, params);   // start AT truth
    assert(r.converged);
    assert((r.pose.translation() - truth.translation()).norm() < 0.03);
  }

  // --- Outliers must not drag the solution (Huber).
  {
    const Eigen::Isometry3d truth = makePose(0.3, 0.0, 0.0, 0.0);
    CloudXYZI scan = transformed(room, truth.inverse());
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> junk(-8.0, 8.0);
    for (int i = 0; i < 500; ++i) {           // ~5% garbage points mid-air
      add(scan, junk(rng), junk(rng), 1.5);
    }
    const auto r = alignPointToPlane(scan, map, Eigen::Isometry3d::Identity(), params);
    assert(r.converged);
    assert((r.pose.translation() - truth.translation()).norm() < 0.10);
  }

  // --- Empty map: must refuse, not invent a pose.
  {
    LocalMap empty(0.5, 20, 100.0);
    const auto r = alignPointToPlane(room, empty, Eigen::Isometry3d::Identity(), params);
    assert(!r.valid);
  }

  // --- REGRESSION: a map fed a cloud sparser than its own voxels fits NO planes,
  // so ICP is permanently under-constrained. This deadlocked the node: sparse map
  // -> ICP fails -> scan not inserted -> map stays sparse -> forever.
  // The map voxel MUST be coarser than the density of what is inserted.
  {
    // One point per 0.5 m voxel (what a 0.5 m downsample leaf produces)...
    CloudXYZI sparse;
    for (double x = -10; x <= 10; x += 0.5) {
      for (double y = -10; y <= 10; y += 0.5) {
        add(sparse, x, y, 0.0);
      }
    }
    LocalMap too_fine(0.5, 20, 1000.0);      // map voxel == point spacing -> BAD
    too_fine.insert(sparse);
    assert(too_fine.num_planes() == 0);      // this is the deadlock condition

    LocalMap coarse(1.5, 20, 1000.0);        // map voxel > point spacing -> planes
    coarse.insert(sparse);
    assert(coarse.num_planes() > 50);
  }

  // --- Max-iterations reached is NOT failure: the pose must still be usable.
  {
    const Eigen::Isometry3d truth = makePose(0.3, 0.2, 0.0, 4.0);
    const CloudXYZI scan = transformed(room, truth.inverse());
    RegistrationParams tight = params;
    tight.max_iterations = 2;                // force it to run out of iterations
    tight.eps_translation = 1e-12;
    tight.eps_rotation = 1e-12;
    const auto r = alignPointToPlane(scan, map, Eigen::Isometry3d::Identity(), tight);
    assert(!r.converged);                    // did not settle below eps...
    assert(r.valid);                         // ...but the pose is still usable
    assert(r.correspondences > 100);
  }

  std::printf("test_registration: all assertions passed\n");
  return 0;
}
