#include "glasslio/tight_registration.hpp"

#include <cmath>

#include "glass_core/gauss_newton.hpp"
#include "glass_core/nav_residual.hpp"

namespace glasslio
{

// The augmented state is [nav (15) ; gravity (3)] -- 18 DoF. Gravity is a free world-frame
// vector estimated alongside the pose, so an initial tilt error can be corrected instead of
// frozen at init (roadmap Phase 1).
using NavEquations = NormalEquationsN<kTightDim>;

// predictState() moved to glass_core/nav_residual.hpp, beside the imuResidual it is the
// forward dual of. It is engine, not LiDAR, and glassvio needs it too.

TightResult alignTightlyCoupled(
  const CloudXYZI & source,
  const LocalMap & map,
  const NavState & xi,
  const ImuPreintegration & pre,
  const Eigen::Vector3d & gravity,
  const NavState & guess,
  const Eigen::Matrix<double, 6, 6> & bias_information,
  const Eigen::Matrix3d & gravity_information,
  const TightParams & params)
{
  TightResult result;
  result.state = guess;
  result.gravity = gravity;

  if (map.empty() || source.empty()) {
    return result;
  }

  // --- Information matrices, built once. -------------------------------------
  //
  // The IMU factor is weighted by the INVERSE of the covariance preintegration
  // accumulated. This is what makes the fusion self-tuning: a longer gap between scans
  // means a bigger Sigma, hence a smaller Sigma^-1, hence less pull from the IMU. No
  // heuristic decides that -- the propagated uncertainty does.
  Eigen::Matrix<double, 9, 9> imu_information =
    pre.covariance().inverse() * params.imu_prior_weight;
  if (!imu_information.allFinite()) {
    imu_information.setZero();   // a singular covariance means we learned nothing
  }

  // The bias prior comes from the CALLER, which carries a covariance and updates it with
  // what each solve learns. It deliberately does NOT come from a random-walk constant:
  // a random walk says how fast a bias may DRIFT, never how wrong it might have been to
  // begin with -- and an accel bias is exactly a quantity you start out wrong about.

  NavState x = guess;
  Eigen::Vector3d g = gravity;   // the gravity iterate; `gravity` is now the prior anchor

  for (int iter = 0; iter < params.max_iterations; ++iter) {
    NavEquations eq;

    // --- 1. LiDAR: one scalar residual per correspondence, Huber-weighted.
    //
    // WHITENED by lidar_sigma. Dividing the residual and its Jacobian by sigma gives
    // each point an information of 1/sigma^2 -- which is what puts it on a common
    // footing with the IMU's Sigma^-1. Without this the two sensors are being compared
    // in different units and the weighting is meaningless. (Huber's threshold is
    // whitened too, so `huber_delta` keeps its natural units of metres.)
    const double inv_sigma = 1.0 / params.lidar_sigma;
    const double huber_whitened = params.huber_delta * inv_sigma;

    double lidar_sq_err = 0.0;
    int n_corr = 0;
    for (const auto & pt : source.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      const Eigen::Vector3d p_sensor(pt.x, pt.y, pt.z);
      const Eigen::Vector3d q = x.R * p_sensor + x.p;   // into the world

      Plane plane;
      if (!map.closestPlane(q, params.max_correspondence_distance, plane)) {
        continue;
      }

      const double r = pointToPlaneResidualNav(x, p_sensor, plane.centroid, plane.normal);
      // Widen to the augmented state: a laser return sees the pose, not gravity, so its
      // gravity columns are structurally zero.
      Eigen::Matrix<double, 1, kTightDim> Jl = Eigen::Matrix<double, 1, kTightDim>::Zero();
      Jl.leftCols<kNavDim>() = pointToPlaneJacobianNav(x, p_sensor, plane.normal);
      eq.addScalar(r * inv_sigma, Jl * inv_sigma, huber_whitened);

      lidar_sq_err += r * r;   // raw, so the reported rmse stays in metres
      ++n_corr;
    }

    result.correspondences = n_corr;
    if (n_corr < params.min_correspondences) {
      result.valid = false;   // under-constrained: refuse rather than invent a pose
      return result;
    }

    // --- 2. The IMU factor: one 9-vector. It is the ONLY thing that couples gravity to the
    //        rest of the state (through the dv/dp residuals), so its gravity columns come
    //        from imuGravityJacobian. Residual uses the CURRENT gravity iterate `g`.
    Eigen::Matrix<double, 9, kTightDim> Jimu = Eigen::Matrix<double, 9, kTightDim>::Zero();
    Jimu.leftCols<kNavDim>() = imuJacobian(xi, x, pre);
    Jimu.rightCols<3>() = imuGravityJacobian(xi, pre);
    eq.addBlock<9>(imuResidual(xi, x, pre, g), Jimu, imu_information);

    // --- 3. The bias random-walk prior (gravity columns zero: biases don't see gravity).
    Eigen::Matrix<double, 6, kTightDim> Jb = Eigen::Matrix<double, 6, kTightDim>::Zero();
    Jb.leftCols<kNavDim>() = biasJacobian();
    eq.addBlock<6>(biasResidual(xi, x), Jb, bias_information);

    // --- 4. Gravity prior: anchor g to the carried estimate. The IMU factor is the only
    //        thing that touches gravity and per scan that is a weak constraint, so without
    //        this anchor gravity wanders. r_g = g - g_prior; Jacobian is I on the gravity
    //        block. gravity_information (stiffness) decides how far the data may move it.
    Eigen::Matrix<double, 3, kTightDim> Jg = Eigen::Matrix<double, 3, kTightDim>::Zero();
    Jg.block<3, 3>(0, kIdxGrav) = Eigen::Matrix3d::Identity();
    eq.addBlock<3>(g - gravity, Jg, gravity_information);

    // --- Solve and retract. The increment is over the 18-DoF augmented state.
    const Eigen::Matrix<double, kTightDim, 1> dx = eq.solve();
    if (!dx.allFinite()) {
      result.valid = false;
      return result;
    }

    x = boxplus(x, dx.head<kNavDim>());   // nav: RIGHT perturbation on R, additive elsewhere
    g += dx.segment<3>(kIdxGrav);         // gravity: additive in R^3
    result.H = eq.H();    // hand the information back so the caller can shrink its P

    result.iterations = iter + 1;
    result.valid = true;
    // Report the LiDAR-only RMSE, so it is directly comparable with the loose path's.
    result.rmse = std::sqrt(lidar_sq_err / static_cast<double>(n_corr));

    if (dx.segment<3>(kIdxPos).norm() < params.eps_translation &&
      dx.segment<3>(kIdxPhi).norm() < params.eps_rotation)
    {
      result.converged = true;
      break;
    }
  }

  result.state = x;
  result.gravity = g;
  return result;
}

}  // namespace glasslio
