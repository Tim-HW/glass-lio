#include "glasslio/local_map.hpp"

#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>

namespace glasslio
{

LocalMap::LocalMap(double voxel_size, int max_points_per_voxel, double max_range)
: voxel_size_(voxel_size),
  max_points_per_voxel_(static_cast<std::size_t>(max_points_per_voxel)),
  max_range_(max_range),
  target_cache_(new CloudXYZI())
{
}

VoxelKey LocalMap::keyOf(const Eigen::Vector3d & p) const
{
  // std::floor, NOT a cast: int(-0.3) == int(0.3) == 0 would fold the two
  // voxels straddling each axis origin into one.
  return {
    static_cast<std::int32_t>(std::floor(p.x() / voxel_size_)),
    static_cast<std::int32_t>(std::floor(p.y() / voxel_size_)),
    static_cast<std::int32_t>(std::floor(p.z() / voxel_size_))};
}

Eigen::Vector3d LocalMap::voxelCenter(const VoxelKey & k) const
{
  return {
    (static_cast<double>(k.x) + 0.5) * voxel_size_,
    (static_cast<double>(k.y) + 0.5) * voxel_size_,
    (static_cast<double>(k.z) + 0.5) * voxel_size_};
}

void LocalMap::insert(const CloudXYZI & cloud_world)
{
  for (const auto & pt : cloud_world.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    const Eigen::Vector3d p(pt.x, pt.y, pt.z);
    auto & v = voxels_[keyOf(p)];
    if (v.points.size() < max_points_per_voxel_) {
      v.points.emplace_back(p.cast<float>());
      v.dirty = true;   // plane must be refitted -- lazily, on first query
    }
  }
  dirty_ = true;
}

void LocalMap::prune(const Eigen::Vector3d & origin)
{
  const double max_sq = max_range_ * max_range_;
  for (auto it = voxels_.begin(); it != voxels_.end(); ) {
    if ((voxelCenter(it->first) - origin).squaredNorm() > max_sq) {
      it = voxels_.erase(it);
      dirty_ = true;
    } else {
      ++it;
    }
  }
}

void LocalMap::fitPlane(Voxel & v) const
{
  v.dirty = false;
  v.plane_ok = false;

  if (v.points.size() < kMinPointsForPlane) {
    return;
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & p : v.points) {
    centroid += p.cast<double>();
  }
  centroid /= static_cast<double>(v.points.size());

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto & p : v.points) {
    const Eigen::Vector3d d = p.cast<double>() - centroid;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(v.points.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  if (es.info() != Eigen::Success) {
    return;
  }
  // Eigenvalues ascending: [0] smallest. A plane means the spread ACROSS the
  // surface is tiny compared with the spread along it.
  const Eigen::Vector3d ev = es.eigenvalues();
  if (ev[1] < 1e-12 || ev[0] > kPlanarityRatio * ev[1]) {
    return;   // a blob or an edge, not a plane -- a normal here would be noise
  }

  v.plane.centroid = centroid;
  v.plane.normal = es.eigenvectors().col(0).normalized();
  v.plane_ok = true;
}

bool LocalMap::closestPlane(const Eigen::Vector3d & p, double max_dist, Plane & out) const
{
  const VoxelKey c = keyOf(p);
  const double max_sq = max_dist * max_dist;

  double best_sq = std::numeric_limits<double>::max();
  bool found = false;

  // The 27-cell neighbourhood. With voxel_size >= max_dist this is guaranteed to
  // contain every voxel that could hold a plane within max_dist of p.
  for (std::int32_t dx = -1; dx <= 1; ++dx) {
    for (std::int32_t dy = -1; dy <= 1; ++dy) {
      for (std::int32_t dz = -1; dz <= 1; ++dz) {
        const auto it = voxels_.find({c.x + dx, c.y + dy, c.z + dz});
        if (it == voxels_.end()) {
          continue;
        }
        Voxel & v = it->second;
        if (v.dirty) {
          fitPlane(v);   // lazy: only voxels actually queried get refitted
        }
        if (!v.plane_ok) {
          continue;
        }
        const double d_sq = (v.plane.centroid - p).squaredNorm();
        if (d_sq < best_sq && d_sq < max_sq) {
          best_sq = d_sq;
          out = v.plane;
          found = true;
        }
      }
    }
  }
  return found;
}

std::size_t LocalMap::num_points() const
{
  std::size_t n = 0;
  for (const auto & [key, v] : voxels_) {
    n += v.points.size();
  }
  return n;
}

std::size_t LocalMap::num_planes() const
{
  std::size_t n = 0;
  for (auto & [key, v] : voxels_) {
    if (v.dirty) {
      fitPlane(v);
    }
    if (v.plane_ok) {
      ++n;
    }
  }
  return n;
}

CloudXYZI::ConstPtr LocalMap::target() const
{
  if (!dirty_) {
    return target_cache_;
  }
  target_cache_->clear();
  target_cache_->reserve(num_points());
  for (const auto & [key, v] : voxels_) {
    for (const auto & p : v.points) {
      pcl::PointXYZI pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
      pt.intensity = 0.0f;
      target_cache_->push_back(pt);
    }
  }
  target_cache_->height = 1;
  target_cache_->width = static_cast<std::uint32_t>(target_cache_->size());
  target_cache_->is_dense = true;
  dirty_ = false;
  return target_cache_;
}

}  // namespace glasslio
