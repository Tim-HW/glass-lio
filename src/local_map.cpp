#include "lidar_odom/local_map.hpp"

#include <cmath>

namespace lidar_odom
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
    auto & bucket = voxels_[keyOf(p)];
    if (bucket.size() < max_points_per_voxel_) {
      bucket.emplace_back(p.cast<float>());
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

std::size_t LocalMap::num_points() const
{
  std::size_t n = 0;
  for (const auto & [key, bucket] : voxels_) {
    n += bucket.size();
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
  for (const auto & [key, bucket] : voxels_) {
    for (const auto & p : bucket) {
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

}  // namespace lidar_odom
