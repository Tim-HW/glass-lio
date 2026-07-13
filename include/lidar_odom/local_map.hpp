#ifndef LIDAR_ODOM_LOCAL_MAP_HPP
#define LIDAR_ODOM_LOCAL_MAP_HPP

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "lidar_odom/data_process.h"  // CloudXYZI

namespace lidar_odom
{

/// Integer voxel coordinate.
struct VoxelKey
{
  std::int32_t x, y, z;
  bool operator==(const VoxelKey & o) const {return x == o.x && y == o.y && z == o.z;}
};

/// Spatial hash (Teschner et al. 2003) — three large primes, xor-combined.
struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & k) const
  {
    return (static_cast<std::size_t>(k.x) * 73856093u) ^
           (static_cast<std::size_t>(k.y) * 19349669u) ^
           (static_cast<std::size_t>(k.z) * 83492791u);
  }
};

/// Sliding local map: the accumulated world geometry that each new scan is
/// registered against. Backed by a voxel hash, which gives O(1) insert and
/// prune, and caps density (max_points_per_voxel) without ever re-quantizing
/// points that are already stored.
///
/// It does not answer nearest-neighbour queries: PCL's GICP builds its own
/// KD-tree over target(). This class owns *storage*, not the spatial index.
class LocalMap
{
public:
  LocalMap(double voxel_size, int max_points_per_voxel, double max_range);

  /// Add a scan already transformed into the world frame. Points landing in a
  /// full voxel are dropped — that cap is the density control.
  void insert(const CloudXYZI & cloud_world);

  /// Drop voxels further than max_range from `origin` (the current pose).
  /// Bounds both memory and the cost of the KD-tree GICP builds over target().
  void prune(const Eigen::Vector3d & origin);

  /// The map as a point cloud, for registration and for RViz. Cached; rebuilt
  /// only after a mutation.
  CloudXYZI::ConstPtr target() const;

  bool empty() const {return voxels_.empty();}
  std::size_t num_voxels() const {return voxels_.size();}
  std::size_t num_points() const;

  VoxelKey keyOf(const Eigen::Vector3d & p) const;
  Eigen::Vector3d voxelCenter(const VoxelKey & k) const;

private:
  double voxel_size_;
  std::size_t max_points_per_voxel_;
  double max_range_;

  std::unordered_map<VoxelKey, std::vector<Eigen::Vector3f>, VoxelKeyHash> voxels_;

  mutable CloudXYZI::Ptr target_cache_;
  mutable bool dirty_ = true;
};

}  // namespace lidar_odom

#endif  // LIDAR_ODOM_LOCAL_MAP_HPP
