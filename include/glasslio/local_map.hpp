#ifndef GLASSLIO_LOCAL_MAP_HPP
#define GLASSLIO_LOCAL_MAP_HPP

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "glasslio/data_process.h"  // CloudXYZI

namespace glasslio
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

/// A local plane fitted to the points of one voxel.
struct Plane
{
  Eigen::Vector3d centroid;
  Eigen::Vector3d normal;   ///< unit
};

/// Sliding local map: the accumulated world geometry each new scan registers
/// against. Backed by a voxel hash — O(1) insert and prune, density capped by
/// max_points_per_voxel, and points are never re-quantized once stored.
///
/// Each voxel caches a PLANE (centroid + normal, by PCA over its own points).
/// This is what makes registration cheap: the voxel *is* the neighbourhood, so
/// the correspondence search is a hash lookup over the 27-cell neighbourhood
/// rather than a KD-tree query, and planes are refitted only for voxels an
/// insert actually touched -- O(changed), not O(map).
class LocalMap
{
public:
  LocalMap(double voxel_size, int max_points_per_voxel, double max_range);

  /// Add a scan already transformed into the world frame. Points landing in a
  /// full voxel are dropped — that cap is the density control.
  void insert(const CloudXYZI & cloud_world);

  /// Drop voxels further than max_range from `origin` (the current pose).
  void prune(const Eigen::Vector3d & origin);

  /// Nearest valid plane to `p`, searching p's voxel and its 26 neighbours.
  /// Returns false if no voxel within `max_dist` holds a well-conditioned plane.
  bool closestPlane(const Eigen::Vector3d & p, double max_dist, Plane & out) const;

  /// The map as a point cloud, for RViz. Cached; rebuilt only after a mutation.
  CloudXYZI::ConstPtr target() const;

  bool empty() const {return voxels_.empty();}
  std::size_t num_voxels() const {return voxels_.size();}
  std::size_t num_points() const;
  /// Voxels currently holding a well-conditioned plane.
  std::size_t num_planes() const;

  VoxelKey keyOf(const Eigen::Vector3d & p) const;
  Eigen::Vector3d voxelCenter(const VoxelKey & k) const;

  /// Minimum points in a voxel before a plane is even attempted.
  static constexpr std::size_t kMinPointsForPlane = 5;
  /// Planarity gate: smallest eigenvalue must be << the middle one, else the
  /// points are a blob or an edge and the "plane" would be noise.
  static constexpr double kPlanarityRatio = 0.1;

private:
  struct Voxel
  {
    std::vector<Eigen::Vector3f> points;
    Plane plane;
    bool plane_ok = false;
    bool dirty = true;     ///< points changed since the plane was fitted
  };

  /// PCA over the voxel's points. Normal = eigenvector of the smallest
  /// eigenvalue. Clears plane_ok if the points are not planar enough.
  void fitPlane(Voxel & v) const;

  double voxel_size_;
  std::size_t max_points_per_voxel_;
  double max_range_;

  mutable std::unordered_map<VoxelKey, Voxel, VoxelKeyHash> voxels_;

  mutable CloudXYZI::Ptr target_cache_;
  mutable bool dirty_ = true;
};

}  // namespace glasslio

#endif  // GLASSLIO_LOCAL_MAP_HPP
