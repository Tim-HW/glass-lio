#ifndef GLASSLIO_LOCAL_MAP_HPP
#define GLASSLIO_LOCAL_MAP_HPP

#include <cstdint>
#include <random>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "glasslio/types.hpp"   // CloudXYZI

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
  /// `min_points_for_plane` : points a voxel needs before PCA is even attempted.
  ///                           PCA needs >= 3 to fit anything; 5 is a robustness margin.
  /// `planarity_ratio`      : the PLANARITY GATE. The smallest eigenvalue must be at
  ///                           most this fraction of the middle one, or the points are a
  ///                           blob (foliage) or an edge (a corner) and any "normal"
  ///                           fitted to them is noise.
  ///
  /// The gate is genuinely environment-dependent, which is why it is a parameter and not
  /// a constant: vegetation and clutter want it TIGHTER (blobs otherwise sneak through
  /// and inject garbage normals); sparse indoor scenes want it LOOSER, or the solve is
  /// starved of planes. It is the knob to reach for when the rmse looks fine but the pose
  /// is mushy.
  LocalMap(
    double voxel_size, int max_points_per_voxel, double max_range,
    int min_points_for_plane = kDefaultMinPointsForPlane,
    double planarity_ratio = kDefaultPlanarityRatio);

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

  /// Defaults, used when the caller does not care (tests, mostly).
  static constexpr int kDefaultMinPointsForPlane = 5;
  static constexpr double kDefaultPlanarityRatio = 0.1;

private:
  struct Voxel
  {
    std::vector<Eigen::Vector3f> points;
    /// Total points that have EVER landed here, including those we declined to keep.
    /// Reservoir sampling needs the true count, not the retained count.
    std::uint64_t seen = 0;
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
  std::size_t min_points_for_plane_;
  double planarity_ratio_;

  mutable std::unordered_map<VoxelKey, Voxel, VoxelKeyHash> voxels_;

  /// For reservoir sampling. Deterministically seeded: a map that reshuffles itself
  /// differently on every run is not reproducible, and an estimator you cannot
  /// reproduce is one you cannot debug. Touched only by the worker thread.
  mutable std::mt19937 rng_{12345};

  mutable CloudXYZI::Ptr target_cache_;
  mutable bool dirty_ = true;
};

}  // namespace glasslio

#endif  // GLASSLIO_LOCAL_MAP_HPP
