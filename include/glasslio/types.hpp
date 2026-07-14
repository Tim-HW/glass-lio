#ifndef GLASSLIO_TYPES_HPP
#define GLASSLIO_TYPES_HPP

#include <vector>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "glasslio/livox_point.hpp"

namespace glasslio
{

/// The shared vocabulary of the pipeline -- the handful of types that more than one stage
/// needs to speak about.
///
/// These used to live in deskew.hpp, which meant local_map.hpp had to include the DESKEW
/// STAGE just to name a point cloud, and sync.hpp had to include it to name a MeasureGroup
/// -- even though sync PRODUCES the MeasureGroup that deskew CONSUMES. The dependency ran
/// backwards. Vocabulary belongs in a header that depends on nothing.

/// Raw Livox points, carrying the per-point `timestamp` the deskew needs.
typedef pcl::PointCloud<LivoxPoint> LivoxCloud;

/// Everything downstream of deskew. Once motion compensation has consumed the per-point
/// time, `timestamp`/`tag`/`line` are dead weight -- and the plain type lets the rest of
/// the pipeline use PCL's precompiled filters.
typedef pcl::PointCloud<pcl::PointXYZI> CloudXYZI;

/// [2] -> [3]. One LiDAR scan together with the IMU samples covering it, WITH a bracket
/// sample just before the scan starts and one just after it ends.
///
/// This is the product of stage [2] (MeasureSync) and the input to stage [3] (Deskew). The
/// brackets are not optional: GyrInt CLAMPS outside its knot range rather than
/// extrapolating, so a point outside the IMU coverage is silently left uncorrected.
struct MeasureGroup
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu;
};

}  // namespace glasslio

#endif  // GLASSLIO_TYPES_HPP
