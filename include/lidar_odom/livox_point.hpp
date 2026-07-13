#pragma once

#include <cstdint>

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_odom
{

/// PCL point matching the livox_ros_driver2 PointCloud2 layout
/// (xfer_format=0): x,y,z,intensity,tag,line,timestamp. The `timestamp` is the
/// absolute acquisition time of the point in seconds — this is what we deskew on.
struct EIGEN_ALIGN16 LivoxPoint
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint8_t tag;
  std::uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace lidar_odom

POINT_CLOUD_REGISTER_POINT_STRUCT(
  lidar_odom::LivoxPoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint8_t, tag, tag)
  (std::uint8_t, line, line)
  (double, timestamp, timestamp))
