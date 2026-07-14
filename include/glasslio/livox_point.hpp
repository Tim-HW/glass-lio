#ifndef GLASSLIO_LIVOX_POINT_HPP
#define GLASSLIO_LIVOX_POINT_HPP

#include <cstdint>

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace glasslio
{

/// PCL point matching the livox_ros_driver2 PointCloud2 layout
/// (xfer_format=0): x,y,z,intensity,tag,line,timestamp. The `timestamp` is the
/// absolute acquisition time of the point in NANOSECONDS — this is what we
/// deskew on. Header stamps are in seconds; mixing the two lands every point
/// ~1e9 away from the gyro knots, every lookup clamps, and the deskew silently
/// becomes a no-op. `intensity` is genuine reflectivity, NOT packed time.
struct EIGEN_ALIGN16 LivoxPoint
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint8_t tag;
  std::uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace glasslio

POINT_CLOUD_REGISTER_POINT_STRUCT(
  glasslio::LivoxPoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint8_t, tag, tag)
  (std::uint8_t, line, line)
  (double, timestamp, timestamp))

#endif  // GLASSLIO_LIVOX_POINT_HPP
