cmake_minimum_required(VERSION 3.8)
project(lidar-odom)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(lio_node
  src/lio_node.cpp
  src/data_process.cpp
  src/gyr_int.cpp
  src/imu_init.cpp
  src/local_map.cpp)
ament_target_dependencies(lio_node
  rclcpp sensor_msgs nav_msgs geometry_msgs pcl_conversions tf2_ros tf2_eigen)
target_link_libraries(lio_node ${PCL_LIBRARIES})

install(TARGETS lio_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS scripts/run_bag.sh
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Plain assert-based self-checks; no test framework needed.
  #
  # CRITICAL: -UNDEBUG. Release builds define NDEBUG, which compiles assert()
  # out entirely -- the tests would then pass while checking nothing at all.
  add_executable(test_local_map test/test_local_map.cpp src/local_map.cpp)
  ament_target_dependencies(test_local_map rclcpp sensor_msgs pcl_conversions)
  target_link_libraries(test_local_map ${PCL_LIBRARIES})
  target_compile_options(test_local_map PRIVATE -UNDEBUG)
  add_test(NAME test_local_map COMMAND test_local_map)

  add_executable(test_imu_init test/test_imu_init.cpp src/imu_init.cpp)
  ament_target_dependencies(test_imu_init rclcpp sensor_msgs)
  target_link_libraries(test_imu_init Eigen3::Eigen)
  target_compile_options(test_imu_init PRIVATE -UNDEBUG)
  add_test(NAME test_imu_init COMMAND test_imu_init)
endif()

ament_package()
