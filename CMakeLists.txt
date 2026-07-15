cmake_minimum_required(VERSION 3.8)
project(glasslio)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Emit compile_commands.json. .devcontainer/devcontainer.json points C++ IntelliSense at
# build/glasslio/compile_commands.json -- without this the file is never generated, so
# code completion silently does nothing and you have no idea why. clangd wants it too.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
# No `registration` component: PCL's GICP is gone, replaced by our own
# point-to-plane ICP over the voxel map (see doc/pipeline.md).
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# The sensor-agnostic estimation engine (SE(3) Gauss-Newton, IMU preintegration, the
# nav state + IMU factor, SO(3) Jacobians, IMU init). Pure CMake, no ROS -- pulled in
# from a subdirectory rather than find_package'd, so this monorepo builds in one shot
# and glassvio will share the exact same engine by including the same dir. It provides
# glass_core::glass_core and, transitively, the vendored Sophus headers.
add_subdirectory(glass_core)

# The pipeline stages, ROS-free apart from the message types they parse. Built
# once as a library so the node and the tests share the same objects instead of
# each re-listing (and re-compiling) the sources.
add_library(lio SHARED
  src/lio/deskew.cpp
  src/lio/gyr_int.cpp
  src/lio/lio_estimator.cpp
  src/lio/local_map.cpp
  src/lio/registration.cpp
  src/lio/sync.cpp
  src/lio/tight_registration.cpp)
target_include_directories(lio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lio rclcpp sensor_msgs pcl_conversions)
# glass_core is PUBLIC: its headers (engine + Sophus) appear in lio's own headers, so
# the node and tests that link lio must see them too. Plain link = public usage reqs.
target_link_libraries(lio ${PCL_LIBRARIES} Eigen3::Eigen glass_core::glass_core)

# The node is just the ROS shell: subscriptions, threading, publishing.
add_executable(glasslio_node src/glasslio_node.cpp)
ament_target_dependencies(glasslio_node
  rclcpp sensor_msgs nav_msgs geometry_msgs pcl_conversions tf2_ros)
target_link_libraries(glasslio_node lio ${PCL_LIBRARIES})

install(TARGETS lio
  DESTINATION lib)
install(TARGETS glasslio_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS scripts/run_bag.sh
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Plain assert-based self-checks; no test framework needed. Each links the `lio`
  # library rather than re-listing its sources.
  #
  # CRITICAL: -UNDEBUG. Release builds define NDEBUG, which compiles assert()
  # out entirely -- the tests would then pass while checking nothing at all.
  # test_imu_init, test_preintegration, test_nav_residual moved to glass_core -- they
  # test the engine, and now live with it. What stays here tests the LiDAR front-end
  # (and test_jacobian pins the point-to-plane Jacobian, which is glasslio's).
  foreach(t test_local_map test_registration test_jacobian test_tight test_sync)
    add_executable(${t} test/${t}.cpp)
    target_link_libraries(${t} lio ${PCL_LIBRARIES})
    target_compile_options(${t} PRIVATE -UNDEBUG)
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()

ament_package()
