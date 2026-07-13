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
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(lio_node
  src/lio_node.cpp
  src/data_process.cpp
  src/gyr_int.cpp
  src/local_map.cpp)
ament_target_dependencies(lio_node
  rclcpp sensor_msgs nav_msgs geometry_msgs pcl_conversions)
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

  # Plain assert-based self-check; no test framework needed.
  add_executable(test_local_map test/test_local_map.cpp src/local_map.cpp)
  ament_target_dependencies(test_local_map rclcpp sensor_msgs pcl_conversions)
  target_link_libraries(test_local_map ${PCL_LIBRARIES})
  add_test(NAME test_local_map COMMAND test_local_map)
endif()

ament_package()
