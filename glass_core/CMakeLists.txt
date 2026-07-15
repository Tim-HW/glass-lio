cmake_minimum_required(VERSION 3.8)
project(glass_core LANGUAGES CXX VERSION 0.0.0)

# Pure CMake. No ROS, no ament -- the estimation engine builds and is usable
# outside any ROS workspace. Its only dependency is Eigen. The one sensor-facing
# class (ImuInit) speaks a plain ImuSample struct, not sensor_msgs, so callers
# convert at their own boundary.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)

# Almost entirely header-only (the Gauss-Newton solver, the SO(3) Jacobians, the
# nav state, the IMU factor). Only preintegration and IMU init carry a .cpp.
#
# STATIC: each front-end (glasslio, glassvio) pulls this in via add_subdirectory and
# compiles its own copy, so there is no shared .so to gain -- and a static lib bakes
# straight into the consumer's liblio.so, sparing us a runtime shared-object to install
# and locate. POSITION_INDEPENDENT_CODE because that consumer IS a shared library.
add_library(glass_core STATIC
  src/preintegration.cpp
  src/imu_init.cpp)
add_library(glass_core::glass_core ALIAS glass_core)
set_target_properties(glass_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(glass_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(glass_core PUBLIC Eigen3::Eigen)
target_compile_features(glass_core PUBLIC cxx_std_17)

# In this repo glass_core is pulled in with add_subdirectory() by each front-end
# (glasslio, glassvio) -- it is NOT a colcon package. The install/export below is only
# for building glass_core STANDALONE (cmake .. && make install) so an out-of-tree
# project could `find_package(glass_core)`. When add_subdirectory'd it must stay silent,
# or it pollutes the parent's install and export set.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  include(CMakePackageConfigHelpers)
  install(TARGETS glass_core
    EXPORT glass_coreTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)
  install(DIRECTORY include/ DESTINATION include)   # ships glass_core/*.hpp AND vendored sophus/
  install(EXPORT glass_coreTargets
    FILE glass_coreTargets.cmake
    NAMESPACE glass_core::
    DESTINATION share/glass_core/cmake)
  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/glass_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/glass_coreConfig.cmake
    INSTALL_DESTINATION share/glass_core/cmake)
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/glass_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/glass_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/glass_coreConfigVersion.cmake
    DESTINATION share/glass_core/cmake)
endif()

# --- Tests: plain assert-based self-checks, no framework. -UNDEBUG is CRITICAL:
#     Release defines NDEBUG, which compiles assert() out, and the suites would
#     then pass while checking nothing at all.
option(GLASS_CORE_BUILD_TESTS "Build glass_core self-checks" ON)
if(GLASS_CORE_BUILD_TESTS)
  # When embedded via add_subdirectory the parent already enabled testing (ament does);
  # add_test still registers the engine's suites into the parent's ctest run.
  if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    enable_testing()
  endif()
  foreach(t test_preintegration test_nav_residual test_imu_init)
    add_executable(${t} test/${t}.cpp)
    target_link_libraries(${t} PRIVATE glass_core)
    target_compile_options(${t} PRIVATE -UNDEBUG)
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()
