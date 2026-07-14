#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "glasslio/deskew.hpp"
#include "glasslio/imu_init.hpp"
#include "glasslio/local_map.hpp"
#include "glasslio/nav_state.hpp"
#include "glasslio/preintegration.hpp"
#include "glasslio/registration.hpp"
#include "glasslio/tight_registration.hpp"
#include "glasslio/ros_time.hpp"

namespace glasslio
{

/// LiDAR-inertial odometry node.
///
/// The pipeline, numbered as doc/pipeline.md numbers it:
///
///   [1] IMU init      gate: nothing runs until it completes   (callback side)
///   [2] sync          pair a scan with the IMU spanning it    (callback side)
///        |
///        v            <-- the MeasureGroup handed to the worker IS stages 1+2
///   [3] deskew        undo intra-scan rotation on SO(3)
///   [4] downsample    voxel grid, ICP source only            processOne(),
///   [5] register      point-to-plane ICP -> the pose         worker thread
///   [6] local map     insert the aligned scan; it is [5]'s target
///   [7] output        /odom + TF, debug clouds
///
/// processOne() is deliberately nothing but that list. Every "why" lives in the stage it
/// belongs to.
///
/// Deskew and downsample are stages inside this node, not separate nodes: the
/// intermediate clouds are only ever consumed here, so publishing them would
/// cost a serialize/deserialize round trip for nothing. They are exposed on
/// debug topics purely for RViz inspection.
///
/// THREADING. The subscription callbacks only buffer and sync -- they never run
/// the pipeline. Registration takes ~100 ms, and running it inside a callback (on
/// a single-threaded executor, holding the buffer mutex) blocked IMU intake for
/// its whole duration, turning a latency problem into DATA LOSS. A worker thread
/// now consumes synced measurement groups, so sensor intake never stalls.
///
/// Ownership: `pose_`, `map_`, `deskew_` and `voxel_` are touched ONLY by the
/// worker. The buffers are touched only under `buf_mutex_`. The queue between
/// them is the single hand-off point.
class GlassLioNode : public rclcpp::Node
{
public:
  GlassLioNode()
  : Node("glasslio_node")
  {
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    output_frame_ = declare_parameter<std::string>("output_frame", "livox_frame");
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    // Extra IMU we require past a scan's header stamp before we deskew it, so
    // the gyro integration fully covers the scan (10 Hz -> ~0.1 s scans).
    scan_guard_ = declare_parameter<double>("scan_guard_sec", 0.12);
    // Voxel edge length (m). Larger = fewer points = faster registration, but
    // coarser geometry. ~0.5 m is a sane start for indoor/outdoor Livox.
    voxel_leaf_ = declare_parameter<double>("voxel_leaf_size", 0.5);

    // Extrinsic lidar -> IMU. Nested YAML maps to dot-separated param names.
    const auto q_xyzw = declare_parameter<std::vector<double>>(
      "extrinsic.lidar_to_imu.quat_xyzw", {0.0, 0.0, 0.0, 1.0});
    const auto t_xyz = declare_parameter<std::vector<double>>(
      "extrinsic.lidar_to_imu.xyz", {0.0, 0.0, 0.0});

    const double map_voxel = declare_parameter<double>("map.voxel_size", 0.5);
    const int map_max_pts = declare_parameter<int>("map.max_points_per_voxel", 20);
    const double map_range = declare_parameter<double>("map.max_range", 100.0);
    // The PLANARITY GATE, and its companion. These decide which voxels yield a plane at
    // all -- i.e. how many correspondences ICP gets, and how trustworthy each one is.
    // Genuinely environment-dependent (see doc/6-local-map.md), hence config, not code.
    const int map_min_pts_plane =
      declare_parameter<int>("map.min_points_for_plane", LocalMap::kDefaultMinPointsForPlane);
    const double map_planarity =
      declare_parameter<double>("map.planarity_ratio", LocalMap::kDefaultPlanarityRatio);

    // IMU initialization. Livox publishes accel in *g*, not m/s^2 (spec violation),
    // so scale it. Get this wrong and every acceleration is 9.81x off.
    const bool accel_in_g = declare_parameter<bool>("imu.accel_in_g", true);
    const int init_samples = declare_parameter<int>("imu.init.num_samples", 200);
    const double init_max_gyro = declare_parameter<double>("imu.init.max_gyro", 0.1);
    const double init_max_acc_sd = declare_parameter<double>("imu.init.max_accel_sd", 0.5);

    q_il_ = parseQuat(q_xyzw);
    // The translation is VALIDATED (a malformed extrinsic must fail loudly at startup)
    // but not stored: deskew is rotation-only, and nothing else reads it yet.
    parseVec3(t_xyz);
    deskew_ = std::make_shared<Deskew>(get_logger());
    deskew_->set_extrinsic(q_il_);

    init_samples_ = init_samples;
    init_max_gyro_ = init_max_gyro;
    init_max_acc_sd_ = init_max_acc_sd;
    accel_scale_ = accel_in_g ? kGravity : 1.0;
    imu_init_ = std::make_unique<ImuInit>(
      init_samples, init_max_gyro, init_max_acc_sd, accel_scale_);

    map_voxel_ = map_voxel;
    map_max_pts_ = map_max_pts;
    map_range_ = map_range;
    map_min_pts_plane_ = map_min_pts_plane;
    map_planarity_ = map_planarity;
    map_ = std::make_unique<LocalMap>(
      map_voxel, map_max_pts, map_range, map_min_pts_plane, map_planarity);

    // Registration (point-to-plane ICP over the voxel map). max_corr_dist is the
    // important knob: too small and a fast motion falls outside the correspondence
    // radius and never converges; too large and it matches the wrong wall.
    reg_params_.max_correspondence_distance = declare_parameter<double>(
      "registration.max_correspondence_distance", 1.0);
    reg_params_.max_iterations = declare_parameter<int>("registration.max_iterations", 30);
    reg_params_.eps_translation = declare_parameter<double>(
      "registration.transformation_epsilon", 1e-3);
    reg_params_.eps_rotation = declare_parameter<double>(
      "registration.rotation_epsilon", 1e-4);
    reg_params_.huber_delta = declare_parameter<double>("registration.huber_delta", 0.2);
    reg_params_.min_correspondences = declare_parameter<int>(
      "registration.min_correspondences", 50);
    max_rmse_ = declare_parameter<double>("registration.max_rmse", 0.5);
    // Constant-velocity translation prior. Without it the guess carries NO
    // translation, so a dropped scan (see max_queue_size) leaves the robot metres
    // from where ICP starts looking.
    //
    // This once caused a runaway -- ICP "converges" near a too-far guess, that bad
    // scan is inserted into the map, the map drifts with it, velocity grows. That
    // loop is now BLOCKED: a rejected scan is no longer inserted, so a bad guess
    // cannot poison the reference it is measured against. If you ever see the pose
    // accelerating away with a healthy rmse, turn this off first.
    use_const_vel_ = declare_parameter<bool>("registration.use_constant_velocity", true);

    // --- Tight coupling. `imu_prior_weight` selects the estimator:
    //
    //   0   -> LOOSE. The IMU predicts, then ICP solves and the IMU gets no vote. This
    //          is the historical path, kept as a directly comparable baseline. With no
    //          IMU information, velocity and the biases are UNOBSERVABLE (the LiDAR
    //          Jacobian's columns for them are structurally zero), so we do not pretend
    //          to estimate them -- we run the 6-DoF SE(3) solve instead.
    //   > 0 -> TIGHT. One joint 15-DoF solve; the IMU is a prior residual in the same
    //          normal equations. See doc/7-tight-coupling.md.
    const double imu_prior_weight =
      declare_parameter<double>("registration.imu_prior_weight", 1.0);
    use_tight_ = imu_prior_weight > 0.0;

    tight_params_.imu_prior_weight = imu_prior_weight;
    tight_params_.max_correspondence_distance = reg_params_.max_correspondence_distance;
    tight_params_.max_iterations = reg_params_.max_iterations;
    tight_params_.min_correspondences = reg_params_.min_correspondences;
    tight_params_.eps_translation = reg_params_.eps_translation;
    tight_params_.eps_rotation = reg_params_.eps_rotation;
    tight_params_.huber_delta = reg_params_.huber_delta;
    // Point-to-plane measurement noise. In the loose path a global scale on the
    // residuals cancels out of H xi = b and is harmless; the moment the IMU enters the
    // SAME normal equations it stops being arbitrary -- it decides which sensor wins.
    tight_params_.lidar_sigma = declare_parameter<double>("registration.lidar_sigma", 0.05);
    bias_rw_gyro_ = declare_parameter<double>("imu.bias_rw_gyro", 1e-4);
    bias_rw_accel_ = declare_parameter<double>("imu.bias_rw_accel", 1e-3);
    // INITIAL bias uncertainty -- how wrong the starting bias might be. Distinct from the
    // random walk, which only says how fast it may drift from wherever it is. A static
    // window cannot observe the accel bias at all, so we start genuinely ignorant of it.
    bias_sigma0_gyro_ = declare_parameter<double>("imu.bias_sigma0_gyro", 0.01);
    bias_sigma0_accel_ = declare_parameter<double>("imu.bias_sigma0_accel", 0.3);
    tight_warmup_scans_ = declare_parameter<int>("registration.tight_warmup_scans", 10);

    // IMU noise densities: how much the preintegrated delta is trusted.
    gyro_noise_ = declare_parameter<double>("imu.gyro_noise", 1.7e-3);
    accel_noise_ = declare_parameter<double>("imu.accel_noise", 2.0e-2);

    // Worker backlog. Small on purpose: if registration cannot keep up, a stale
    // pose is useless, so drop scans rather than accumulate latency.
    max_queue_ = static_cast<std::size_t>(declare_parameter<int>("max_queue_size", 3));

    voxel_.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);

    auto qos = rclcpp::SensorDataQoS();
    sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, qos,
      std::bind(&GlassLioNode::lidarCallback, this, std::placeholders::_1));
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      std::bind(&GlassLioNode::imuCallback, this, std::placeholders::_1));

    pub_deskewed_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed", 10);
    pub_downsampled_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/downsampled", 10);
    pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/local_map", 1);
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      get_logger(), "glasslio_node up. lidar='%s' imu='%s' voxel=%.2fm",
      lidar_topic_.c_str(), imu_topic_.c_str(), voxel_leaf_);
    RCLCPP_INFO(
      get_logger(), "extrinsic lidar->imu: q_xyzw=[%.4f %.4f %.4f %.4f] t=[%.4f %.4f %.4f]",
      q_xyzw[0], q_xyzw[1], q_xyzw[2], q_xyzw[3], t_xyz[0], t_xyz[1], t_xyz[2]);
    RCLCPP_INFO(
      get_logger(),
      "local map: voxel=%.2fm max_pts/voxel=%d range=%.1fm min_pts/plane=%d planarity=%.2f",
      map_voxel, map_max_pts, map_range, map_min_pts_plane, map_planarity);

    worker_ = std::thread(&GlassLioNode::workerLoop, this);
  }

  ~GlassLioNode() override
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stop_ = true;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  /// A malformed extrinsic silently corrupts every scan, so fail loudly here.
  Eigen::Quaterniond parseQuat(const std::vector<double> & q_xyzw)
  {
    if (q_xyzw.size() != 4) {
      throw std::runtime_error("extrinsic.lidar_to_imu.quat_xyzw must have 4 elements (x,y,z,w)");
    }
    const Eigen::Quaterniond q(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);  // (w,x,y,z) ctor
    if (q.norm() < 1e-6) {
      throw std::runtime_error("extrinsic.lidar_to_imu.quat_xyzw is degenerate (zero norm)");
    }
    return q.normalized();
  }

  Eigen::Vector3d parseVec3(const std::vector<double> & v)
  {
    if (v.size() != 3) {
      throw std::runtime_error("extrinsic.lidar_to_imu.xyz must have 3 elements");
    }
    return {v[0], v[1], v[2]};
  }

  /// A timestamp older than the last one means one of three things, and they need
  /// very different responses:
  ///   - a big jump back  -> the source RESTARTED (bag replay/loop). The pose and
  ///                         map now describe a world we are no longer in, so the
  ///                         whole estimator must be reset, not just the buffers.
  ///   - a small step back -> one out-of-order message. Drop it. Clearing the
  ///                         buffer here would throw away good data.
  ///   - neither           -> normal.
  /// Returns true if the caller should DROP this message.
  bool handleTimeJump(double t, double & last_t, const char * what)
  {
    if (last_t < 0.0 || t >= last_t) {
      last_t = t;
      return false;
    }
    if (last_t - t > kRestartJumpSec) {
      RCLCPP_WARN(
        get_logger(), "%s time jumped back %.1fs -- source restarted; resetting estimator",
        what, last_t - t);
      reset();
      last_t = t;
      return false;
    }
    // Out of order by a hair. Drop just this one; do not nuke the buffer.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "%s messages arriving out of order (by %.3fs); dropping. Are two publishers "
      "running (e.g. a stray 'ros2 bag play')?", what, last_t - t);
    return true;
  }

  /// Full reset: the world we mapped is gone. Callback thread.
  ///
  /// Only the callback-owned state is reset HERE. The estimator (map_, pose_,
  /// velocity_) belongs to the worker, and clearing the queue does NOT make it
  /// safe to touch: the worker may already have popped a scan and be inside
  /// processOne(), so destroying map_ under it is a use-after-free. So we only
  /// REQUEST the reset; the worker applies it to itself, in order, between scans.
  void reset()
  {
    lidar_buffer_.clear();
    imu_buffer_.clear();
    last_lidar_time_ = -1.0;
    last_imu_time_ = -1.0;
    imu_init_ = std::make_unique<ImuInit>(
      init_samples_, init_max_gyro_, init_max_acc_sd_, accel_scale_);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_.clear();
      reset_requested_ = true;
    }
    queue_cv_.notify_one();
    RCLCPP_WARN(get_logger(), "estimator reset: map cleared, re-initializing IMU");
  }

  /// Apply the requested reset. Worker thread only -- this is the thread that
  /// owns the estimator, so nothing can be mid-scan against it.
  void resetEstimator()
  {
    map_ = std::make_unique<LocalMap>(
      map_voxel_, map_max_pts_, map_range_, map_min_pts_plane_, map_planarity_);
    map_bootstrapped_ = false;
    pose_ = Eigen::Isometry3d::Identity();
    velocity_.setZero();
    state_ = NavState();
    scans_done_ = 0;
    prev_scan_end_ = -1.0;
    resetBiasCovariance();
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(buf_mutex_);
    const double t = stamp_sec(msg);
    if (handleTimeJump(t, last_imu_time_, "IMU")) {
      return;
    }

    if (!imu_init_->initialized()) {
      if (imu_init_->add(*msg)) {
        onInitialized();
      }
      return;   // no scans are processed until the IMU is initialized
    }

    imu_buffer_.push_back(msg);
    enqueueReady();
  }

  /// Publish the init result to the worker, which owns the estimator. Callback
  /// thread: it must not write pose_ or deskew_ itself (see reset()).
  void onInitialized()
  {
    // The world frame is gravity-aligned (Z up). The initial LIDAR pose is
    // therefore R_wl = R_wi * R_il -- the extrinsic still has to be applied,
    // because gravity was measured in the IMU frame, not the lidar frame.
    Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();
    init_pose.linear() =
      (imu_init_->initial_rotation().unit_quaternion() * q_il_).toRotationMatrix();
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pending_init_pose_ = init_pose;
      pending_init_bias_ = imu_init_->gyro_bias();
      // Gravity, in the gravity-aligned world frame: straight down, at the STANDARD
      // magnitude.
      //
      // NOT the measured magnitude, and this is the bug that sank the first attempt. The
      // measured specific force at rest is  g + b_a  -- gravity PLUS the accelerometer
      // bias. Baking that sum into a "gravity" constant conflates two things that behave
      // completely differently: gravity is fixed in the WORLD frame, while the bias is
      // fixed in the BODY frame. They agree at init and then diverge the moment the robot
      // turns, injecting a spurious acceleration of order |b_a| in an arbitrary
      // direction. (Measured: ~0.2 m/s^2, i.e. Z falling 380 m over a minute.)
      //
      // Use true gravity here, and let b_a be ESTIMATED as the body-frame quantity it is.
      pending_init_gravity_ = Eigen::Vector3d(0.0, 0.0, -kGravity);
      init_pending_ = true;
    }
    queue_cv_.notify_one();

    const auto & b = imu_init_->gyro_bias();
    const auto & g = imu_init_->gravity();
    const double tilt =
      std::acos(std::clamp(g.normalized().z(), -1.0, 1.0)) * 180.0 / M_PI;

    RCLCPP_INFO(get_logger(), "IMU initialized (%d window(s) rejected for motion)",
      imu_init_->rejected_windows());
    RCLCPP_INFO(get_logger(), "  gyro bias : [%+.5f %+.5f %+.5f] rad/s", b.x(), b.y(), b.z());
    RCLCPP_INFO(get_logger(), "  gravity   : [%+.3f %+.3f %+.3f] m/s^2  |g|=%.3f",
      g.x(), g.y(), g.z(), g.norm());
    RCLCPP_INFO(get_logger(), "  mount tilt: %.2f deg from vertical", tilt);

    if (std::abs(g.norm() - kGravity) > 1.0) {
      RCLCPP_ERROR(
        get_logger(),
        "|gravity| = %.3f m/s^2, expected ~%.2f. Check imu.accel_in_g -- wrong units "
        "will make every acceleration 9.81x off.", g.norm(), kGravity);
    }
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(buf_mutex_);

    if (!imu_init_->initialized()) {
      // Drop scans rather than buffer them: they predate initialization, and the
      // IMU samples that would deskew them were consumed by the init window.
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "waiting for IMU initialization (hold still)...");
      return;
    }

    const double t = stamp_sec(msg);
    if (handleTimeJump(t, last_lidar_time_, "lidar")) {
      return;
    }
    lidar_buffer_.push_back(msg);
    enqueueReady();
  }

  /// Callback side: move every scan the IMU now covers onto the worker queue.
  /// Caller holds buf_mutex_. Does NO heavy work -- that is the whole point.
  void enqueueReady()
  {
    MeasureGroup meas;
    while (syncMeasure(meas)) {
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        // Bounded. If the worker cannot keep up, DROP the oldest rather than let
        // latency and memory grow without bound -- a stale pose is useless anyway.
        if (queue_.size() >= max_queue_) {
          queue_.pop_front();
          ++dropped_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "worker behind: dropped %ld scan(s); registration is too slow", dropped_);
        }
        queue_.push_back(std::move(meas));
      }
      queue_cv_.notify_one();
      meas = MeasureGroup();
    }
  }

  /// Worker side: the actual pipeline. Runs off the callback threads.
  void workerLoop()
  {
    while (rclcpp::ok()) {
      MeasureGroup meas;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(
          lock, [this] {
            return stop_ || reset_requested_ || init_pending_ || !queue_.empty();
          });
        if (stop_) {
          return;
        }
        // Ordered against the scans: a scan already in flight when the reset was
        // requested completes first, and its writes are then wiped by the reset.
        if (reset_requested_) {
          reset_requested_ = false;
          lock.unlock();
          resetEstimator();
          continue;
        }
        // Both reset and init hand the estimator over here rather than writing it
        // from a callback, so the worker's exclusive ownership actually holds.
        if (init_pending_) {
          init_pending_ = false;
          const Eigen::Isometry3d init_pose = pending_init_pose_;
          const Eigen::Vector3d init_bias = pending_init_bias_;
          const Eigen::Vector3d init_gravity = pending_init_gravity_;
          lock.unlock();

          pose_ = init_pose;
          deskew_->set_gyro_bias(init_bias);

          // Seed the 15-DoF state from the init result. Velocity starts at zero and the
          // accel bias at zero: neither is observable from a static window, and the
          // tight solve estimates both from here on.
          gravity_ = init_gravity;
          state_ = NavState();
          state_.R = Sophus::SO3d(Eigen::Quaterniond(init_pose.linear()).normalized());
          state_.p = init_pose.translation();
          state_.bg = init_bias;
          continue;
        }
        meas = std::move(queue_.front());
        queue_.pop_front();
      }
      processOne(meas);
    }
  }

  /// The pipeline for one measurement group. Worker thread only.
  ///
  /// This function is deliberately nothing but the pipeline, in the order the docs number
  /// it (doc/pipeline.md). Stages 1 and 2 -- IMU init and sync -- already happened on the
  /// callback side; a MeasureGroup arriving here IS the product of those two. Every "why"
  /// below lives in the stage it belongs to, so that this reads as the table of contents.
  void processOne(const MeasureGroup & meas)
  {
    // --- [3] DESKEW: undo the intra-scan rotation (doc/3-deskew.md) ---
    const CloudXYZI::Ptr deskewed = deskew_->Process(meas);
    if (!deskewed || deskewed->empty()) {
      return;
    }

    // --- [4] DOWNSAMPLE: fewer points for ICP to chew (doc/4-downsample.md) ---
    const CloudXYZI::Ptr downsampled = downsample(deskewed);

    // --- [5] REGISTER: align against the local map -> pose_ (doc/5-registration.md) ---
    const bool ok = registerScan(downsampled, meas);

    // --- [6] LOCAL MAP: fold the aligned scan back in (doc/6-local-map.md) ---
    insertIntoMap(deskewed, ok);

    // --- [7] OUTPUT: odom, TF, debug clouds ---
    logScan(*deskewed, *downsampled, ok);
    publishAll(meas, deskewed, downsampled);
  }

  /// [4] Voxel-grid downsample. Feeds ICP only -- the MAP gets the dense cloud, see
  /// insertIntoMap().
  CloudXYZI::Ptr downsample(const CloudXYZI::Ptr & cloud)
  {
    CloudXYZI::Ptr out(new CloudXYZI());
    voxel_.setInputCloud(cloud);
    voxel_.filter(*out);
    return out;
  }

  /// [5] Register, loose or tight, and hand back whether the pose can be trusted.
  ///
  /// TIGHT COUPLING NEEDS A VELOCITY TO START FROM, and IMU init cannot give it one: a
  /// static window cannot tell REST from CONSTANT VELOCITY -- both show zero rotation and
  /// zero acceleration variance (doc/1-imu-init.md). On this bag the robot is already
  /// cruising at ~1.5 m/s when recording starts, so init reports "static" and seeds v = 0.
  ///
  /// Loose coupling shrugs that off: velocity is only a prior there, and it self-corrects
  /// by finite-differencing poses. Tight coupling CANNOT, because it holds x_i FIXED -- a
  /// wrong v_i is treated as CERTAIN, so the IMU factor spends every scan insisting the
  /// robot is stationary while the LiDAR insists otherwise. They fight, the residual grows,
  /// correspondences collapse, and the estimator free-falls on dead reckoning. (Observed:
  /// 532/691 scans rejected, pose to +6 km.)
  ///
  /// So run LOOSE for the first few scans, let ICP MEASURE the velocity, and only then hand
  /// a correct initial state to the tight solver.
  bool registerScan(const CloudXYZI::Ptr & scan, const MeasureGroup & meas)
  {
    const bool warming_up = use_tight_ && scans_done_ < tight_warmup_scans_;

    const bool ok = (use_tight_ && !warming_up) ?
      registerScanTight(scan, meas) :
      registerScanLoose(scan);

    if (warming_up && scans_done_ + 1 == tight_warmup_scans_) {
      seedNavStateFromLoose();
    }
    ++scans_done_;
    return ok;
  }

  /// [6] Fold the aligned scan into the map, in the world frame.
  ///
  /// Insert ONLY if we trust the pose. A scan placed at a known-wrong pose POISONS the map
  /// -- and the map is what the NEXT scan registers against, so the error compounds into a
  /// death spiral: the pose freezes, the robot keeps moving, each frozen-pose scan corrupts
  /// the map further, correspondences collapse, and it never recovers.
  ///
  /// EXCEPT while bootstrapping. Until the map has supported one successful registration it
  /// is too sparse to register against (a voxel needs >= 5 points before PCA yields a
  /// plane), so refusing to insert would deadlock: sparse map -> ICP under-constrained ->
  /// no insert -> map stays sparse, forever. During bootstrap we dead-reckon on the IMU
  /// prior and keep feeding the map anyway.
  void insertIntoMap(const CloudXYZI::Ptr & deskewed, bool pose_trusted)
  {
    if (pose_trusted) {
      map_bootstrapped_ = true;
    }
    if (!pose_trusted && map_bootstrapped_) {
      return;
    }

    // Feed the map the DENSE cloud, not the downsampled one. Downsampling is an ICP
    // *source* optimisation; the map is the plane-fitting *target* and needs the density.
    // At a 0.5 m leaf the downsampled cloud gives ~1 point per map voxel -- never enough
    // for a plane.
    CloudXYZI::Ptr scan_world(new CloudXYZI());
    pcl::transformPointCloud(*deskewed, *scan_world, pose_.matrix());
    map_->insert(*scan_world);
    map_->prune(pose_.translation());
  }

  /// [7] One line per scan. `map N vox` is the health check: with a correct pose the voxel
  /// count PLATEAUS (the same geometry re-observed lands in the same voxels). Climbing
  /// without bound means the same wall is being re-inserted at slightly wrong places --
  /// i.e. the pose is drifting. See doc/6-local-map.md.
  void logScan(const CloudXYZI & deskewed, const CloudXYZI & downsampled, bool ok)
  {
    const Eigen::Vector3d & t = pose_.translation();
    RCLCPP_INFO(
      get_logger(),
      "scan %zu->%zu | pose [%+.2f %+.2f %+.2f] | rmse %.3f (%d corr)%s | map %zu vox | q %zu",
      deskewed.size(), downsampled.size(),
      t.x(), t.y(), t.z(), last_rmse_, last_corr_, ok ? "" : " (DIVERGED)",
      map_->num_voxels(), queueSize());
  }

  /// [7] Odometry + TF, and the debug clouds (which skip serialization when unsubscribed).
  void publishAll(
    const MeasureGroup & meas,
    const CloudXYZI::Ptr & deskewed,
    const CloudXYZI::Ptr & downsampled)
  {
    publishOdom(meas.lidar->header);
    publish(pub_deskewed_, deskewed, meas.lidar->header);
    publish(pub_downsampled_, downsampled, meas.lidar->header);
    publishMap(meas.lidar->header);
  }

  std::size_t queueSize()
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return queue_.size();
  }

  /// Preintegrate the IMU samples spanning this scan, at the CURRENT bias estimate.
  ///
  /// The deltas are computed relative to `state_.bg` / `state_.ba`; if the solve then
  /// moves the bias, the first-order Jacobians shift the deltas instead of forcing a
  /// re-integration. That is the whole point of preintegration.
  /// Integrate over EXACTLY (t_begin, t_end] -- the previous scan's end to this one's.
  ///
  /// THE WINDOW IS NOT THE MEASUREGROUP. `meas.imu` deliberately over-covers the scan:
  /// it carries a bracket sample before the start and IMU past the end (see
  /// doc/2-sync.md), so it spans ~0.12 s. But deskew compensates every point into the
  /// scan-END frame, so consecutive POSES are 0.10 s apart.
  ///
  /// Integrate the whole group and the IMU factor asserts "over 0.12 s you moved dp"
  /// against a pose delta that covers 0.10 s -- a systematic ~20% over-integration, every
  /// scan, compounding. It reads exactly like a gravity error: the estimator accelerates
  /// away and Z falls quadratically. (Observed: Z to -1 km.)
  ///
  /// So the interval is clipped at both edges, including partial sample intervals.
  ImuPreintegration buildPreintegration(
    const MeasureGroup & meas, double t_begin, double t_end) const
  {
    ImuPreintegration pre(state_.bg, state_.ba, gyro_noise_, accel_noise_);

    for (std::size_t i = 0; i + 1 < meas.imu.size(); ++i) {
      const double ta = stamp_sec(meas.imu[i]);
      const double tb = stamp_sec(meas.imu[i + 1]);

      // Clip this sample's interval into the window.
      const double lo = std::max(ta, t_begin);
      const double hi = std::min(tb, t_end);
      const double dt = hi - lo;
      if (dt <= 0.0 || dt > kMaxImuGapSec) {
        continue;   // outside the window, duplicate, out of order, or a hole
      }

      const auto & m = *meas.imu[i];   // zero-order hold over the interval
      const Eigen::Vector3d w(
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);
      // SI, always. The Livox reports accel in g (see doc/1-imu-init.md); feeding raw
      // g into an integrator that also adds gravity in m/s^2 would be incoherent.
      const Eigen::Vector3d a = Eigen::Vector3d(
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z) *
        accel_scale_;
      pre.integrate(w, a, dt);
    }
    return pre;
  }

  /// TIGHTLY-COUPLED registration: one joint solve over the 15-DoF nav state, with the
  /// LiDAR residuals and the preintegrated IMU factor in the SAME normal equations.
  /// Updates `state_` (and `pose_`/`velocity_`, which are just views of it).
  bool registerScanTight(const CloudXYZI::Ptr & scan, const MeasureGroup & meas)
  {
    // The pose refers to the scan-END instant (deskew compensates there), so the IMU
    // factor must span previous-scan-end -> this-scan-end. Nothing else is coherent.
    const double t_end = deskew_->last_scan_end();
    const double t_begin = (prev_scan_end_ > 0.0) ?
      prev_scan_end_ :
      t_end - deskew_->last_scan_duration();

    const ImuPreintegration pre = buildPreintegration(meas, t_begin, t_end);
    if (pre.dt() <= 0.0) {
      return false;   // no usable IMU span; nothing to predict from
    }
    prev_scan_end_ = t_end;

    // RANDOM WALK: the bias may have drifted since the last scan, so we are now slightly
    // LESS certain of it than we were. Covariance grows with time.
    const double dt_s = pre.dt();
    bias_cov_.block<3, 3>(0, 0) +=
      Eigen::Matrix3d::Identity() * (bias_rw_gyro_ * bias_rw_gyro_ * dt_s);
    bias_cov_.block<3, 3>(3, 3) +=
      Eigen::Matrix3d::Identity() * (bias_rw_accel_ * bias_rw_accel_ * dt_s);

    // PREDICT. This replaces the constant-velocity guess outright: it integrates the
    // accelerometer rather than assuming the acceleration was zero.
    const NavState guess = predictState(state_, pre, gravity_);

    // Bootstrap: the first scan defines the origin. Keep the gravity-aligned attitude
    // from init, zero translation -- there is nothing to align against yet.
    if (map_->empty()) {
      RCLCPP_INFO(get_logger(), "first scan: seeding map, origin defined");
      last_rmse_ = 0.0;
      last_corr_ = 0;
      return true;
    }

    const TightResult r = alignTightlyCoupled(
      *scan, *map_, state_, pre, gravity_, guess, bias_cov_.inverse(), tight_params_);
    last_rmse_ = r.rmse;
    last_corr_ = r.correspondences;

    if (!r.valid || r.rmse > max_rmse_) {
      RCLCPP_WARN(
        get_logger(),
        "tight solve rejected (%s, rmse %.3f, %d corr) -- coasting on IMU, scan NOT added",
        r.valid ? "residual too large" : "under-constrained", r.rmse, r.correspondences);
      // Coast on the IMU prediction. Note this is a REAL dead-reckon now (velocity and
      // bias are states, driven by the accelerometer), not a constant-velocity guess.
      commitState(guess);
      return false;
    }

    // POSTERIOR: fold in what this solve actually learned about the biases.
    //
    //     P_posterior = (P_prior^-1 + H_bias)^-1
    //
    // THIS is the difference between a filter and a one-shot factor. Without it the
    // estimator can never become more certain of a bias than it was at startup, so a
    // quantity only the data can reveal -- the accel bias -- stays frozen at its initial
    // guess forever, and every error it causes is permanent.
    const Eigen::Matrix<double, 6, 6> posterior_info =
      bias_cov_.inverse() + r.H.block<6, 6>(kIdxBg, kIdxBg);
    const Eigen::Matrix<double, 6, 6> posterior_cov = posterior_info.inverse();
    if (posterior_cov.allFinite()) {
      bias_cov_ = posterior_cov;
    }

    commitState(r.state);
    return true;
  }

  /// Reset the carried bias covariance to genuine ignorance about the accel bias.
  void resetBiasCovariance()
  {
    bias_cov_.setZero();
    bias_cov_.block<3, 3>(0, 0) =
      Eigen::Matrix3d::Identity() * (bias_sigma0_gyro_ * bias_sigma0_gyro_);
    bias_cov_.block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity() * (bias_sigma0_accel_ * bias_sigma0_accel_);
  }

  /// Hand the loose path's answer to the tight solver: the pose it converged on, and --
  /// the part that actually matters -- the VELOCITY it measured by finite-differencing.
  /// That is the number IMU init could not observe.
  void seedNavStateFromLoose()
  {
    state_ = NavState();
    state_.R = Sophus::SO3d(Eigen::Quaterniond(pose_.linear()).normalized());
    state_.p = pose_.translation();
    state_.v = velocity_;
    state_.bg = imu_init_->gyro_bias();
    // Accel bias starts at zero -- and, crucially, starts UNCERTAIN. A static window
    // cannot observe it, so we say so, and let the data reveal it.
    resetBiasCovariance();

    RCLCPP_INFO(
      get_logger(),
      "tight coupling engaged after %d warm-up scans. seeded v = [%+.2f %+.2f %+.2f] m/s "
      "(|v| = %.2f) -- IMU init could not observe this.",
      tight_warmup_scans_, state_.v.x(), state_.v.y(), state_.v.z(), state_.v.norm());
  }

  /// Commit a nav state and mirror it into pose_/velocity_, which the map insert, the
  /// odometry publisher and the TF broadcaster all read.
  void commitState(const NavState & x)
  {
    state_ = x;
    pose_.linear() = x.R.matrix();
    pose_.translation() = x.p;
    velocity_ = x.v;
  }

  /// [5] LOOSE registration: the IMU only proposes a guess, then ICP solves alone.
  /// Used when imu_prior_weight == 0 (the default). Returns false if ICP failed, in which
  /// case we keep the IMU prediction and the scan is NOT inserted into the map.
  bool registerScanLoose(const CloudXYZI::Ptr & scan)
  {
    // Bootstrap: nothing to align against. The first scan DEFINES the origin --
    // pose_ keeps the gravity-aligned orientation from init, translation zero.
    if (map_->empty()) {
      RCLCPP_INFO(get_logger(), "first scan: seeding map, origin defined");
      last_rmse_ = 0.0;
      return true;
    }

    // --- Predict. ICP is a LOCAL optimizer: hand it a guess a few degrees off
    // and it will happily lock onto the wrong wall and report a confident fit.
    // Rotation comes from the gyro (accurate); translation from a constant-
    // velocity model (off by default -- see the runaway note above).
    const double dt = deskew_->last_scan_duration();
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess.linear() = pose_.linear() * deskew_->last_delta_rot().matrix();
    guess.translation() = use_const_vel_ ?
      (pose_.translation() + velocity_ * dt).eval() :
      pose_.translation();

    const auto r = alignPointToPlane(*scan, *map_, guess, reg_params_);
    last_rmse_ = r.rmse;
    last_corr_ = r.correspondences;

    // Trust `valid` (enough correspondences + finite solve) and the residual --
    // NOT `converged`. Hitting max_iterations is not failure: ICP routinely
    // plateaus above eps while sitting on a perfectly good fit, and rejecting
    // those threw away good poses and froze the estimator.
    if (!r.valid || r.rmse > max_rmse_) {
      RCLCPP_WARN(
        get_logger(),
        "ICP rejected (%s, rmse %.3f, %d corr) -- coasting, scan NOT added to map",
        r.valid ? "residual too large" : "under-constrained",
        r.rmse, r.correspondences);
      // Coast on the prediction. The pose is now a guess, so the scan must NOT
      // go into the map -- see processOne().
      updatePose(guess, dt);
      return false;
    }

    if (!r.converged) {
      RCLCPP_DEBUG(
        get_logger(), "ICP hit max_iterations (rmse %.3f) -- accepting anyway", r.rmse);
    }

    updatePose(r.pose, dt);
    return true;
  }

  /// Commit a new pose and refresh the constant-velocity estimate.
  void updatePose(const Eigen::Isometry3d & new_pose, double dt)
  {
    if (dt > 1e-6) {
      velocity_ = (new_pose.translation() - pose_.translation()) / dt;
    }
    pose_ = new_pose;
    // Re-orthonormalize: repeated float<->double round trips through PCL slowly
    // erode the rotation block away from SO(3).
    Eigen::Quaterniond q(pose_.linear());
    pose_.linear() = q.normalized().toRotationMatrix();
  }

  void publishOdom(const std_msgs::msg::Header & header)
  {
    const Eigen::Quaterniond q(pose_.linear());
    const Eigen::Vector3d & t = pose_.translation();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = header.stamp;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = output_frame_;
    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    // Velocity is in the world frame; twist is specified in child_frame_id.
    const Eigen::Vector3d v_body = pose_.linear().transpose() * velocity_;
    odom.twist.twist.linear.x = v_body.x();
    odom.twist.twist.linear.y = v_body.y();
    odom.twist.twist.linear.z = v_body.z();
    pub_odom_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = header.stamp;
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = output_frame_;
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  void publish(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const CloudXYZI::ConstPtr & cloud,
    const std_msgs::msg::Header & header,
    const std::string & frame)
  {
    if (pub->get_subscription_count() == 0) {
      return;  // debug topic: skip the serialize cost when nobody is looking
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header = header;
    msg.header.frame_id = frame;
    pub->publish(msg);
  }

  void publish(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const CloudXYZI::ConstPtr & cloud,
    const std_msgs::msg::Header & header)
  {
    publish(pub, cloud, header, output_frame_);  // sensor-frame clouds
  }

  /// The map lives in the world frame, not the sensor frame.
  void publishMap(const std_msgs::msg::Header & header)
  {
    publish(pub_map_, map_->target(), header, world_frame_);
  }

  /// Assemble one lidar frame with the IMU samples spanning it. Caller holds
  /// buf_mutex_. Requires an IMU sample before the scan start and IMU coverage
  /// past scan_start + scan_guard_.
  bool syncMeasure(MeasureGroup & meas)
  {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
      return false;
    }
    const double scan_t = stamp_sec(lidar_buffer_.front());
    const double scan_end = scan_t + scan_guard_;

    // Need an IMU sample before the scan to bracket the start.
    if (stamp_sec(imu_buffer_.front()) > scan_t) {
      RCLCPP_WARN(get_logger(), "dropping lidar frame with no preceding IMU");
      lidar_buffer_.pop_front();
      return false;
    }
    // Wait until IMU covers the end of the scan.
    if (stamp_sec(imu_buffer_.back()) < scan_end) {
      return false;
    }

    meas.lidar = lidar_buffer_.front();
    lidar_buffer_.pop_front();

    // Copy IMU covering the scan (keep them buffered for the next scan, which
    // starts inside this window). Include one bracket sample past scan_end.
    meas.imu.clear();
    for (const auto & imu : imu_buffer_) {
      meas.imu.push_back(imu);
      if (stamp_sec(imu) >= scan_end) {
        break;
      }
    }
    // Drop IMU strictly older than this scan start; the rest brackets the next.
    while (imu_buffer_.size() > 1 && stamp_sec(imu_buffer_[1]) <= scan_t) {
      imu_buffer_.pop_front();
    }
    return true;
  }

  /// A backwards time jump larger than this means the source restarted (bag loop),
  /// not just an out-of-order message.
  static constexpr double kRestartJumpSec = 1.0;

  std::string lidar_topic_, imu_topic_, output_frame_, world_frame_;
  double scan_guard_ = 0.12;
  double voxel_leaf_ = 0.5;

  // Kept so reset() can rebuild the map and initializer from scratch.
  double map_voxel_ = 0.5, map_range_ = 100.0;
  int map_max_pts_ = 20;
  int map_min_pts_plane_ = LocalMap::kDefaultMinPointsForPlane;
  double map_planarity_ = LocalMap::kDefaultPlanarityRatio;
  int init_samples_ = 200;
  double init_max_gyro_ = 0.1, init_max_acc_sd_ = 0.5, accel_scale_ = kGravity;

  // --- Callback side: sensor buffers. Never touched by the worker.
  std::mutex buf_mutex_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> lidar_buffer_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;
  double last_lidar_time_ = -1.0;
  double last_imu_time_ = -1.0;

  // --- The hand-off: the only shared state between callbacks and the worker.
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<MeasureGroup> queue_;
  bool stop_ = false;
  /// Set by a callback, acted on by the worker: see reset() / resetEstimator().
  bool reset_requested_ = false;
  /// Init result handed from the callback to the worker: see onInitialized().
  bool init_pending_ = false;
  Eigen::Isometry3d pending_init_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d pending_init_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d pending_init_gravity_{0.0, 0.0, -kGravity};
  std::size_t max_queue_ = 3;
  long dropped_ = 0;

  std::shared_ptr<Deskew> deskew_;
  std::unique_ptr<ImuInit> imu_init_;
  Eigen::Quaterniond q_il_ = Eigen::Quaterniond::Identity();  ///< extrinsic lidar -> IMU
  pcl::VoxelGrid<pcl::PointXYZI> voxel_;
  std::unique_ptr<LocalMap> map_;

  /// LIDAR pose in the world frame, set by registration each scan.
  Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();
  /// World-frame linear velocity, from the last pose delta. Feeds the prediction.
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();

  /// Has the map ever supported a successful registration? Until it has, it is
  /// too sparse to align against and we must keep feeding it (see processOne).
  bool map_bootstrapped_ = false;

  RegistrationParams reg_params_;
  double max_rmse_ = 0.5;
  double last_rmse_ = 0.0;
  int last_corr_ = 0;
  bool use_const_vel_ = true;

  // --- Tight coupling. Worker-owned, like the rest of the estimator.
  bool use_tight_ = true;
  TightParams tight_params_;
  double gyro_noise_ = 1.7e-3;
  /// Scans to run LOOSE before engaging the IMU, so ICP can measure the velocity that
  /// IMU init cannot observe. See processOne().
  int tight_warmup_scans_ = 10;
  int scans_done_ = 0;
  /// Scan-end time of the previous processed scan: the lower edge of the IMU factor's
  /// integration window. -1 until the first tight scan.
  mutable double prev_scan_end_ = -1.0;
  double accel_noise_ = 2.0e-2;
  double bias_rw_gyro_ = 1e-4, bias_rw_accel_ = 1e-3;
  double bias_sigma0_gyro_ = 0.01, bias_sigma0_accel_ = 0.3;
  /// Covariance of the bias estimate, CARRIED ACROSS SCANS: it grows by the random walk
  /// and shrinks by what each solve learns. Starting it LOOSE is what lets the accel bias
  /// -- which a static window cannot observe at all -- actually be discovered from data.
  /// Without this the bias is frozen at its initial guess and every error it causes is
  /// permanent: a factor, not a filter.
  Eigen::Matrix<double, 6, 6> bias_cov_ = Eigen::Matrix<double, 6, 6>::Identity();
  /// The 15-DoF state (R, p, v, b_g, b_a). In tight mode this is the source of truth
  /// and pose_/velocity_ are views onto it; in loose mode it is unused.
  NavState state_;
  /// Gravity in the WORLD frame. The world is gravity-aligned at init, so this is
  /// straight down -- but with the MAGNITUDE we actually measured, not 9.80665, because
  /// the accelerometer's scale factor is what the estimator will be fighting.
  Eigen::Vector3d gravity_{0.0, 0.0, -kGravity};

  /// An IMU gap longer than this is a hole in the stream, not a sample interval.
  static constexpr double kMaxImuGapSec = 0.1;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_deskewed_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_downsampled_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace glasslio

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<glasslio::GlassLioNode>());
  rclcpp::shutdown();
  return 0;
}
