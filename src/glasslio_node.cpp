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
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "glasslio/deskew.hpp"
#include "glasslio/imu_init.hpp"
#include "glasslio/lio_estimator.hpp"
#include "glasslio/local_map.hpp"
#include "glasslio/sync.hpp"
#include "glasslio/ros_time.hpp"

namespace glasslio
{

/// LiDAR-inertial odometry node -- the ROS SHELL, and nothing more.
///
/// The pipeline, numbered as doc/pipeline.md numbers it:
///
///   [1] IMU init      gate: nothing runs until it completes    | THIS FILE
///   [2] sync          pair a scan with the IMU spanning it     | (callback side,
///                     -> MeasureSync                           |  under buf_mutex_)
///        |
///        v            the MeasureGroup handed to the worker IS stages [1]+[2]
///
///   [3] deskew        undo intra-scan rotation on SO(3)        | LioEstimator
///   [4] downsample    voxel grid, ICP source only              | (worker thread,
///   [5] register      point-to-plane ICP -> the pose           |  see
///   [6] local map     insert the aligned scan; [5]'s target    |  lio_estimator.hpp)
///
///   [7] output        /odom + TF, debug clouds                 | THIS FILE
///
/// The node does NOT run the pipeline. It parses parameters, subscribes, syncs, owns the
/// worker thread, hands MeasureGroups to the estimator, and turns what comes back into log
/// lines and topics. Stages [3]-[6] live in LioEstimator and know nothing about ROS.
///
/// The intermediate clouds are exposed on debug topics purely for RViz -- they are never
/// consumed outside this process, so publishing them between separate nodes would buy a
/// serialize/deserialize round trip and nothing else.
///
/// THREADING. The subscription callbacks only buffer and sync -- they never run the
/// pipeline. Registration takes ~100 ms, and running it inside a callback (on a
/// single-threaded executor, holding the buffer mutex) blocked IMU intake for its whole
/// duration, turning a latency problem into DATA LOSS. A worker thread now consumes synced
/// measurement groups, so sensor intake never stalls.
///
/// OWNERSHIP is the invariant that keeps that safe:
///   * `estimator_` (and everything inside it) is touched ONLY by the worker.
///   * `sync_` and the sensor buffers are touched only under `buf_mutex_`.
///   * the queue is the single hand-off point.
/// A callback that wants the estimator reset or initialized therefore REQUESTS it; the
/// worker applies it to itself, between scans. See workerLoop().
///
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
    sync_ = MeasureSync(scan_guard_);
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

    // [1] IMU init: the gate. Lives on the callback side; its result is handed to the
    // estimator over the queue.
    init_samples_ = init_samples;
    init_max_gyro_ = init_max_gyro;
    init_max_acc_sd_ = init_max_acc_sd;
    accel_scale_ = accel_in_g ? kGravity : 1.0;
    imu_init_ = std::make_unique<ImuInit>(
      init_samples, init_max_gyro, init_max_acc_sd, accel_scale_);

    // --- Everything the ESTIMATOR needs, gathered in one place. This is the node's only
    // say in how the pipeline behaves: parse the parameters, hand them over, get out of
    // the way.
    EstimatorParams ep;
    ep.extrinsic_q_il = q_il_;
    ep.accel_scale = accel_scale_;
    ep.voxel_leaf_size = declare_parameter<double>("voxel_leaf_size", 0.5);

    ep.map_voxel_size = map_voxel;
    ep.map_max_points_per_voxel = map_max_pts;
    ep.map_max_range = map_range;
    ep.map_min_points_for_plane = map_min_pts_plane;
    ep.map_planarity_ratio = map_planarity;

    // [5] Registration. max_corr_dist is the important knob: too small and a fast motion
    // falls outside the correspondence radius and never converges; too large and it
    // matches the WRONG wall.
    ep.reg.max_correspondence_distance = declare_parameter<double>(
      "registration.max_correspondence_distance", 1.0);
    ep.reg.max_iterations = declare_parameter<int>("registration.max_iterations", 30);
    ep.reg.eps_translation = declare_parameter<double>(
      "registration.transformation_epsilon", 1e-3);
    ep.reg.eps_rotation = declare_parameter<double>("registration.rotation_epsilon", 1e-4);
    ep.reg.huber_delta = declare_parameter<double>("registration.huber_delta", 0.2);
    ep.reg.min_correspondences = declare_parameter<int>(
      "registration.min_correspondences", 50);
    ep.max_rmse = declare_parameter<double>("registration.max_rmse", 0.5);

    // Constant-velocity translation prior. Without it the guess carries NO translation, so
    // a dropped scan leaves the robot metres from where ICP starts looking. It once caused
    // a runaway; that loop is now blocked at the insert (a rejected scan is not added to
    // the map). If the pose ever accelerates away with a healthy rmse, turn this off first.
    ep.use_constant_velocity = declare_parameter<bool>(
      "registration.use_constant_velocity", true);

    // [7] Tight coupling. `imu_prior_weight` selects the estimator outright:
    //   0   -> LOOSE: the IMU predicts, then ICP solves and the IMU gets no vote. With no
    //          IMU information, velocity and the biases are UNOBSERVABLE (the LiDAR
    //          Jacobian's columns for them are structurally zero), so we do not pretend to
    //          estimate them -- we run the 6-DoF SE(3) solve instead.
    //   > 0 -> TIGHT: one joint 15-DoF solve. See doc/7-tight-coupling.md.
    const double imu_prior_weight =
      declare_parameter<double>("registration.imu_prior_weight", 1.0);
    ep.use_tight = imu_prior_weight > 0.0;

    ep.tight.imu_prior_weight = imu_prior_weight;
    ep.tight.max_correspondence_distance = ep.reg.max_correspondence_distance;
    ep.tight.max_iterations = ep.reg.max_iterations;
    ep.tight.min_correspondences = ep.reg.min_correspondences;
    ep.tight.eps_translation = ep.reg.eps_translation;
    ep.tight.eps_rotation = ep.reg.eps_rotation;
    ep.tight.huber_delta = ep.reg.huber_delta;
    // Point-to-plane measurement noise. Irrelevant with one sensor (a global scale cancels
    // out of H xi = b); the moment the IMU enters the SAME normal equations it decides
    // which sensor is believed.
    ep.tight.lidar_sigma = declare_parameter<double>("registration.lidar_sigma", 0.05);
    ep.tight_warmup_scans = declare_parameter<int>("registration.tight_warmup_scans", 10);

    ep.gyro_noise = declare_parameter<double>("imu.gyro_noise", 1.7e-3);
    ep.accel_noise = declare_parameter<double>("imu.accel_noise", 2.0e-2);
    ep.bias_rw_gyro = declare_parameter<double>("imu.bias_rw_gyro", 1e-4);
    ep.bias_rw_accel = declare_parameter<double>("imu.bias_rw_accel", 1e-3);
    ep.bias_sigma0_gyro = declare_parameter<double>("imu.bias_sigma0_gyro", 0.01);
    ep.bias_sigma0_accel = declare_parameter<double>("imu.bias_sigma0_accel", 0.3);

    // Worker backlog. Small on purpose: if registration cannot keep up, a stale pose is
    // useless, so drop the oldest scan rather than accumulate latency without bound.
    max_queue_ = static_cast<std::size_t>(declare_parameter<int>("max_queue_size", 3));

    estimator_ = std::make_unique<LioEstimator>(ep, get_logger());

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
      lidar_topic_.c_str(), imu_topic_.c_str(), ep.voxel_leaf_size);
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

  /// React to whatever the sync watchdog saw. MeasureSync reports the FACT; the node owns
  /// the response, because only the node can log and only the node can reset the estimator.
  ///
  /// Returns true if the caller should DROP this message.
  bool onTimeJump(MeasureSync::TimeJump jump, const char * what)
  {
    switch (jump) {
      case MeasureSync::TimeJump::None:
        return false;

      case MeasureSync::TimeJump::Restart:
        // The source restarted (a bag loop). MeasureSync has already dropped its buffers;
        // the ESTIMATOR must go too -- its pose and map describe a world we have left.
        RCLCPP_WARN(
          get_logger(), "%s time jumped backwards -- source restarted; resetting estimator",
          what);
        reset();
        return false;   // the message itself is fine; it is the first of the new run

      case MeasureSync::TimeJump::OutOfOrder:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "%s messages arriving out of order; dropping. Are two publishers running "
          "(e.g. a stray 'ros2 bag play')?", what);
        return true;
    }
    return false;
  }

  /// Full reset: the world we mapped is gone. Callback thread.
  ///
  /// Only the callback-owned state is reset HERE. The ESTIMATOR belongs to the worker,
  /// and clearing the queue does NOT make it
  /// safe to touch: the worker may already have popped a scan and be inside
  /// handleScan(), so destroying the estimator under it is a use-after-free. So we only
  /// REQUEST the reset; the worker applies it to itself, in order, between scans.
  void reset()
  {
    sync_.clear();
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

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(buf_mutex_);
    const double t = stamp_sec(msg);
    if (onTimeJump(sync_.checkImu(t), "IMU")) {
      return;   // out of order: drop this one message
    }

    if (!imu_init_->initialized()) {
      if (imu_init_->add(*msg)) {
        onInitialized();
      }
      return;   // no scans are processed until the IMU is initialized
    }

    sync_.pushImu(msg);
    enqueueReady();
  }

  /// Publish the init result to the worker, which owns the estimator. Callback
  /// thread: it must not touch the estimator itself (see reset()).
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
    if (onTimeJump(sync_.checkLidar(t), "lidar")) {
      return;   // out of order: drop this one message
    }
    sync_.pushLidar(msg);
    enqueueReady();
  }

  /// Callback side: move every scan the IMU now covers onto the worker queue.
  /// Caller holds buf_mutex_. Does NO heavy work -- that is the whole point.
  void enqueueReady()
  {
    MeasureGroup meas;
    bool dropped_no_imu = false;
    while (sync_.next(meas, &dropped_no_imu)) {
      // (warning for a discarded frame is emitted after the loop -- see below)
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
          estimator_->reset();
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

          // Both reset and init hand the estimator over here rather than writing it from a
          // callback, so the worker's exclusive ownership actually holds.
          estimator_->initialize(init_pose, init_bias, init_gravity);
          continue;
        }
        meas = std::move(queue_.front());
        queue_.pop_front();
      }
      handleScan(meas);
    }
  }

  /// [7] OUTPUT. One measurement group: run it through the estimator, emit what came out.
  /// Worker thread only.
  ///
  /// NOT called processOne() any more -- it does not process anything. Stages [3]-[6] live
  /// in LioEstimator; this is where their results become log lines and topics.
  void handleScan(const MeasureGroup & meas)
  {
    const ScanResult r = estimator_->processScan(meas);
    if (!r.ok) {
      return;   // unusable scan (no cloud came out)
    }

    // --- [7] OUTPUT ---
    logScan(r);
    publishAll(meas, r);
  }

  /// [7] One line per scan. `map N vox` is the health check: with a correct pose the voxel
  /// count PLATEAUS (the same geometry re-observed lands in the same voxels). Climbing
  /// without bound means the same wall is being re-inserted at slightly wrong places --
  /// i.e. the pose is drifting. See doc/6-local-map.md.
  void logScan(const ScanResult & r)
  {
    const Eigen::Vector3d & t = estimator_->pose().translation();
    RCLCPP_INFO(
      get_logger(),
      "scan %zu->%zu | pose [%+.2f %+.2f %+.2f] | rmse %.3f (%d corr)%s | map %zu vox | q %zu",
      r.deskewed->size(), r.downsampled->size(),
      t.x(), t.y(), t.z(), r.rmse, r.correspondences,
      r.pose_trusted ? "" : " (DIVERGED)",
      estimator_->map().num_voxels(), queueSize());
  }

  /// [7] Odometry + TF, and the debug clouds (which skip serialization when unsubscribed).
  void publishAll(const MeasureGroup & meas, const ScanResult & r)
  {
    publishOdom(meas.lidar->header);
    publish(pub_deskewed_, r.deskewed, meas.lidar->header);
    publish(pub_downsampled_, r.downsampled, meas.lidar->header);
    publishMap(meas.lidar->header);
  }

  std::size_t queueSize()
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return queue_.size();
  }

  void publishOdom(const std_msgs::msg::Header & header)
  {
    const Eigen::Quaterniond q(estimator_->pose().linear());
    const Eigen::Vector3d & t = estimator_->pose().translation();

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
    const Eigen::Vector3d v_body =
      estimator_->pose().linear().transpose() * estimator_->velocity();
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
    publish(pub_map_, estimator_->map().target(), header, world_frame_);
  }

  std::string lidar_topic_, imu_topic_, output_frame_, world_frame_;
  double scan_guard_ = 0.12;

  // --- Callback side: sensor buffers. Never touched by the worker.
  std::mutex buf_mutex_;
  /// [2] Stage 2 lives here -- the buffers, the bracketing, and the time-jump watchdog.
  /// Touched only under buf_mutex_. See doc/2-sync.md.
  MeasureSync sync_{0.12};

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

  /// [1] IMU init lives on the callback side: it gates everything and its result is handed
  /// to the estimator over the queue (never written from a callback -- see workerLoop).
  std::unique_ptr<ImuInit> imu_init_;
  Eigen::Quaterniond q_il_ = Eigen::Quaterniond::Identity();   ///< extrinsic lidar -> IMU
  double accel_scale_ = kGravity;
  int init_samples_ = 200;
  double init_max_gyro_ = 0.1, init_max_acc_sd_ = 0.5;

  /// STAGES [3]-[6]. The node does not run the pipeline; it owns the thing that does.
  /// Worker thread only.
  std::unique_ptr<LioEstimator> estimator_;

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
