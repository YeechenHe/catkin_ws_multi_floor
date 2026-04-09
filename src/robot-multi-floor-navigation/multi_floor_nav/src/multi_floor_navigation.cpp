#include <multi_floor_nav/multi_floor_navigation.h>

#include <fstream>
#include <iomanip>
#include <ros/package.h>
#include <sstream>

namespace {
constexpr double kLiftEntryMinHardStopDistance = 0.18;
}  // namespace

using namespace std;

MultiFloorNav::MultiFloorNav(){
    nav_state = MultiFloorNav::State::LOAD_MAP;
    received_amcl_pose = false;
    goal_active = false;
    goal_sent = false;
    to_start = false;
    received_scan_ = false;
    loop_rate = 1.0;
    max_angular_error = 0.0;
    max_linear_error = 0.0;
    max_linear_error_L1_ = -1.0;  // >0：L1 单独门限；<=0 时与 L0 共用 max_linear_error
    curr_odom.pose.pose.orientation = first_odom.pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    desired_map_level = 0;
    use_floor_window_params_ = false;
    amcl_reconf_client_ = nullptr;
    use_covariance_reloc_ = false;
    reloc_confidence_consecutive_count_ = 0;
    reloc_check_timeout_sec_ = 45.0;
    reloc_check_timeout_L0_ = 25.0;
    reloc_check_timeout_L1_ = 45.0;
    reloc_post_monitoring_ = false;
    reloc_post_monitor_duration_ = 2.0;
    reloc_min_check_delay_sec_ = 6.0;
    // 修复缺陷一：per-floor sigma 默认值（0 = 关闭，与传统 AMCL 一致）
    region_init_sigma_x_L0_ = 0.0; region_init_sigma_y_L0_ = 0.0; region_init_sigma_yaw_L0_ = 0.0;
    region_init_sigma_x_L1_ = 0.0; region_init_sigma_y_L1_ = 0.0; region_init_sigma_yaw_L1_ = 0.0;
    // 修复缺陷二：per-floor delay 默认值（与全局一致，可通过参数单独覆盖）
    reloc_min_check_delay_L0_ = 6.0;
    reloc_min_check_delay_L1_ = 6.0;
    // 修复缺陷五：retry 退避默认值
    reloc_retry_count_ = 0;
    region_init_backoff_factor_ = 1.3;
    region_init_max_backoff_ = 2.0;
    // 自适应 sigma 默认值
    region_init_sigma_adaptive_alpha_ = 0.5;
    region_init_sigma_min_scale_ = 0.03;
    region_init_sigma_min_yaw_ = 0.02;
    scan_topic_ = "front/scan";
    lift_enter_linear_speed_ = 0.12;
    lift_enter_target_distance_ = 3.0;
    lift_front_safety_stop_distance_ = 0.35;
    lift_front_scan_half_angle_rad_ = 0.20;
    lift_enter_max_heading_error_rad_ = 0.10;
    lift_enter_align_kp_ = 1.8;
    lift_enter_align_max_w_ = 0.35;
    reloc_curve_logging_enabled_ = true;
    reloc_curve_csv_path_.clear();
    reloc_curve_csv_path_param_.clear();
    reloc_curve_sample_period_sec_ = 0.2;
    reloc_curve_pass_row_logged_ = false;
}

void MultiFloorNav::initialize(ros::NodeHandle& n){
    ros::NodeHandle np("~");

    ros::param::param<double>("~loop_rate", loop_rate, 5.0);
    ros::param::param<double>("~max_angular_error", max_angular_error, 0.05);
    ros::param::param<double>("~max_linear_error", max_linear_error, 0.5);
    ros::param::param<double>("~max_linear_error_L1", max_linear_error_L1_, -1.0);
    ros::param::param<std::string>("~scan_topic", scan_topic_, "front/scan");
    ros::param::param<double>("~lift_enter_linear_speed", lift_enter_linear_speed_, 0.12);
    ros::param::param<double>("~lift_enter_target_distance", lift_enter_target_distance_, 3.0);
    ros::param::param<double>("~lift_front_safety_stop_distance", lift_front_safety_stop_distance_, 0.35);
    ros::param::param<double>("~lift_front_scan_half_angle_rad", lift_front_scan_half_angle_rad_, 0.20);
    ros::param::param<double>("~lift_enter_max_heading_error_rad", lift_enter_max_heading_error_rad_, 0.10);
    ros::param::param<double>("~lift_enter_align_kp", lift_enter_align_kp_, 1.8);
    ros::param::param<double>("~lift_enter_align_max_w", lift_enter_align_max_w_, 0.35);
    ros::param::param<bool>("~reloc_curve_logging_enabled", reloc_curve_logging_enabled_, true);
    ros::param::param<std::string>("~reloc_curve_csv_path", reloc_curve_csv_path_param_, std::string(""));
    ros::param::param<double>("~reloc_curve_sample_period_sec", reloc_curve_sample_period_sec_, 0.2);
    if (reloc_curve_sample_period_sec_ < 0.05) {
        reloc_curve_sample_period_sec_ = 0.05;
    }
    reloc_curve_csv_path_.clear();
    if (max_linear_error_L1_ > 0.0) {
        ROS_INFO("[Multi Floor Nav] L1 linear error threshold: %.3f m (L0: %.3f m)", max_linear_error_L1_, max_linear_error);
    }

    initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    elevator_pub = n.advertise<std_msgs::String>("elevator", 1, true);
        
    amcl_pose_sub = n.subscribe("amcl_pose", 1, &MultiFloorNav::amclPoseCallback, this);
    odom_sub = n.subscribe("odometry/filtered", 1, &MultiFloorNav::odomCallback, this);
    move_base_status_sub = n.subscribe("move_base/status", 1, &MultiFloorNav::movebaseStatusCallback, this);
    start_sub = n.subscribe("start",1, &MultiFloorNav::startCallback, this);
    scan_sub = n.subscribe(scan_topic_, 1, &MultiFloorNav::scanCallback, this);

    change_map_client = n.serviceClient<multi_floor_nav::IntTrigger>("change_map");

    // Module B (AP-AMCL): disabled, use plain AMCL
    use_floor_window_params_ = false;
    ros::param::param<int>("~amcl_window_max_particles", amcl_window_max_particles_, 3500);
    ros::param::param<double>("~amcl_window_recovery_alpha_fast", amcl_window_recovery_alpha_fast_, 0.2);

    // Module C (covariance reloc criterion): disabled, use plain AMCL
    use_covariance_reloc_ = false;
    ros::param::param<bool>("~use_covariance_reloc", use_covariance_reloc_, false);
    ros::param::param<double>("~reloc_confidence_lambda", reloc_confidence_lambda_, 1.0);
    ros::param::param<double>("~reloc_confidence_tau", reloc_confidence_tau_, 0.8);
    ros::param::param<int>("~reloc_confidence_K", reloc_confidence_K_, 3);
    ros::param::param<double>("~reloc_check_timeout_sec", reloc_check_timeout_sec_, 45.0);
    ros::param::param<double>("~reloc_post_monitor_duration", reloc_post_monitor_duration_, 2.0);
    ros::param::param<double>("~reloc_min_check_delay_sec", reloc_min_check_delay_sec_, 6.0);
    nomotion_update_client_ = n.serviceClient<std_srvs::Empty>("request_nomotion_update");
    if (use_covariance_reloc_) {
        ROS_INFO("[Multi Floor Nav] Module C: covariance reloc criterion enabled (lambda=%.2f, tau=%.2f, K=%d, post_monitor=%.1fs, min_delay=%.1fs)",
                 reloc_confidence_lambda_, reloc_confidence_tau_, reloc_confidence_K_, reloc_post_monitor_duration_, reloc_min_check_delay_sec_);
    }

    // Module A (SPA-AMCL region init): load configurable sigma values.
    // If no params are provided, defaults remain 0.0 (plain AMCL single-point init).
    double region_init_sigma_x = 0.0;
    double region_init_sigma_y = 0.0;
    double region_init_sigma_yaw = 0.0;
    ros::param::param<double>("~region_init_sigma_x",   region_init_sigma_x,   0.0);
    ros::param::param<double>("~region_init_sigma_y",   region_init_sigma_y,   0.0);
    ros::param::param<double>("~region_init_sigma_yaw", region_init_sigma_yaw, 0.0);

    ros::param::param<double>("~region_init_sigma_x_L0",   region_init_sigma_x_L0_,   region_init_sigma_x);
    ros::param::param<double>("~region_init_sigma_y_L0",   region_init_sigma_y_L0_,   region_init_sigma_y);
    ros::param::param<double>("~region_init_sigma_yaw_L0", region_init_sigma_yaw_L0_, region_init_sigma_yaw);
    ros::param::param<double>("~region_init_sigma_x_L1",   region_init_sigma_x_L1_,   region_init_sigma_x);
    ros::param::param<double>("~region_init_sigma_y_L1",   region_init_sigma_y_L1_,   region_init_sigma_y);
    ros::param::param<double>("~region_init_sigma_yaw_L1", region_init_sigma_yaw_L1_, region_init_sigma_yaw);

    // 修复缺陷二：per-floor nomotion 等待时间（未设时继承全局 reloc_min_check_delay_sec）
    ros::param::param<double>("~reloc_min_check_delay_L0", reloc_min_check_delay_L0_, reloc_min_check_delay_sec_);
    ros::param::param<double>("~reloc_min_check_delay_L1", reloc_min_check_delay_L1_, reloc_min_check_delay_sec_);

    // per-floor timeout（未设时继承全局 reloc_check_timeout_sec）
    ros::param::param<double>("~reloc_check_timeout_L0", reloc_check_timeout_L0_, reloc_check_timeout_sec_);
    ros::param::param<double>("~reloc_check_timeout_L1", reloc_check_timeout_L1_, reloc_check_timeout_sec_);

    // 修复缺陷五：retry 退避参数
    ros::param::param<double>("~region_init_backoff_factor", region_init_backoff_factor_, 1.3);
    ros::param::param<double>("~region_init_max_backoff",    region_init_max_backoff_,    2.0);

    // 自适应 sigma 参数
    ros::param::param<double>("~region_init_sigma_adaptive_alpha", region_init_sigma_adaptive_alpha_, 0.5);
    ros::param::param<double>("~region_init_sigma_min_scale",      region_init_sigma_min_scale_,      0.03);
    ros::param::param<double>("~region_init_sigma_min_yaw",        region_init_sigma_min_yaw_,        0.02);

    ROS_INFO("[Multi Floor Nav] Module A sigma: L0=(%.3f,%.3f,%.3f) L1=(%.3f,%.3f,%.3f)",
             region_init_sigma_x_L0_, region_init_sigma_y_L0_, region_init_sigma_yaw_L0_,
             region_init_sigma_x_L1_, region_init_sigma_y_L1_, region_init_sigma_yaw_L1_);
    ROS_INFO("[Multi Floor Nav] Module A delay: L0=%.1fs  L1=%.1fs  backoff: factor=%.2f max=%.2f",
             reloc_min_check_delay_L0_, reloc_min_check_delay_L1_,
             region_init_backoff_factor_, region_init_max_backoff_);
    if (use_covariance_reloc_) {
        ROS_INFO("[Multi Floor Nav] Module C timeout: L0=%.1fs  L1=%.1fs",
                 reloc_check_timeout_L0_, reloc_check_timeout_L1_);
    }
}

void MultiFloorNav::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    received_amcl_pose = true;
    curr_pose.pose = msg->pose;
}

void MultiFloorNav::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    curr_odom = *msg;
}

void MultiFloorNav::movebaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    move_base_status_msg = *msg;
}

void MultiFloorNav::startCallback(const std_msgs::Empty::ConstPtr& msg){
    to_start = true;
}

void MultiFloorNav::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    latest_scan_ = *msg;
    received_scan_ = true;
}

double MultiFloorNav::getFrontObstacleDistance() const{
    if (!received_scan_) {
        return std::numeric_limits<double>::infinity();
    }

    double min_range = std::numeric_limits<double>::infinity();
    int32_t count = static_cast<int32_t>(latest_scan_.ranges.size());
    for (int32_t i = 0; i < count; ++i) {
        double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;
        if (fabs(angle) > lift_front_scan_half_angle_rad_) {
            continue;
        }
        float range = latest_scan_.ranges.at(i);
        if (!std::isfinite(range)) {
            continue;
        }
        if (range >= latest_scan_.range_min && range <= latest_scan_.range_max && range < min_range) {
            min_range = range;
        }
    }
    return min_range;
}

tf2::Quaternion MultiFloorNav::convertYawtoQuartenion(double yaw){
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    quat.normalize();
    return quat;
}

void MultiFloorNav::set_init_pose(geometry_msgs::Pose2D pose){
    // 初值扰动（用于补充实验：模拟电梯出口估计不准，发布的 initialpose 带偏差）
    double offset_x = 0.0, offset_y = 0.0, offset_theta = 0.0;
    ros::NodeHandle np("~");
    ros::param::param<double>("~init_pose_offset_x", offset_x, 0.0);
    ros::param::param<double>("~init_pose_offset_y", offset_y, 0.0);
    ros::param::param<double>("~init_pose_offset_theta", offset_theta, 0.0);
    // 仅在 L1 施加扰动；L0 使用无偏初值，避免在起始层因人工扰动导致重定位卡死。
    if (desired_map_level == 0) {
        offset_x = 0.0;
        offset_y = 0.0;
        offset_theta = 0.0;
    }
    pose.x += offset_x;
    pose.y += offset_y;
    pose.theta += offset_theta;
    if (offset_x != 0.0 || offset_y != 0.0 || offset_theta != 0.0)
        ROS_WARN("[Multi Floor Nav] Initial pose perturbation applied: offset_x=%.2f, offset_y=%.2f, offset_theta=%.2f", offset_x, offset_y, offset_theta);

    geometry_msgs::PoseWithCovarianceStamped init_pose;

    init_pose.header.frame_id = "map";
    init_pose.header.stamp = ros::Time::now();

    init_pose.pose.pose.position.x = pose.x;
    init_pose.pose.pose.position.y = pose.y;
    init_pose.pose.pose.position.z = 0.0;

    tf2::Quaternion quat;
    quat = convertYawtoQuartenion(pose.theta);

    init_pose.pose.pose.orientation.x= quat[0];
    init_pose.pose.pose.orientation.y= quat[1];
    init_pose.pose.pose.orientation.z= quat[2];
    init_pose.pose.pose.orientation.w= quat[3];

    // 修复缺陷一：按楼层选取对应的 sigma（L0 or L1）
    double sigma_x   = (desired_map_level == 0) ? region_init_sigma_x_L0_   : region_init_sigma_x_L1_;
    double sigma_y   = (desired_map_level == 0) ? region_init_sigma_y_L0_   : region_init_sigma_y_L1_;
    double sigma_yaw = (desired_map_level == 0) ? region_init_sigma_yaw_L0_ : region_init_sigma_yaw_L1_;

    // 自适应 sigma：根据偏移量动态缩放，使 sigma 与不确定性匹配
    // 注意：模块 A 开启时，即便无偏移也应保留配置的区域初始化 sigma，
    // 以实现 SPA-AMCL 的“区域粒子散布”效果；纯 AMCL 对照组通过不加载模块 A
    // 参数文件使 sigma=0 达成。
    if (sigma_x > 0.0 || sigma_y > 0.0 || sigma_yaw > 0.0) {
        double offset_planar = sqrt(offset_x * offset_x + offset_y * offset_y);
        if (offset_planar < 1e-6 && fabs(offset_theta) < 1e-6) {
            ROS_INFO("[Multi Floor Nav] Adaptive sigma: no offset detected, keep configured sigma (%.3f, %.3f, %.3f)",
                     sigma_x, sigma_y, sigma_yaw);
        } else {
            double adaptive_scale = std::max(region_init_sigma_min_scale_,
                                             region_init_sigma_adaptive_alpha_ * offset_planar);
            sigma_x   = std::min(sigma_x,   adaptive_scale);
            sigma_y   = std::min(sigma_y,   adaptive_scale);
            double adaptive_yaw = region_init_sigma_adaptive_alpha_ * fabs(offset_theta)
                                  + region_init_sigma_min_yaw_;
            sigma_yaw = std::min(sigma_yaw, adaptive_yaw);
            ROS_INFO("[Multi Floor Nav] Adaptive sigma: offset_planar=%.3f, adaptive_scale=%.3f, adaptive_yaw=%.3f → sigma(%.3f, %.3f, %.3f)",
                     offset_planar, adaptive_scale, adaptive_yaw, sigma_x, sigma_y, sigma_yaw);
        }
    }

    // 修复缺陷五：retry 退避——每次重试将 sigma 乘以退避倍数，上限 region_init_max_backoff_ 倍
    if (reloc_retry_count_ > 0 && (sigma_x > 0.0 || sigma_y > 0.0 || sigma_yaw > 0.0)) {
        double scale = std::min(std::pow(region_init_backoff_factor_, reloc_retry_count_),
                                region_init_max_backoff_);
        sigma_x   *= scale;
        sigma_y   *= scale;
        sigma_yaw *= scale;
        ROS_WARN("[Multi Floor Nav] Retry #%d: sigma scaled by %.3f → (%.3f, %.3f, %.3f)",
                 reloc_retry_count_, scale, sigma_x, sigma_y, sigma_yaw);
    }

    for (int i = 0; i < 36; ++i) init_pose.pose.covariance[i] = 0.0;
    if (sigma_x   > 0.0) init_pose.pose.covariance[0]  = sigma_x   * sigma_x;
    if (sigma_y   > 0.0) init_pose.pose.covariance[7]  = sigma_y   * sigma_y;
    if (sigma_yaw > 0.0) init_pose.pose.covariance[35] = sigma_yaw * sigma_yaw;

    ROS_INFO("[Multi Floor Nav] Initializing Robot at x: %.1f, y: %.1f, yaw: %.1f (level=%d, sigma: x=%.3f, y=%.3f, yaw=%.3f, retry=%d)",
             pose.x, pose.y, pose.theta, desired_map_level, sigma_x, sigma_y, sigma_yaw, reloc_retry_count_);
    initial_pose_pub.publish(init_pose);
}

bool MultiFloorNav::check_robot_pose(geometry_msgs::Pose2D pose){
    while (!received_amcl_pose);

    double amcl_x = curr_pose.pose.pose.position.x;
    double amcl_y = curr_pose.pose.pose.position.y;
    double diff_x = fabs(pose.x - amcl_x);
    double diff_y = fabs(pose.y - amcl_y);
    double linear_error = sqrt(pow(diff_x,2) + pow(diff_y,2));
    double amcl_yaw = tf::getYaw(curr_pose.pose.pose.orientation);
    double angular_error = fabs(angles::normalize_angle(pose.theta - amcl_yaw));
    ROS_INFO("[Multi Floor Nav] Current Robot Pose, x: %.2f, y: %.2f, yaw: %.2f", amcl_x, amcl_y, amcl_yaw);
    ROS_INFO("[Multi Floor Nav] Linear Error: %.2f, Angular Error: %.2f", linear_error, angular_error);

    double eps_lin = max_linear_error;
    if (desired_map_level == 1 && max_linear_error_L1_ > 0.0) {
        eps_lin = max_linear_error_L1_;
    }
    bool linear_ok = (linear_error < eps_lin);
    bool angular_ok = (angular_error < max_angular_error);
    bool pose_ok = (linear_ok && angular_ok);

    if (use_covariance_reloc_) {
        // 模块 C：C_t = exp(-λ · tr(Σ))，tr(Σ) 取 x,y,yaw 方差（covariance 索引 0,7,35）
        double tr_sigma = curr_pose.pose.covariance[0] + curr_pose.pose.covariance[7] + curr_pose.pose.covariance[35];
        double C_t = exp(-reloc_confidence_lambda_ * tr_sigma);
        bool confidence_ok = (C_t > reloc_confidence_tau_);
        ROS_INFO("[Multi Floor Nav] Module C: tr(Σ)=%.4f, C_t=%.4f (tau=%.2f), frame_ok=%d",
                 tr_sigma, C_t, reloc_confidence_tau_, (pose_ok && confidence_ok) ? 1 : 0);
        if (pose_ok && confidence_ok) {
            reloc_confidence_consecutive_count_++;
            bool passed = (reloc_confidence_consecutive_count_ >= reloc_confidence_K_);
            if (passed)
                ROS_INFO("[Multi Floor Nav] Module C: reloc passed (consecutive %d >= K=%d)", reloc_confidence_consecutive_count_, reloc_confidence_K_);
            return passed;
        } else {
            reloc_confidence_consecutive_count_ = 0;
            return false;
        }
    }

    return pose_ok;
}

void MultiFloorNav::send_simple_goal(geometry_msgs::Pose2D goal_pose){
    geometry_msgs::PoseStamped goal;

    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    goal.pose.position.x = goal_pose.x;
    goal.pose.position.y = goal_pose.y;
    
    tf2::Quaternion quat;
    quat = convertYawtoQuartenion(goal_pose.theta);
    
    goal.pose.orientation.x = quat[0];
    goal.pose.orientation.y = quat[1];
    goal.pose.orientation.z = quat[2];
    goal.pose.orientation.w = quat[3];

    ROS_INFO("[Multi Floor Nav] Sending Robot to x: %.1f, y: %.1f, yaw: %.1f", 
        goal_pose.x, goal_pose.y, goal_pose.theta);
    goal_pub.publish(goal);
}

void MultiFloorNav::send_cmd_vel(double x_vel=0.0, double theta_vel=0.0){
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = x_vel;
    cmd_vel.angular.z = theta_vel;
    cmd_vel.linear.y = cmd_vel.linear.z = cmd_vel.angular.x = cmd_vel.angular.y = 0.0;

    cmd_vel_pub.publish(cmd_vel);
}

void MultiFloorNav::request_lift(string floor){
    std_msgs::String floor_id;

    floor_id.data = floor;

    elevator_pub.publish(floor_id);
}

double MultiFloorNav::dist(geometry_msgs::Point A, geometry_msgs::Point B) {
  return length(B.x - A.x, B.y - A.y, A.z - B.z);
}

double MultiFloorNav::length(double diff_x, double diff_y, double diff_z) {
  return sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
}

double MultiFloorNav::getPositionOffset(geometry_msgs::Point A, geometry_msgs::Point B) {
  return dist(A, B);
}   

bool MultiFloorNav::reached_distance(double distance){
  double distance_travelled = getPositionOffset(
    curr_odom.pose.pose.position, first_odom.pose.pose.position);
  if(distance_travelled >= distance){
    return true;
  }
  return false;
}

void MultiFloorNav::apply_amcl_floor_window_params(bool use_window){
    if (!amcl_reconf_client_) return;
    amcl::AMCLConfig config;
    if (!amcl_reconf_client_->getCurrentConfiguration(config, ros::Duration(3.0))) {
        ROS_WARN("[Multi Floor Nav] AP-AMCL: failed to get current AMCL config (timeout 3s)");
        return;
    }
    if (use_window) {
        amcl_config_saved_ = config;
        config.max_particles = amcl_window_max_particles_;
        config.recovery_alpha_fast = amcl_window_recovery_alpha_fast_;
        amcl_reconf_client_->setConfiguration(config);
        ROS_INFO("[Multi Floor Nav] AP-AMCL: applied floor-window params (max_particles=%d, recovery_alpha_fast=%.2f)",
                 amcl_window_max_particles_, amcl_window_recovery_alpha_fast_);
    } else {
        amcl_reconf_client_->setConfiguration(amcl_config_saved_);
        ROS_INFO("[Multi Floor Nav] AP-AMCL: restored AMCL params (max_particles=%d, recovery_alpha_fast=%.2f)",
                 amcl_config_saved_.max_particles, amcl_config_saved_.recovery_alpha_fast);
    }
}

void MultiFloorNav::startRelocCurveLog() {
    if (!reloc_curve_logging_enabled_) {
        return;
    }

    std::string csv_path = reloc_curve_csv_path_param_;
    if (csv_path.empty()) {
        std::string package_path = ros::package::getPath("multi_floor_nav");
        std::ostringstream stream;
        stream << package_path << "/experiment_logs/reloc_curve_"
               << ros::WallTime::now().sec << "_L" << desired_map_level << ".csv";
        csv_path = stream.str();
    }
    reloc_curve_csv_path_ = csv_path;

    std::ofstream out(csv_path.c_str(), std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        ROS_WARN("[Multi Floor Nav] Failed to create reloc curve csv: %s", csv_path.c_str());
        reloc_curve_logging_enabled_ = false;
        return;
    }

    out << "elapsed_sec,linear_error_nom,angular_error_nom,linear_error_pub,angular_error_pub,"
           "tr_sigma,C_t,pose_ok,pass_now\n";
    out.close();

    reloc_curve_start_time_ = ros::Time::now();
    reloc_curve_last_sample_time_ = ros::Time(0);
    reloc_curve_pass_row_logged_ = false;
    ROS_INFO("[Multi Floor Nav] Reloc curve logging to: %s", csv_path.c_str());
}

void MultiFloorNav::appendRelocCurveSample(const double elapsed_sec,
                                           const double linear_error_nom,
                                           const double angular_error_nom,
                                           const double linear_error_pub,
                                           const double angular_error_pub,
                                           const double tr_sigma,
                                           const double c_t,
                                           const bool pose_ok,
                                           const bool pass_now) {
    if (!reloc_curve_logging_enabled_ || reloc_curve_csv_path_.empty()) {
        return;
    }

    std::ofstream out(reloc_curve_csv_path_.c_str(), std::ios::out | std::ios::app);
    if (!out.is_open()) {
        ROS_WARN_THROTTLE(1.0, "[Multi Floor Nav] Failed to append reloc curve csv: %s",
                          reloc_curve_csv_path_.c_str());
        return;
    }

    out << std::fixed << std::setprecision(4)
        << elapsed_sec << ","
        << linear_error_nom << ","
        << angular_error_nom << ","
        << linear_error_pub << ","
        << angular_error_pub << ","
        << tr_sigma << ","
        << c_t << ","
        << (pose_ok ? 1 : 0) << ","
        << (pass_now ? 1 : 0) << "\n";
    out.close();
}

void MultiFloorNav::execute(){
    if (reloc_post_monitoring_ && received_amcl_pose) {
        double dt_post = (ros::Time::now() - reloc_pass_time_).toSec();
        if (dt_post <= reloc_post_monitor_duration_) {
            double ax = curr_pose.pose.pose.position.x;
            double ay = curr_pose.pose.pose.position.y;
            double ayaw = tf::getYaw(curr_pose.pose.pose.orientation);
            double ple = sqrt(pow(reloc_pass_desired_pose_.x - ax, 2) + pow(reloc_pass_desired_pose_.y - ay, 2));
            double pae = fabs(angles::normalize_angle(reloc_pass_desired_pose_.theta - ayaw));
            ROS_INFO("[Multi Floor Nav] POST_PASS: dt=%.3fs, linear_error=%.4f, angular_error=%.4f", dt_post, ple, pae);
        } else {
            reloc_post_monitoring_ = false;
        }
    }

    switch(nav_state){
        case MultiFloorNav::State::LOAD_MAP:
            if(to_start){
                srv.request.req_int = desired_map_level;
                if(change_map_client.call(srv)){
                    ROS_INFO("[Multi Floor Nav] %s", srv.response.message.c_str());
                    // 修复缺陷五：切换楼层时重置退避计数器
                    reloc_retry_count_ = 0;
                    if(desired_map_level == 0){
                        desired_init_pose.x = 4.0;
                        desired_init_pose.y = -5.0;
                        desired_init_pose.theta = 0.0;
                    }
                    else{
                        desired_init_pose.x = 3.0;
                        desired_init_pose.y = -0.5;
                        desired_init_pose.theta = M_PI;
                        // 模块 B：进入 L1 切层窗口，临时提高 AMCL 粒子数/恢复速度
                        if (use_floor_window_params_)
                            apply_amcl_floor_window_params(true);
                    }
                    nav_state = MultiFloorNav::State::INIT_POSE;
                }
                else{
                    ROS_INFO("[Multi Floor Nav] Failed to load map for level %d", desired_map_level);
                }
            }
            break;
        case MultiFloorNav::State::INIT_POSE:
            if (use_covariance_reloc_)
                reloc_confidence_consecutive_count_ = 0;
            // 修复缺陷五：首次进入楼层时重置退避计数器（LOAD_MAP → INIT_POSE 切换时 retry=0）
            // retry 后再次进入 INIT_POSE 则不重置，由 CHECK_INITPOSE 的 else 分支递增
            set_init_pose(desired_init_pose);
            startRelocCurveLog();
            ROS_INFO("[Multi Floor Nav] Initializing Pose to x: %.2f, y: %.2f at level: %d",
                        desired_init_pose.x, desired_init_pose.y, desired_map_level);
            reloc_check_start_time_ = ros::Time::now();
            nav_state = MultiFloorNav::State::CHECK_INITPOSE;
            ros::Duration(1.0).sleep();
            break;
        case MultiFloorNav::State::CHECK_INITPOSE:
        {
            std_srvs::Empty nomotion_srv;
            nomotion_update_client_.call(nomotion_srv);

            double floor_delay = (desired_map_level == 0) ? reloc_min_check_delay_L0_ : reloc_min_check_delay_L1_;
            double elapsed_since_init = (ros::Time::now() - reloc_check_start_time_).toSec();
            const bool after_delay = elapsed_since_init >= floor_delay;
            bool reloc_pass = false;

            if (received_amcl_pose) {
                const double amcl_x = curr_pose.pose.pose.position.x;
                const double amcl_y = curr_pose.pose.pose.position.y;
                const double amcl_yaw = tf::getYaw(curr_pose.pose.pose.orientation);
                const double linear_error_nom =
                    sqrt(pow(desired_init_pose.x - amcl_x, 2) + pow(desired_init_pose.y - amcl_y, 2));
                const double angular_error_nom =
                    fabs(angles::normalize_angle(desired_init_pose.theta - amcl_yaw));

                double offset_x = 0.0;
                double offset_y = 0.0;
                double offset_theta = 0.0;
                ros::param::param<double>("~init_pose_offset_x", offset_x, 0.0);
                ros::param::param<double>("~init_pose_offset_y", offset_y, 0.0);
                ros::param::param<double>("~init_pose_offset_theta", offset_theta, 0.0);
                if (desired_map_level == 0) {
                    offset_x = 0.0;
                    offset_y = 0.0;
                    offset_theta = 0.0;
                }
                const double pub_x = desired_init_pose.x + offset_x;
                const double pub_y = desired_init_pose.y + offset_y;
                const double pub_yaw = desired_init_pose.theta + offset_theta;
                const double linear_error_pub =
                    sqrt(pow(pub_x - amcl_x, 2) + pow(pub_y - amcl_y, 2));
                const double angular_error_pub =
                    fabs(angles::normalize_angle(pub_yaw - amcl_yaw));

                double eps_lin = max_linear_error;
                if (desired_map_level == 1 && max_linear_error_L1_ > 0.0) {
                    eps_lin = max_linear_error_L1_;
                }
                const bool pose_ok_for_curve =
                    after_delay && (linear_error_nom < eps_lin) && (angular_error_nom < max_angular_error);
                const double tr_sigma =
                    curr_pose.pose.covariance[0] + curr_pose.pose.covariance[7] +
                    curr_pose.pose.covariance[35];
                const double c_t = exp(-reloc_confidence_lambda_ * tr_sigma);
                reloc_pass = after_delay && check_robot_pose(desired_init_pose);

                const ros::Time now = ros::Time::now();
                const double elapsed_curve = (now - reloc_curve_start_time_).toSec();
                bool should_sample = false;
                if (reloc_curve_logging_enabled_ && !reloc_curve_csv_path_.empty()) {
                    if (reloc_curve_last_sample_time_.isZero()) {
                        should_sample = true;
                    } else if ((now - reloc_curve_last_sample_time_).toSec() >= reloc_curve_sample_period_sec_) {
                        should_sample = true;
                    }
                    if (reloc_pass && !reloc_curve_pass_row_logged_) {
                        should_sample = true;
                    }
                }
                if (should_sample) {
                    appendRelocCurveSample(elapsed_curve,
                                           linear_error_nom,
                                           angular_error_nom,
                                           linear_error_pub,
                                           angular_error_pub,
                                           tr_sigma,
                                           c_t,
                                           pose_ok_for_curve,
                                           reloc_pass);
                    reloc_curve_last_sample_time_ = now;
                    if (reloc_pass) {
                        reloc_curve_pass_row_logged_ = true;
                    }
                }
            }

            if (!after_delay) {
                ROS_INFO_THROTTLE(1.0, "[Multi Floor Nav] Waiting for AMCL nomotion updates (%.1f/%.1fs, L%d)...",
                                  elapsed_since_init, floor_delay, desired_map_level);
                break;
            }

            if(reloc_pass){
                ROS_INFO("[Multi Floor Nav] Robot at correct pose");
                {
                    const double amcl_px = curr_pose.pose.pose.position.x;
                    const double amcl_py = curr_pose.pose.pose.position.y;
                    const double amcl_pyaw = tf::getYaw(curr_pose.pose.pose.orientation);
                    const double le =
                        sqrt(pow(desired_init_pose.x - amcl_px, 2) + pow(desired_init_pose.y - amcl_py, 2));
                    const double ae =
                        fabs(angles::normalize_angle(desired_init_pose.theta - amcl_pyaw));
                    double offset_x = 0.0;
                    double offset_y = 0.0;
                    double offset_theta = 0.0;
                    ros::param::param<double>("~init_pose_offset_x", offset_x, 0.0);
                    ros::param::param<double>("~init_pose_offset_y", offset_y, 0.0);
                    ros::param::param<double>("~init_pose_offset_theta", offset_theta, 0.0);
                    if (desired_map_level == 0) {
                        offset_x = 0.0;
                        offset_y = 0.0;
                        offset_theta = 0.0;
                    }
                    const double pub_x = desired_init_pose.x + offset_x;
                    const double pub_y = desired_init_pose.y + offset_y;
                    const double pub_yaw = desired_init_pose.theta + offset_theta;
                    const double le_pub =
                        sqrt(pow(pub_x - amcl_px, 2) + pow(pub_y - amcl_py, 2));
                    const double ae_pub =
                        fabs(angles::normalize_angle(pub_yaw - amcl_pyaw));
                    const double tr_s =
                        curr_pose.pose.covariance[0] + curr_pose.pose.covariance[7] +
                        curr_pose.pose.covariance[35];
                    const double ct = exp(-reloc_confidence_lambda_ * tr_s);
                    ROS_INFO("[Multi Floor Nav] RELOC_PASS: linear_error=%.4f, angular_error=%.4f, linear_error_init=%.4f, angular_error_init=%.4f, tr_sigma=%.4f, C_t=%.4f",
                             le, ae, le_pub, ae_pub, tr_s, ct);
                    reloc_post_monitoring_ = true;
                    reloc_pass_time_ = ros::Time::now();
                    reloc_pass_desired_pose_ = desired_init_pose;
                }
                if (desired_map_level == 1 && use_floor_window_params_)
                    apply_amcl_floor_window_params(false);
                nav_state = MultiFloorNav::State::NAV_TO_GOAL;
                ros::Duration(1.0).sleep();
            }
            else{
                double floor_timeout = (desired_map_level == 0) ? reloc_check_timeout_L0_ : reloc_check_timeout_L1_;
                if (use_covariance_reloc_ &&
                    elapsed_since_init < floor_timeout) {
                    ;
                } else {
                    // 修复缺陷五：每次 retry 递增退避计数器，set_init_pose 会据此扩大 sigma
                    reloc_retry_count_++;
                    ROS_INFO("[Multi Floor Nav] Setting initial pose failed. Will Retry (retry #%d)", reloc_retry_count_);
                    nav_state = MultiFloorNav::State::INIT_POSE;
                }
            }
        }
            break;
        case MultiFloorNav::State::NAV_TO_GOAL:
            if(!goal_sent){
                if(desired_map_level ==0 ){
                    desired_goal_pose.x = 3.0;
                    desired_goal_pose.y = -0.52;
                    desired_goal_pose.theta = M_PI;                    
                }
                else{
                    desired_goal_pose.x = 4.0;
                    desired_goal_pose.y = 5.0;
                    desired_goal_pose.theta = 0.0;    
                }
                ROS_INFO_THROTTLE(30, "[Multi Floor Nav] Will send goal to x: %.f, y: %.f, at level: %d", 
                                desired_goal_pose.x, desired_goal_pose.y, desired_map_level); 
                send_simple_goal(desired_goal_pose);
                goal_sent = true;
            }
            if(!move_base_status_msg.status_list.empty()){
                if(move_base_status_msg.status_list[0].status == actionlib_msgs::GoalStatus::ACTIVE){
                goal_active = true;
                }
                else if(goal_active && move_base_status_msg.status_list[0].status == actionlib_msgs::GoalStatus::SUCCEEDED){
                if(desired_map_level == 0)
                    nav_state = MultiFloorNav::State::SEND_LIFT_0;
                else
                    nav_state = MultiFloorNav::State::DONE;
                goal_sent = goal_active = false;
                first_odom = curr_odom;
                send_cmd_vel(); //stops the amr in case of still oscillating (theorectically shouldn't)
                }
            }
            break;
        case MultiFloorNav::State::SEND_LIFT_0:
            ROS_INFO_ONCE("[Multi Floor Nav] Requesting Lift to Level 0");
            request_lift("0");
            nav_state = MultiFloorNav::State::ENTER_LIFT_LEVEL_0;
            break;

        case MultiFloorNav::State::ENTER_LIFT_LEVEL_0:
            ROS_INFO_ONCE("[Multi Floor Nav] Entering Lift at Level 0");
            {
                double front_obstacle_distance = getFrontObstacleDistance();
                if (std::isfinite(front_obstacle_distance) &&
                    front_obstacle_distance < std::max(lift_front_safety_stop_distance_, kLiftEntryMinHardStopDistance)) {
                    send_cmd_vel();
                    ROS_WARN_THROTTLE(0.5,
                                      "[Multi Floor Nav] Lift entry safety stop: front obstacle %.3f m < %.3f m",
                                      front_obstacle_distance,
                                      std::max(lift_front_safety_stop_distance_, kLiftEntryMinHardStopDistance));
                    break;
                }

                send_cmd_vel(lift_enter_linear_speed_);
                ROS_INFO_THROTTLE(0.5,
                                  "[Multi Floor Nav] Lift entry simple drive: front=%.3f m, cmd_v=%.3f, target_dist=%.3f",
                                  front_obstacle_distance,
                                  lift_enter_linear_speed_,
                                  lift_enter_target_distance_);
            }

            if(reached_distance(lift_enter_target_distance_)){
                send_cmd_vel();
                nav_state= MultiFloorNav::State::SEND_LIFT_1;
                first_odom = curr_odom;
                ros::Duration(5).sleep(); // wait 5 sec for the lift to close
            }      
            break;
        case MultiFloorNav::State::SEND_LIFT_1:
            ROS_INFO("[Multi Floor Nav] Requesting Lift to Level 1");
            request_lift("1");
            ros::Duration(7).sleep(); // wait 7 secs for the lift to travel to level 1 and open
            nav_state= MultiFloorNav::State::EXIT_LIFT_LEVEL_1;
            break;

        case MultiFloorNav::State::EXIT_LIFT_LEVEL_1:
            // request_lift("1");
            ROS_INFO_ONCE("[Multi Floor Nav] Exiting Lift at Level 1");
            send_cmd_vel(-0.25); 
            if(reached_distance(3.0)){
                send_cmd_vel();
                nav_state= MultiFloorNav::State::LOAD_MAP;
                first_odom = curr_odom;
                desired_map_level = 1;
            }
            break;
        case MultiFloorNav::State::DONE:
            ROS_INFO_ONCE("[Multi Floor Nav] Robot arrived at Goal");
            break;
    }
}

double MultiFloorNav::getLoopRate(){
    return loop_rate;
}

int main(int argc, char** argv) {   
    ros::init(argc, argv, "multi_floor_navigation_node");
    ros::NodeHandle n;

    MultiFloorNav multi_floor_nav;
    multi_floor_nav.initialize(n);
    double rate = multi_floor_nav.getLoopRate();
    if(rate <= 0.0){
        rate = 1.0;
    }
    ros::Rate loop_rate(rate);

    while (ros::ok()) {
        ros::spinOnce();
        multi_floor_nav.execute();
        loop_rate.sleep();
    }

    ros::spin();

    return (0);
}

