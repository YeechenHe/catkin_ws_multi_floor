#ifndef MULTI_FLOOR_NAVIGATION_H
#define MULTI_FLOOR_NAVIGATION_H

#include <ros/ros.h>


#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Empty.h"
#include <std_srvs/Empty.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <math.h>
#include <multi_floor_nav/IntTrigger.h>
#include <dynamic_reconfigure/client.h>
#include <amcl/AMCLConfig.h>

class MultiFloorNav{
    private:
        enum State{
            LOAD_MAP,
            INIT_POSE,
            CHECK_INITPOSE, 
            NAV_TO_GOAL,
            SEND_LIFT_0,
            ENTER_LIFT_LEVEL_0, 
            SEND_LIFT_1,
            EXIT_LIFT_LEVEL_1,
            DONE
        };
        State nav_state;
        int desired_map_level;
        bool received_amcl_pose, goal_sent, goal_active, to_start;
        double loop_rate, max_linear_error, max_angular_error;
        ros::Publisher initial_pose_pub, goal_pub, cmd_vel_pub, elevator_pub;
        ros::Subscriber amcl_pose_sub, odom_sub, move_base_status_sub, start_sub;
        ros::ServiceClient change_map_client;
        geometry_msgs::PoseWithCovarianceStamped curr_pose;
        nav_msgs::Odometry curr_odom, first_odom;
        actionlib_msgs::GoalStatusArray move_base_status_msg;
        geometry_msgs::Pose2D desired_init_pose, desired_goal_pose;
        multi_floor_nav::IntTrigger srv;
        // 模块 B：切层窗口参数切换（AP-AMCL）
        bool use_floor_window_params_;
        int amcl_window_max_particles_;
        double amcl_window_recovery_alpha_fast_;
        amcl::AMCLConfig amcl_config_saved_;  // 切层前保存的 AMCL 参数，用于恢复
        dynamic_reconfigure::Client<amcl::AMCLConfig>* amcl_reconf_client_;
        void apply_amcl_floor_window_params(bool use_window);

        // 模块 C：重定位完成判据优化（基于协方差的置信度 + 连续 K 帧）
        bool use_covariance_reloc_;
        double reloc_confidence_lambda_;
        double reloc_confidence_tau_;
        int reloc_confidence_K_;
        int reloc_confidence_consecutive_count_;
        ros::Time reloc_check_start_time_;
        double reloc_check_timeout_sec_;
        double reloc_check_timeout_L0_;   // L0 层 timeout（较短，允许更快 retry）
        double reloc_check_timeout_L1_;   // L1 层 timeout（跨层重定位需要更长收敛时间）

        // AMCL nomotion update: force AMCL to process laser scans while robot is stationary
        ros::ServiceClient nomotion_update_client_;
        double reloc_min_check_delay_sec_;

        // 修复缺陷一：per-floor 区域初始化 sigma（模块 A）
        // L0/L1 分别配置，允许按楼层激光环境独立调参
        double region_init_sigma_x_L0_, region_init_sigma_y_L0_, region_init_sigma_yaw_L0_;
        double region_init_sigma_x_L1_, region_init_sigma_y_L1_, region_init_sigma_yaw_L1_;

        // 修复缺陷二：per-floor nomotion 等待时间
        double reloc_min_check_delay_L0_;   // L0 层专用等待（单点 Baseline 同样受此约束）
        double reloc_min_check_delay_L1_;   // L1 层专用等待（SP-AMCL 粒子散布较慢）

        // 修复缺陷五：retry 退避策略
        // 每次 retry 将 sigma 乘以 backoff_factor，最多扩大 max_backoff_scale 倍
        int    reloc_retry_count_;          // 当前楼层已重试次数（INIT_POSE 进入时重置）
        double region_init_backoff_factor_; // 退避乘数（默认 1.3）
        double region_init_max_backoff_;    // sigma 最大倍数上限（默认 2.0）

        // 自适应 sigma：根据偏移量动态缩放，小偏移用小 sigma，大偏移用大 sigma
        double region_init_sigma_adaptive_alpha_; // sigma = alpha * offset_planar
        double region_init_sigma_min_scale_;      // 最小 sigma（m）
        double region_init_sigma_min_yaw_;        // 最小 yaw sigma（rad）

        // 通过后误差监控（用于采集 e_pass 和 after_pass 稳定性指标）
        bool reloc_post_monitoring_;
        ros::Time reloc_pass_time_;
        double reloc_post_monitor_duration_;
        geometry_msgs::Pose2D reloc_pass_desired_pose_;

        void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void movebaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
        void startCallback(const std_msgs::Empty::ConstPtr& msg);

        tf2::Quaternion convertYawtoQuartenion(double yaw);
        void set_init_pose(geometry_msgs::Pose2D pose);
        bool check_robot_pose(geometry_msgs::Pose2D pose);
        void send_simple_goal(geometry_msgs::Pose2D goal_pose);
        void send_cmd_vel(double x_vel, double theta_vel);
        void request_lift(std::string floor);
        double dist(geometry_msgs::Point A, geometry_msgs::Point B);
        double length(double x, double y, double z = 0);
        double getPositionOffset(geometry_msgs::Point A, geometry_msgs::Point B);
        bool reached_distance(double distance);
        void set_desired_level(int level_id);

    public:
        MultiFloorNav();
        void initialize (ros::NodeHandle& n);
        double getLoopRate();
        void execute();
        double test;
};

#endif