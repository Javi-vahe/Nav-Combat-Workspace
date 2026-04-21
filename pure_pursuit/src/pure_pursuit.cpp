/* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).

   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and
   Sorniotti et al. Path tracking for Automated Driving, 2017.
 */

#include <string>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

#include <kdl/frames.hpp>

// TODO: Figure out how to use tf2 DataConversions
// for more elegant and compact code
// #include <tf2_kdl/tf2_kdl.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <pure_pursuit/PurePursuitConfig.h>

#include "pure_pursuit/path_checker.hpp"

#define UP_MODE 0
#define DOWN_MODE 1

enum PlannerStatus
{
  PLANNER_IDLE = 0, // 空闲状态
  PLANNER_RUNNING = 1,
  PLANNER_FINISHED = 2,
  PLANNER_FAILED = 3,
  PLANNER_ABORTED = 4
};

using std::string;

double normalizeRadian(double radian)
{
  radian = fmod(radian, 2.0 * M_PI); // 先取模
  // 将 [-2π, 2π] 映射到 [-π, π)
  if (radian > M_PI)
    radian -= 2.0 * M_PI;
  else if (radian < -M_PI)
    radian += 2.0 * M_PI;
  return radian;
}

class PurePursuit
{
public:
  //! Constructor
  PurePursuit();

  //! Compute velocit commands each time new odometry data is received.
  void computeVelocities(nav_msgs::Odometry odom);

  //! Receive path to follow.
  void receivePath(nav_msgs::Path path);

  void planEnableCallback(const std_msgs::Bool::ConstPtr msg)
  {
    // Enable or disable the path following
    if (plan_enable_ && !msg->data)
    {
      planner_status.data = PLANNER_ABORTED;
      pub_status_.publish(planner_status);
    }
    plan_enable_ = msg->data;
  }
  void costmapCB(const nav_msgs::OccupancyGrid &costmap)
  {
    current_costmap_ = costmap; // 存储下来
  }

  void costmapUpdateCB(const map_msgs::OccupancyGridUpdate &update)
  {
    if (current_costmap_.data.empty())
    {
      ROS_WARN("Received costmap update before initial costmap.");
      return;
    }

    const int width = update.width;
    const int height = update.height;
    const int x = update.x;
    const int y = update.y;
    const int map_width = current_costmap_.info.width;
    const int map_height = current_costmap_.info.height;

    if ((x + width) > map_width || (y + height) > map_height)
    {
      ROS_ERROR("Costmap update out of bounds! Skipping.");
      return;
    }

    for (int row = 0; row < height; ++row)
    {
      int dst_index = (y + row) * map_width + x;
      int src_index = row * width;

      std::copy_n(
          update.data.begin() + src_index,
          width,
          current_costmap_.data.begin() + dst_index);
    }

    // 立即使用更新后的地图检查路径
    // if (plan_enable_ && !path_.poses.empty() && chassis_mode_ == UP_MODE)
    if (plan_enable_ && !path_.poses.empty())
    {
      path_checker_.setCostmap(current_costmap_);

      if (path_checker_.checkPath(idx_))
      {
        ROS_INFO("Path still valid after costmap update at index %d", idx_);
      }
      else
      {
        ROS_WARN("Path became blocked after costmap update at index %d", idx_);
        plan_enable_ = false;
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.0;
        pub_vel_.publish(cmd_vel_);
        planner_status.data = PLANNER_FAILED;
        pub_status_.publish(planner_status);
      }
    }
  }

  void chassisModeCB(const std_msgs::UInt8::ConstPtr &msg)
  {
    chassis_mode_ = msg->data;
  }

  //! Compute transform that transforms a pose into the robot frame (base_link)
  KDL::Frame transformToBaseLink(const geometry_msgs::Pose &pose,
                                 const geometry_msgs::Transform &tf);

  //! Helper founction for computing eucledian distances in the x-y plane.
  template <typename T1, typename T2>
  double distance(T1 pt1, T2 pt2)
  {
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
  }
  template <typename T1, typename T2>
  double distance_2(T1 pt1, T2 pt2)
  {
    return pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2);
  }

  //! Run the controller.
  void run();

private:
  //! Dynamic reconfigure callback.
  void reconfigure(pure_pursuit::PurePursuitConfig &config, uint32_t level);

  // Vehicle parameters
  double L_;
  // Algorithm variables
  // Position tolerace is measured along the x-axis of the robot!
  double ld_, pos_tolerance_;
  // Generic control variables
  double v_max_, v_, w_max_;
  double slow_start_radian;
  // Control variables for Ackermann steering
  // Steering angle is denoted by delta
  nav_msgs::Path path_;
  unsigned idx_;
  bool goal_reached_;
  bool useSteeringAngle_;
  bool plan_enable_ = false;
  uint8_t chassis_mode_ = 1; // 底盘模式 0:上位机模式 1:下位机模式
  geometry_msgs::Twist cmd_vel_;

  // Ros infrastructure
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber sub_odom_, sub_path_, sub_plan_enable_;
  ros::Subscriber sub_local_costmap_, sub_chassis_mode_;
  ros::Subscriber sub_local_costmap_update_; // 新增
  nav_msgs::OccupancyGrid current_costmap_;  // 当前维护的完整地图
  ros::Publisher pub_vel_, pub_status_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped lookahead_;
  string map_frame_id_, robot_frame_id_, lookahead_frame_id_;

  dynamic_reconfigure::Server<pure_pursuit::PurePursuitConfig> reconfigure_server_;
  dynamic_reconfigure::Server<pure_pursuit::PurePursuitConfig>::CallbackType reconfigure_callback_;

  PathChecker path_checker_;
  std_msgs::UInt8 planner_status;
};

PurePursuit::PurePursuit() : ld_(1.0), v_max_(0.1), v_(v_max_), w_max_(1.0), pos_tolerance_(0.1), idx_(0),
                             goal_reached_(true), nh_private_("~"), tf_listener_(tf_buffer_),
                             map_frame_id_("map"), robot_frame_id_("base_link"),
                             lookahead_frame_id_("lookahead")
{
  nh_private_.param<double>("wheelbase", L_, 1.0);
  nh_private_.param<double>("lookahead_distance", ld_, 1.0);
  nh_private_.param<double>("max_linear_velocity", v_max_, 1.0);
  nh_private_.param<double>("max_rotational_velocity", w_max_, 1.0);
  nh_private_.param<double>("slow_start_radian", slow_start_radian, 1.0);
  nh_private_.param<double>("position_tolerance", pos_tolerance_, 0.1);
  nh_private_.param<bool>("use_steering_angle", useSteeringAngle_, false);
  nh_private_.param<string>("map_frame_id", map_frame_id_, "map");
  nh_private_.param<string>("robot_frame_id", robot_frame_id_, "base_link");
  nh_private_.param<string>("lookahead_frame_id", lookahead_frame_id_, "lookahead");

  lookahead_.header.frame_id = robot_frame_id_;
  lookahead_.child_frame_id = lookahead_frame_id_;
  planner_status.data = PLANNER_IDLE;

  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pub_status_ = nh_.advertise<std_msgs::UInt8>("/pure_pursuit/status", 1);

  sub_path_ = nh_.subscribe("path_segment", 2, &PurePursuit::receivePath, this);
  sub_odom_ = nh_.subscribe("odometry", 1, &PurePursuit::computeVelocities, this, ros::TransportHints().tcpNoDelay());
  sub_plan_enable_ = nh_.subscribe("/pure_pursuit/plan_enable", 1, &PurePursuit::planEnableCallback, this);
  sub_local_costmap_ = nh_.subscribe("/move_base/local_costmap/costmap", 1, &PurePursuit::costmapCB, this, ros::TransportHints().tcpNoDelay());

  sub_local_costmap_update_ = nh_.subscribe("/move_base/local_costmap/costmap_updates", 2, &PurePursuit::costmapUpdateCB, this, ros::TransportHints().tcpNoDelay());
  sub_chassis_mode_ = nh_.subscribe("/chassis_mode", 1, &PurePursuit::chassisModeCB, this);

  reconfigure_callback_ = boost::bind(&PurePursuit::reconfigure, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_callback_);
}

void PurePursuit::computeVelocities(nav_msgs::Odometry odom)
{
  std_msgs::UInt8 status_msg;
  if (!plan_enable_)
  {
    // status_msg.data = PLANNER_IDLE;
    // pub_status_.publish(status_msg);
    return;
  }
  // The velocity commands are computed, each time a new Odometry message is received.
  // Odometry is not used directly, but through the tf tree.

  // Get the current robot pose
  geometry_msgs::TransformStamped tf_map_base;
  try
  {
    tf_map_base = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
    // We first compute the new point to track, based on our current pose,
    // path information and lookahead distance.
    for (; idx_ < path_.poses.size() - 1; ++idx_)
    {
      if (distance_2(path_.poses[idx_].pose.position, tf_map_base.transform.translation) > ld_ * ld_)
      {
        // 获取前瞻点 lookahead_
        // Transformed lookahead to base_link frame is lateral error
        KDL::Frame F_bl_ld = transformToBaseLink(path_.poses[idx_].pose, tf_map_base.transform);
        lookahead_.transform.translation.x = F_bl_ld.p.x();
        lookahead_.transform.translation.y = F_bl_ld.p.y();
        lookahead_.transform.translation.z = F_bl_ld.p.z();
        F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x,
                                lookahead_.transform.rotation.y,
                                lookahead_.transform.rotation.z,
                                lookahead_.transform.rotation.w);

        // TODO: See how the above conversion can be done more elegantly
        // using tf2_kdl and tf2_geometry_msgs

        break;
      }
    }

    if (!path_.poses.empty() && idx_ >= path_.poses.size() - 1)
    {
      // We are approaching the goal,
      // which is closer than ld

      // This is the pose of the goal w.r.t. the base_link frame
      KDL::Frame F_bl_end = transformToBaseLink(path_.poses.back().pose, tf_map_base.transform);

      if (fabs(F_bl_end.p.x()) <= pos_tolerance_) // 如果x轴分量上，最后一个路径点大于 pos_tolerance_ (tolerance)
      {
        // We have reached the goal
        goal_reached_ = true;

        // Reset the path
        path_ = nav_msgs::Path();
      }
      else
      {
        // We need to extend the lookahead distance
        // beyond the goal point.

        // Find the intersection between the circle of radius ld
        // 找到半径为ld的圆的交点,圆心在base_link
        // centered at the robot (origin)
        // and the line defined by the last path pose
        // double roll, pitch, yaw;
        // F_bl_end.M.GetRPY(roll, pitch, yaw);
        // double k_end = tan(yaw); // （斜率）Slope of line defined by the last path pose
        // double l_end = F_bl_end.p.y() - k_end * F_bl_end.p.x();
        // double a = 1 + k_end * k_end;
        // double b = 2 * l_end;
        // double c = l_end * l_end - ld_ * ld_;
        // double D = sqrt(b * b - 4 * a * c);
        // double x_ld = (-b + copysign(D, v_)) / (2 * a);
        // double y_ld = k_end * x_ld + l_end;

        // if (F_bl_end.p.x() < 0)
        // {
        //   // If we are moving backwards, we need to flip the sign of the lookahead point
        //   x_ld = -x_ld;
        //   y_ld = -y_ld;
        // }
        // lookahead_.transform.translation.x = x_ld;
        // lookahead_.transform.translation.y = y_ld;
        // lookahead_.transform.translation.z = F_bl_end.p.z();
        F_bl_end.M.GetQuaternion(lookahead_.transform.rotation.x,
                                 lookahead_.transform.rotation.y,
                                 lookahead_.transform.rotation.z,
                                 lookahead_.transform.rotation.w);
        lookahead_.transform.translation.x = F_bl_end.p.x();
        lookahead_.transform.translation.y = F_bl_end.p.y();
        lookahead_.transform.translation.z = F_bl_end.p.z();
      }
    }

    if (!goal_reached_)
    {
      // We are tracking.

      // Compute linear velocity.
      // Right now,this is not very smart :)
      double ld_yaw = atan2(lookahead_.transform.translation.y, lookahead_.transform.translation.x);
      // std::cout << "ld_yaw: " << ld_yaw << std::endl;
      if (abs(ld_yaw) < slow_start_radian)
      {
        v_ = v_max_;
      }
      else if (abs(ld_yaw) <= M_PI / 2)
      {
        v_ = abs(slow_start_radian / ld_yaw) * v_max_;
      }
      else // 前瞻点在后面
      {
        if (abs(ld_yaw) > (M_PI / 2) && normalizeRadian(abs(M_PI - ld_yaw)) <= (slow_start_radian))
        {
          v_ = v_max_;
        }
        else
        {
          v_ = abs(slow_start_radian / normalizeRadian(M_PI - ld_yaw)) * v_max_;
        }
      }

      // Compute the angular velocity.
      // Lateral error is the y-value of the lookahead point (in base_link frame)
      // double yt = lookahead_.transform.translation.y;
      // double ld_2 = ld_ * ld_;
      // cmd_vel_.angular.z = std::min(2 * v_ / ld_2 * yt, w_max_);
      double R = ld_ / (2 * sin(ld_yaw));  // 计算转弯半径
      double steeringAngle = atan(L_ / R); // 计算转向角度
      // std::cout << "steeringAngle: " << steeringAngle << std::endl;
      if (useSteeringAngle_)
      {
        if (steeringAngle >= 0)
          cmd_vel_.angular.z = std::min(steeringAngle, w_max_);
        else
          cmd_vel_.angular.z = std::max(steeringAngle, -w_max_);
      }
      else
      {
        double angular = v_ / R;
        if (angular >= 0)
          cmd_vel_.angular.z = std::min(angular, w_max_);
        else
          cmd_vel_.angular.z = std::max(angular, -w_max_);
      }

      if (!std::isfinite(cmd_vel_.angular.z))
        cmd_vel_.angular.z = 0.0;
      if (!std::isfinite(cmd_vel_.angular.x))
        cmd_vel_.angular.x = 0.0;

      // Compute desired Ackermann steering angle

      // Set linear velocity for tracking.
      if (lookahead_.transform.translation.x < 0.0)
      {
        cmd_vel_.linear.x = -v_;
        // cmd_vel_.angular.z = cmd_vel_.angular.z; // Reverse the angular velocity if moving backwards
      }
      else
        cmd_vel_.linear.x = v_;

      planner_status.data = PLANNER_RUNNING;
      pub_status_.publish(planner_status);
    }
    else // goal_reached_
    {
      // We are at the goal!
      // Stop the vehicle
      // The lookahead target is at our current pose.
      lookahead_.transform = geometry_msgs::Transform();
      lookahead_.transform.rotation.w = 1.0;

      // Stop moving.
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.angular.z = 0.0;
      plan_enable_ = false;

      planner_status.data = PLANNER_FINISHED;
      pub_status_.publish(planner_status);
    }

    // Publish the lookahead target transform.
    lookahead_.header.stamp = ros::Time::now();
    tf_broadcaster_.sendTransform(lookahead_);

    // Publish the velocities
    pub_vel_.publish(cmd_vel_);

    // Publish ackerman steering setpoints
    // pub_acker_.publish(cmd_acker_);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM(ex.what());
  }
}

void PurePursuit::receivePath(nav_msgs::Path new_path)
{
  // When a new path received, the previous one is simply discarded
  // It is up to the planner/motion manager to make sure that the new
  // path is feasible.
  // Callbacks are non-interruptible, so this will
  // not interfere with velocity computation callback.

  if (new_path.header.frame_id == map_frame_id_)
  {
    path_ = new_path;
    idx_ = 0;
    if (new_path.poses.size() > 0)
    {
      goal_reached_ = false;
      plan_enable_ = true;
      path_checker_.setGlobalPath(new_path);
    }
    else
    {
      goal_reached_ = true;
      plan_enable_ = false;
      ROS_WARN_STREAM("Received empty path!");
    }
  }
  else
  {
    ROS_WARN_STREAM("The path must be published in the " << map_frame_id_
                                                         << " frame! Ignoring path in " << new_path.header.frame_id
                                                         << " frame!");
  }
}

KDL::Frame PurePursuit::transformToBaseLink(const geometry_msgs::Pose &pose,
                                            const geometry_msgs::Transform &tf)
{
  // Pose in global (map) frame
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w),
                        KDL::Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z));

  // Robot (base_link) in global (map) frame
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x,
                                                tf.rotation.y,
                                                tf.rotation.z,
                                                tf.rotation.w),
                      KDL::Vector(tf.translation.x,
                                  tf.translation.y,
                                  tf.translation.z));

  // TODO: See how the above conversions can be done more elegantly
  // using tf2_kdl and tf2_geometry_msgs

  return F_map_tf.Inverse() * F_map_pose;
}

void PurePursuit::run()
{
  ros::spin();
}

void PurePursuit::reconfigure(pure_pursuit::PurePursuitConfig &config, uint32_t level)
{
  v_max_ = config.max_linear_velocity;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_pursuit");
  PurePursuit controller;
  controller.run();
  return 0;
}
