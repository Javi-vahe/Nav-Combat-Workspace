#ifndef DONOTHING_LOCAL_PLANNER_H_
#define DONOTHING_LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

using namespace std;

namespace donothing_local_planner
{

    class DoNothingLocalPlanner : public nav_core::BaseLocalPlanner
    {
    public:
        DoNothingLocalPlanner();
        DoNothingLocalPlanner(std::string name, tf2_ros::Buffer *tf,
                              costmap_2d::Costmap2DROS *costmap_ros);

        ~DoNothingLocalPlanner();

        void initialize(std::string name, tf2_ros::Buffer *tf,
                        costmap_2d::Costmap2DROS *costmap_ros);
        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);
        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
        bool isGoalReached();
        void isReachedCB(const std_msgs::Bool::ConstPtr &msg);

        bool isPathBlocked();
        bool getCurrentPosition(geometry_msgs::PoseStamped &current_pose);
        int findClosestWaypointIndex(const geometry_msgs::PoseStamped &current_pose);

    private:
        costmap_2d::Costmap2DROS *costmap_ros_;
        tf2_ros::Buffer *tf_;
        bool initialized_;

        ros::Subscriber isReachedSub_;
        bool isReached_ = false;
        bool useExternalGoalCheck_  = false; // Use external goal check from isReached topic
        bool usePathBlockedCheck_ = true; // 是否检查路径是否被阻塞

        double xyGoalTolerance_;
        double yawGoalTolerance_;
        std::string robotFrame_;
        geometry_msgs::PoseStamped currentPose_; // 记录当前机器人坐标
        geometry_msgs::PoseStamped goalPose_;   // 记录路线目标点
        std::vector<geometry_msgs::PoseStamped> globalPlan_; // 记录全局规划路径
        bool allowUnknown_; // 是否允许未知区域（代价为255）
        int ObstacleCost_ = costmap_2d::INSCRIBED_INFLATED_OBSTACLE; // 默认将代价为253的区域被视为障碍物
        int currentGoalIndex_ = 0;
        double robotRadius_ = 0.2; // 机器人半径，单位为米
    };
};

#endif