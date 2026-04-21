#include "donothing_local_planner/donothing_local_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(donothing_local_planner::DoNothingLocalPlanner, nav_core::BaseLocalPlanner)

namespace donothing_local_planner
{

    DoNothingLocalPlanner::DoNothingLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

    DoNothingLocalPlanner::DoNothingLocalPlanner(std::string name, tf2_ros::Buffer *tf,
                                                 costmap_2d::Costmap2DROS *costmap_ros)
        : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        // std::cout << "DoNothingLocalPlanner constructor called" << std::endl;
        initialize(name, tf, costmap_ros);
    }

    DoNothingLocalPlanner::~DoNothingLocalPlanner() {}

    // Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
    void DoNothingLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                           costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            ros::NodeHandle nh("~/" + name);

            nh.param("use_path_blocked_check", usePathBlockedCheck_, true); // 默认检查路径是否被阻塞
            std::cout << "use_path_blocked_check: " << (usePathBlockedCheck_ ? "true" : "false") << std::endl;
            if (usePathBlockedCheck_)
            {
                nh.param("allow_unknown", allowUnknown_, false);                                        // 默认不允许未知区域
                nh.param("robot_radius", robotRadius_, 0.2);                                            // 默认机器人半径为0.2米
                nh.param("obstacle_cost", ObstacleCost_, int(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)); // 默认将代价为253的区域视为障碍物
                std::cout << "allow_unknown: " << (allowUnknown_ ? "true" : "false") << std::endl;
                std::cout << "robot_radius: " << robotRadius_ << std::endl;
                std::cout << "obstacle_cost: " << ObstacleCost_ << std::endl;
            }

            nh.param("use_external_goal_check", useExternalGoalCheck_, false);
            if (useExternalGoalCheck_)
            {
                std::cout << "Using external goal check from isReached topic." << std::endl;
                std::string isReachedTopic;
                nh.param("is_reach_topic", isReachedTopic, std::string("is_reached"));
                std::cout << "is_reach_topic: " << isReachedTopic << std::endl;
                isReachedSub_ = nh.subscribe(isReachedTopic, 1, &DoNothingLocalPlanner::isReachedCB, this, ros::TransportHints().tcpNoDelay());
            }
            else
            {
                std::cout << "Using internal goal check." << std::endl;
                nh.param("xy_goal_tolerance", xyGoalTolerance_, 0.1);
                nh.param("yaw_goal_tolerance", yawGoalTolerance_, 0.1);
                nh.param("robot_frame", robotFrame_, std::string("base_link"));
                std::cout << "xy_goal_tolerance: " << xyGoalTolerance_ << std::endl;
                std::cout << "yaw_goal_tolerance: " << yawGoalTolerance_ << std::endl;
                std::cout << "robot_frame: " << robotFrame_ << std::endl;
            }
            initialized_ = true;
        }
    }

    bool DoNothingLocalPlanner::setPlan(
        const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        // std::cout << "DoNothingLocalPlanner setPlan called" << std::endl;
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        if (!useExternalGoalCheck_) // 内部判断
        {
            globalPlan_ = orig_global_plan;
            goalPose_ = globalPlan_.back(); // 最后一个点为目标
        }
        else // 外部判断
        {
            isReached_ = false;
        }
        return true;
    }

    bool DoNothingLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        // std::cout << "DoNothingLocalPlanner computeVelocityCommands called" << std::endl;
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        if (usePathBlockedCheck_ && isPathBlocked())
        {
            ROS_WARN("Path is blocked. Stopping robot and requesting replan.");
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            return false; // 返回 false 触发 move_base 的 replan
                          // move_base 的 max_planning_retries 参数为允许失败次数
                          // 当返回false的次数大于等于这个参数时会触发move_base的重规划
        }

        return true;
    }

    // 检查路径中是否有障碍物
    bool DoNothingLocalPlanner::isPathBlocked()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        if (globalPlan_.empty())
        {
            ROS_ERROR("Global plan is empty, cannot check for path blockage.");
            return false; // 没有路径，无法判断
        }

        if (!getCurrentPosition(currentPose_))
        {
            ROS_WARN("Could not get current position for path checking.");
            return false; // 不确定位置时暂不触发 replan
        }

        // 找到最近的路径点索引
        currentGoalIndex_ = findClosestWaypointIndex(currentPose_);

        costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
        if (!costmap)
        {
            ROS_WARN("Costmap is not available.");
            return false;
        }

        unsigned int mx, my;
        double origin_x = costmap->getOriginX(), origin_y = costmap->getOriginY();
        double resolution = costmap->getResolution();

        for (unsigned int i = currentGoalIndex_; i < globalPlan_.size(); ++i)
        {
            const auto &pose = globalPlan_[i];
            double wx = pose.pose.position.x;
            double wy = pose.pose.position.y;

            // 将世界坐标转换为地图坐标
            if (!costmap->worldToMap(wx, wy, mx, my))
            {
                ROS_WARN("Point (%f, %f) out of local costmap bounds.", wx, wy);
                return false;
            }

            unsigned char cost = costmap->getCost(mx, my);

            // 判断是否是障碍物或高代价区域
            if (cost == costmap_2d::NO_INFORMATION)
            {
                if (!allowUnknown_)
                {
                    ROS_WARN("Path crosses unknown area at (%f, %f), and allow_unknown is false.", wx, wy);
                    return true;
                }
            }
            else if (cost >= ObstacleCost_)
            {
                ROS_WARN("Obstacle detected on path at (%f, %f), cost: %d", wx, wy, cost);
                return true;
            }
        }
        return false; // 路径畅通
    }

    int DoNothingLocalPlanner::findClosestWaypointIndex(const geometry_msgs::PoseStamped &current_pose)
    {
        double min_dist = std::numeric_limits<double>::infinity();
        double last_dist = std::numeric_limits<double>::infinity();
        int closest_index = currentGoalIndex_;

        for (int i = closest_index; i < globalPlan_.size(); ++i)
        {
            const auto &pt = globalPlan_[i];
            double dx = pt.pose.position.x - current_pose.pose.position.x;
            double dy = pt.pose.position.y - current_pose.pose.position.y;
            double dist = dx * dx + dy * dy;

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_index = i;
            }
            else
            {
                // 如果当前点的距离比上一个点更远，且小于机器人半径，说明已经超过了最近点
                if (dist > last_dist && dist <= robotRadius_)
                {
                    break; // 退出循环
                    // 因为会对 closest_index 及之后的所有路径点进行检查
                    // 所以即使没有考虑到所有情况而导致寻找的路径点不是最近点也无所谓。
                }
                last_dist = dist; // 更新上一个点的距离
            }
        }

        return closest_index;
    }

    bool DoNothingLocalPlanner::isGoalReached()
    {
        // std::cout << "DoNothingLocalPlanner isGoalReached called" << std::endl;
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        if (!useExternalGoalCheck_) // donothing_local_planner 自身判断是否到达目标点
        {
            // 获取当前机器人坐标
            if (!getCurrentPosition(currentPose_))
            {
                return false;
            }

            // 提取当前坐标朝向
            double current_yaw = tf2::getYaw(currentPose_.pose.orientation);
            // 获取目标点朝向
            double goal_yaw = tf2::getYaw(goalPose_.pose.orientation);

            // 判断位置误差是否在容差范围内
            double dx = currentPose_.pose.position.x - goalPose_.pose.position.x;
            double dy = currentPose_.pose.position.y - goalPose_.pose.position.y;
            double dist = sqrt(dx * dx + dy * dy);
            double dyaw = fabs(current_yaw - goal_yaw);

            if (dist <= xyGoalTolerance_ && dyaw <= yawGoalTolerance_)
            {
                ROS_INFO("Goal has been reached!");
                return true;
            }
            else
            {
                return false;
            }
        }
        else // 使用外部条件判断是否到达目标
        {
            if (isReached_)
            {
                ROS_INFO("Goal has been reached!");
                isReached_ = false; // Reset isReached_ for the next goal
                return true;
            }
            else
            {
                // ROS_INFO("Goal has not been reached yet.");
                return false;
            }
        }
    }

    // 监听tf获取当前机器人位置
    bool DoNothingLocalPlanner::getCurrentPosition(geometry_msgs::PoseStamped &current_pose)
    {
        try
        {
            geometry_msgs::TransformStamped transform = tf_->lookupTransform(
                costmap_ros_->getGlobalFrameID(), robotFrame_,
                ros::Time(0), ros::Duration(0.5));

            current_pose.header.stamp = transform.header.stamp;
            current_pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            current_pose.pose.position.x = transform.transform.translation.x;
            current_pose.pose.position.y = transform.transform.translation.y;
            current_pose.pose.orientation = transform.transform.rotation;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failed to get current position: %s", ex.what());
            return false;
        }

        return true;
    }

    void DoNothingLocalPlanner::isReachedCB(const std_msgs::Bool::ConstPtr &msg)
    {
        // std::cout << "DoNothingLocalPlanner isReachedCB called" << std::endl;
        if (isReached_ == false)
        {
            isReached_ = msg->data;
        }
    }
}