// src/ackermann_recovery.cpp
#include <ackermann_recovery/ackermann_recovery.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ackermann_recovery::AckermannRecovery, nav_core::RecoveryBehavior)

namespace ackermann_recovery
{

    AckermannRecovery::AckermannRecovery() : initialized_(false), history_size_(20), reverse_duration_(1.0), publish_rate_(10.0) {}

    void AckermannRecovery::initialize(std::string name,
                                       tf2_ros::Buffer *tf,
                                       costmap_2d::Costmap2DROS *global_costmap,
                                       costmap_2d::Costmap2DROS *local_costmap)
    {
        if (initialized_)
        {
            ROS_WARN("AckermannRecovery has already been initialized.");
            return;
        }

        nh_ = ros::NodeHandle("~/" + name);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &AckermannRecovery::cmdVelCallback, this);

        nh_.param("history_size", history_size_, 20);
        nh_.param("reverse_duration", reverse_duration_, 1.0);
        nh_.param("publish_rate", publish_rate_, 10.0);

        initialized_ = true;
        ROS_INFO("AckermannRecovery initialized.");
        ROS_INFO("AckermannRecovery initialized.");
        ROS_INFO("AckermannRecovery initialized.");
        ROS_INFO("AckermannRecovery initialized.");
        ROS_INFO("AckermannRecovery initialized.");
    }

    void AckermannRecovery::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        if (msg->linear.x == 0.0)
            return;
        if (cmd_vel_history_.size() >= static_cast<size_t>(history_size_))
        {
            cmd_vel_history_.pop_front();
        }
        cmd_vel_history_.push_back(*msg);
    }

    void AckermannRecovery::runBehavior()
    {
        if (!initialized_)
        {
            ROS_ERROR("AckermannRecovery not initialized.");
            return;
        }

        if (cmd_vel_history_.empty())
        {
            ROS_WARN("No cmd_vel history available.");
            return;
        }

        double avg_linear_x = 0.0;
        for (const auto &twist : cmd_vel_history_)
        {
            avg_linear_x += twist.linear.x;
        }
        avg_linear_x /= cmd_vel_history_.size();

        geometry_msgs::Twist reverse_cmd;
        reverse_cmd.linear.x = -avg_linear_x;
        reverse_cmd.angular.z = 0.0;

        ros::Rate rate(publish_rate_);
        int count = static_cast<int>(reverse_duration_ * publish_rate_);

        ROS_INFO("AckermannRecovery: publishing reverse velocity for %d cycles.", count);

        for (int i = 0; i < count && ros::ok(); ++i)
        {
            cmd_vel_pub_.publish(reverse_cmd);
            rate.sleep();
        }

        geometry_msgs::Twist stop;
        cmd_vel_pub_.publish(stop);
        ROS_INFO("AckermannRecovery: stop command published.");
    }

} // namespace ackermann_recovery
