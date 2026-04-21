#ifndef ACKERMANN_RECOVERY_HPP
#define ACKERMANN_RECOVERY_HPP
#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Twist.h>
#include <deque>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>

#include <pluginlib/class_loader.h>
namespace ackermann_recovery
{
    
    class AckermannRecovery : public nav_core::RecoveryBehavior
    {
    public:
        AckermannRecovery();

        void initialize(std::string name,
                        tf2_ros::Buffer *tf,
                        costmap_2d::Costmap2DROS *global_costmap,
                        costmap_2d::Costmap2DROS *local_costmap) override;

        void runBehavior() override;

    private:
        ros::NodeHandle nh_;
        ros::Subscriber cmd_vel_sub_;
        ros::Publisher cmd_vel_pub_;
        std::deque<geometry_msgs::Twist> cmd_vel_history_;
        bool initialized_;
        int history_size_;
        double reverse_duration_; // seconds
        double publish_rate_;     // Hz

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    };

} // namespace ackermann_recovery

#endif
