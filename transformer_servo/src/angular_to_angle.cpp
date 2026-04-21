#include "ros/ros.h"
#include <signal.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"

// 车辆参数（根据车辆的实际情况设置）
const double L = 0.148;                     // 轴距 (m)
const double MIN_SPEED = 0.1;               // 最小速度，防止除以零
const double MAX_STEERING_ANGLE_DEG = 30.0; // 前轮最大转向角（度）

class AngularToAngle
{
public:
    ros::Publisher anglePub_;

public:
    AngularToAngle(ros::NodeHandle &nh) : nh_(nh)
    {
        velSub_ = nh.subscribe("/cmd_vel", 10, &AngularToAngle::vel_callback, this, ros::TransportHints().tcpNoDelay());
        stopSub_ = nh.subscribe("/stop_signal", 10, &AngularToAngle::stopCB, this, ros::TransportHints().tcpNoDelay());
        anglePub_ = nh.advertise<geometry_msgs::Twist>("/pursuitAngle", 10);
    }

private:
    ros::NodeHandle &nh_;
    ros::Subscriber velSub_;
    ros::Subscriber stopSub_;

    void vel_callback(geometry_msgs::Twist::ConstPtr msg)
    {
        double v = msg->linear.x;
        double omega = msg->angular.z;

        if (std::abs(v) >= MIN_SPEED)
            omega = std::atan(L * omega / v);

        double steeringAngle = rad2deg(omega);

        // 限制最大转向角
        if (steeringAngle > MAX_STEERING_ANGLE_DEG)
            steeringAngle = MAX_STEERING_ANGLE_DEG;
        else if (steeringAngle < -MAX_STEERING_ANGLE_DEG)
            steeringAngle = -MAX_STEERING_ANGLE_DEG;

        geometry_msgs::Twist twist_ = *msg;
        twist_.angular.z = steeringAngle;
        anglePub_.publish(twist_);
    }

    void stopCB(const std_msgs::Empty::ConstPtr &msg)
    {
        pubStopCmd();
    }

    void pubStopCmd()
    {
        geometry_msgs::Twist twist_;
        twist_.linear.x = 0.0;
        twist_.angular.z = 0.0;
        anglePub_.publish(twist_);
    }

    inline double deg2rad(double &deg)
    {
        return deg * M_PI / 180.0;
    }

    inline double rad2deg(double &rad)
    {
        return rad * 180.0 / M_PI;
    }
};

std::unique_ptr<AngularToAngle> angularToAngleNodePtr_;
void mySigintHandler(int sig)
{
    // 这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("angularToAngleNode shutting down...");
    geometry_msgs::Twist twist_;
    twist_.linear.x = 0.0;
    twist_.angular.z = 0.0;
    angularToAngleNodePtr_->anglePub_.publish(twist_);
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "angular_to_angle");
    ros::NodeHandle nh;
    AngularToAngle node(nh);
    angularToAngleNodePtr_ = std::make_unique<AngularToAngle>(nh);
    signal(SIGINT, mySigintHandler);
    ros::spin();
    return 0;
}