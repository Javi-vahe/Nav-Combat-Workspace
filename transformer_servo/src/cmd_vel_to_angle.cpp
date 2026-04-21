#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class CmdVelToAngle {
public:
    CmdVelToAngle(ros::NodeHandle &nh) : n_(nh) {
        vel_sub_ = nh.subscribe("/cmd_vel", 10, &CmdVelToAngle::vel_callback, this, ros::TransportHints().tcpNoDelay());
        angle_pub_ = nh.advertise<geometry_msgs::Twist>("/pursuitAngle", 10);
    }

private:
    ros::NodeHandle &n_;
    ros::Subscriber vel_sub_;
    ros::Publisher angle_pub_;

    void vel_callback(geometry_msgs::Twist::ConstPtr msg){
        geometry_msgs::Twist twist_ = *msg;
        twist_.angular.z = msg->angular.z / M_PI * 180.0;
        angle_pub_.publish(twist_);
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_vel_to_angle_node");
    ros::NodeHandle nh;
    CmdVelToAngle node(nh);
    ros::spin();
    return 0;
}