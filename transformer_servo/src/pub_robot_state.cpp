#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pub_robot_state_node");
    ros::NodeHandle nh;

    // 创建TF监听者
    tf::TransformListener tf_listener;

    // 创建Odometry消息发布者
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/state_estimation", 10);

    ros::Rate rate(30.0); // 发布频率30Hz

    while (ros::ok()) {
        tf::StampedTransform transform;
        try {
            // 获取从 map 到 odom 的变换
            tf_listener.lookupTransform("map", "base_link", ros::Time(0), transform);
            
            // 构建Odometry消息
            nav_msgs::Odometry odom;
            odom.header.stamp = transform.stamp_;
            odom.header.frame_id = "map";
            odom.child_frame_id = "base_link";

            // 设置位置
            odom.pose.pose.position.x = transform.getOrigin().x();
            odom.pose.pose.position.y = transform.getOrigin().y();
            odom.pose.pose.position.z = transform.getOrigin().z();

            // 设置方向（四元数）
            tf::Quaternion quat = transform.getRotation();
            odom.pose.pose.orientation.x = quat.x();
            odom.pose.pose.orientation.y = quat.y();
            odom.pose.pose.orientation.z = quat.z();
            odom.pose.pose.orientation.w = quat.w();

            // 可选：速度信息设为0，因为TF中没有提供
            odom.twist.twist.linear.x = 0.0;
            odom.twist.twist.linear.y = 0.0;
            odom.twist.twist.angular.z = 0.0;

            // 发布消息
            odom_pub.publish(odom);
        }
        catch (tf::TransformException &ex) {
            // ROS_WARN("Failed to get transform: %s", ex.what());
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}