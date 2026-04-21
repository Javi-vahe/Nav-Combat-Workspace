#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <zllc_msgs/Motors.h>

class GimbalTF
{
private:
    ros::NodeHandle &nh_;
    // ros::NodeHandle private_nh_{"~"};
    ros::Subscriber gimbalMotorSub_;

    // 创建 TransformBroadcaster
    tf2_ros::TransformBroadcaster br;

public:
    GimbalTF(ros::NodeHandle &nh) : nh_(nh)
    {
        gimbalMotorSub_ = nh_.subscribe("/gimbal_feedback", 1, &GimbalTF::gimbalMotorCallback, this, ros::TransportHints().tcpNoDelay());
    }
    void gimbalMotorCallback(const zllc_msgs::Motors::ConstPtr &msg)
    {   // 修正中值-2047
        const int motor0_center_position_fix = 2047 - 2047;                            // 电机0云台俯仰中心位置修正值
        const int motor1_center_position_fix = 2047 - 2047;                            // 电机1云台偏航中心位置修正值
        int motor0_position_fixed = msg->motor0.position - motor0_center_position_fix; // 电机0位置修正
        int motor1_position_fixed = msg->motor1.position - motor1_center_position_fix; // 电机1位置修正
        // 处理云台电机数据
        // ROS_INFO("Gimbal Pitch Motor 0 - Position: %d, Fixed Position: %d", msg->motor0.position, motor0_position_fixed);
        // ROS_INFO("Gimbal Pitch Motor 1 - Position: %d, Fixed Position: %d", msg->motor1.position, motor1_position_fixed);

        double pitch = (motor0_position_fixed / 4096.0 * 360.0) - 180.0; // 假设电机位置范围是0-4096，对应角度是180到-180
        double yaw = -((motor1_position_fixed / 4096.0 * 360.0) - 180.0);
        // normalizeAngle(pitch);
        // normalizeAngle(yaw);
        // ROS_INFO("Gimbal Pitch Angle: %.2f, Yaw Angle: %.2f", pitch, yaw);
        pitch = angle2Radians(pitch);
        yaw = angle2Radians(yaw);
        normalizeRadians(pitch);
        normalizeRadians(yaw);
        // 将欧拉角转为四元数
        tf2::Quaternion q;
        q.setRPY(0.0, pitch, yaw); // 注意顺序是 R:roll, P:pitch, Y:yaw

        geometry_msgs::TransformStamped base_link_to_gimbal;
        base_link_to_gimbal.header.stamp = msg->header.stamp;
        base_link_to_gimbal.header.frame_id = "base_link";
        base_link_to_gimbal.child_frame_id = "gimbal_link";
        base_link_to_gimbal.transform.translation.x = 0.07;
        base_link_to_gimbal.transform.translation.y = 0.0;
        base_link_to_gimbal.transform.translation.z = 0.15;

        base_link_to_gimbal.transform.rotation.x = q.x();
        base_link_to_gimbal.transform.rotation.y = q.y();
        base_link_to_gimbal.transform.rotation.z = q.z();
        base_link_to_gimbal.transform.rotation.w = q.w();

        br.sendTransform(base_link_to_gimbal);
    }

    // 将角度转换为弧度
    inline double angle2Radians(double angle)
    {
        return angle * M_PI / 180.0;
    }
    // 将弧度转换为角度
    inline double radians2Angle(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    inline void normalizeAngle(double &angle)
    {
        while (angle > 180.0)
            angle -= 360.0;
        while (angle < -180.0)
            angle += 360.0;
    }
    inline void normalizeRadians(double &radians)
    {
        while (radians > M_PI)
            radians -= 2 * M_PI;
        while (radians < -M_PI)
            radians += 2 * M_PI;
    }
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "gimbal_tf_node");
    ros::NodeHandle nh;
    GimbalTF node(nh);
    ros::spin();

    return 0;
}