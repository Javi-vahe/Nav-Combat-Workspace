#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <zllc_msgs/Motors.h>

class ControlGimbalToMaterial
{
private:
    ros::NodeHandle &nh_, private_nh_{"~"};
    ros::Publisher gimbalPositionPub_;
    ros::Subscriber materialPositionSub_;
    ros::Subscriber gimbalPositionSub_;
    ros::Subscriber shakeSub_;

    ros::Timer controlTimer_;
    int controlFreq_ = 30; // 控制频率，单位Hz

    // 创建TF监听者
    tf::TransformListener tfListener_;

    bool enableControl_ = false;
    bool enableShake_ = false; // 是否在朝向附近启用摇晃控制
    uint8_t materialNum_ = 0;

    std::string baseFrame_, gimbalFrame_, materialFrame_;
    double visiableAngle_ = 60.0;  // 可见角度，单位度
    double shakeAnglePitch_ = 5.0; // 单方向摇晃角度，单位度
    double shakeAngleYaw_ = 5.0;   // 单方向摇晃角度，单位度

    zllc_msgs::Motors gimbalPosition_;                  // 云台位置反馈 0 pitch, 1 yaw
    const int motor0_center_position_fix = 2047 - 2047; // 电机0云台俯仰中心位置修正值
    const int motor1_center_position_fix = 2047 - 2047; // 电机1云台偏航中心位置修正值
    const int motor_value_per_degree = 4096 / 360;      // 每度对应的电机值

public:
    ControlGimbalToMaterial(ros::NodeHandle &nh) : nh_(nh)
    {
        private_nh_.param<std::string>("base_frame", baseFrame_, "base_link");
        private_nh_.param<std::string>("gimbal_frame", gimbalFrame_, "gimbal_link");
        private_nh_.param<int>("control_freq", controlFreq_, 30);
        private_nh_.param<double>("visiable_angle", visiableAngle_, 60.0);
        private_nh_.param<bool>("enable_shake", enableShake_, false);
        private_nh_.param<double>("shake_angle_pitch", shakeAnglePitch_, 10.0);
        private_nh_.param<double>("shake_angle_yaw", shakeAngleYaw_, 10.0);

        // 初始化发布者
        gimbalPositionPub_ = nh_.advertise<geometry_msgs::Twist>("/gimbal_control/motor_control", 1);

        // 初始化订阅者
        materialPositionSub_ = nh_.subscribe<std_msgs::UInt8>("/gimbal_control/material_position", 1,
                                                              &ControlGimbalToMaterial::materialPositionCallback, this, ros::TransportHints().tcpNoDelay());
        shakeSub_ = nh_.subscribe<std_msgs::Bool>("/gimbal_control/shake", 1,
                                                  &ControlGimbalToMaterial::shakeCallback, this, ros::TransportHints().tcpNoDelay());
        // gimbalPositionSub_ = nh_.subscribe<zllc_msgs::Motors>("gimbal_feedback", 1,
        //                                                       &ControlGimbalToMaterial::gimbalPositionCallback, this, ros::TransportHints().tcpNoDelay());

        // 设置TF监听器
        // tfListener_.setExtrapolationLimit(ros::Duration(0.1)); // 设置时间差限制
        controlTimer_ = nh_.createTimer(ros::Duration(1.0 / controlFreq_), &ControlGimbalToMaterial::controlLoop, this);
    }

    void controlLoop(const ros::TimerEvent &)
    {
        if (!enableControl_)
            return;

        try
        {
            // 判断是否能看见图片
            {
                tf::StampedTransform tf_material_gimbal;
                tfListener_.lookupTransform(materialFrame_, gimbalFrame_, ros::Time(0), tf_material_gimbal);
                double x = tf_material_gimbal.getOrigin().x();
                double y = tf_material_gimbal.getOrigin().y();
                double yaw_angle = atan2(y, x) / M_PI * 180.0;
                if (abs(yaw_angle) > visiableAngle_)
                {
                    // ROS_WARN("Material %d is out of visible range, yaw angle: %.2f", materialNum_, yaw);
                    return;
                }
            }

            tf::StampedTransform tf_base_material;
            tfListener_.lookupTransform(baseFrame_, materialFrame_, ros::Time(0), tf_base_material);
            // 获取四元数
            tf::Quaternion q = tf_base_material.getRotation();

            // 将base_link系下的坐标转到 gimbal 系下
            double x = tf_base_material.getOrigin().x() - 0.07;
            double y = tf_base_material.getOrigin().y();
            double z = tf_base_material.getOrigin().z() - 0.15;
            double pitch, yaw;
            yaw = atan2(y, x);
            pitch = atan2(-z, sqrt(x * x + y * y));
            // ROS_INFO("Transform: x=%.2f, y=%.2f, z=%.2f, pitch=%.2f, yaw=%.2f",
            //          x, y, z, pitch, yaw);

            geometry_msgs::Twist gimbalCommand; // pitch x, yaw y
            // pitch 数值增大 往下
            // yaw 数值增大 往右
            // double now_pitch = motorPitchToRadian(gimbalPosition_.motor0.position);
            // double now_yaw = motorYawToRadian(gimbalPosition_.motor1.position);
            // double new_pitch = now_pitch + pitch;
            // double new_yaw = now_yaw + yaw;
            gimbalCommand.linear.x = radianToMotorPitch(pitch); // pitch
            gimbalCommand.linear.y = radianToMotorYaw(yaw);     // yaw
            if (enableShake_)
                shakeControl(gimbalCommand.linear.x, gimbalCommand.linear.y);
            gimbalPositionPub_.publish(gimbalCommand);
            // std::cout << "pitch pos:" << gimbalPosition_.motor0.position << " angle: " << now_pitch / M_PI * 180
            //           << ", yaw pos: " << gimbalPosition_.motor1.position << " angle: " << now_yaw / M_PI * 180 << std::endl;
            // std::cout << "new pitch pos:" << gimbalCommand.linear.x << " angle: " << pitch / M_PI * 180
            //           << ", new yaw pos: " << gimbalCommand.linear.y << " angle: " << yaw / M_PI * 180 << std::endl;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Failed to get transform: %s", ex.what());
        }
    }

    void shakeControl(double &pitch, double &yaw)
    {
        static int last_pitch_increment = motor_value_per_degree;
        static int last_yaw_increment = -motor_value_per_degree;
        static int pitch_shake_direction = 1; // 1: 正向，-1: 反向
        static int yaw_shake_direction = 1;   // 1: 正向，-1: 反向
        int max_pitch_increment = static_cast<int>(shakeAnglePitch_ * motor_value_per_degree);
        int max_yaw_increment = static_cast<int>(shakeAngleYaw_ * motor_value_per_degree);
        static int interval = 0;
        static int pitch_increment_step = 200;
        static int yaw_increment_step = 100 * (shakeAngleYaw_ / shakeAnglePitch_); // 保持比例关系
        pitch += last_pitch_increment;
        yaw += last_yaw_increment;

        if (interval++ >= 5)
        {
            interval = 0;
            // pitch_increment_step = abs(100 * sin(pitch));
            // last_pitch_increment += pitch_increment_step * pitch_shake_direction;
            // yaw_increment_step = abs(100 * cos(yaw));
            // last_yaw_increment += yaw_increment_step * yaw_shake_direction;
            last_pitch_increment += pitch_increment_step * pitch_shake_direction;
            last_yaw_increment += yaw_increment_step * yaw_shake_direction;

            if (last_pitch_increment >= max_pitch_increment)
            {
                last_pitch_increment = max_pitch_increment;
                pitch_shake_direction = -1; // 反向
            }
            else if (last_pitch_increment <= -max_pitch_increment)
            {
                last_pitch_increment = -max_pitch_increment;
                pitch_shake_direction = 1; // 正向
            }

            if (last_yaw_increment >= max_yaw_increment)
            {
                last_yaw_increment = max_yaw_increment;
                yaw_shake_direction = -1; // 反向
            }
            else if (last_yaw_increment <= -max_yaw_increment)
            {
                last_yaw_increment = -max_yaw_increment;
                yaw_shake_direction = 1; // 正向
            }
            // std::cout << "Shaking: pitch increment = " << last_pitch_increment
            //           << ", yaw increment = " << last_yaw_increment << std::endl;
        }
    }

    double motorPitchToRadian(uint16_t position)
    {
        double result = ((position - motor0_center_position_fix) / 4096.0 * 360.0 - 180) / 180.0 * M_PI;
        result = normalizeRadian(result); // 确保结果在 -π 到 π 之间
        return result;
    }
    double motorYawToRadian(uint16_t position)
    {
        double result = -(((position - motor1_center_position_fix) / 4096.0 * 360.0) + 180) / 180.0 * M_PI;
        result = normalizeRadian(result); // 确保结果在 -π 到 π 之间
        return result;
    }

    uint16_t radianToMotorPitch(double radian)
    {
        return static_cast<uint16_t>((radian / M_PI * 180.0 + 180) / 360.0 * 4096.0) + motor0_center_position_fix;
    }
    uint16_t radianToMotorYaw(double radian)
    {
        return static_cast<uint16_t>(-((radian / M_PI * 180.0 - 180) / 360.0 * 4096.0)) + motor1_center_position_fix;
    }

    void gimbalPositionCallback(const zllc_msgs::Motors::ConstPtr &msg)
    {
        gimbalPosition_ = *msg;
    }

    void shakeCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        enableShake_ = msg->data;
    }

    void materialPositionCallback(const std_msgs::UInt8::ConstPtr &msg)
    {
        if (!checkMaterialValid(msg->data))
        {
            enableControl_ = false;
            return;
        }
        enableControl_ = true;
        materialNum_ = msg->data;
        if (materialNum_ < 100)
            materialFrame_ = "self_material_" + std::to_string(static_cast<int>(materialNum_));
        else
            materialFrame_ = "enemy_material_" + std::to_string(static_cast<int>(materialNum_ - 100));
    }

    // 检查材料编号是否在有效范围内
    bool checkMaterialValid(uint8_t num)
    {
        return (num > 0 && num <= 20) || (num > 100 && num <= 120);
    }

    inline double normalizeRadian(double radian)
    {
        while (radian > M_PI)
            radian -= 2 * M_PI;
        while (radian < -M_PI)
            radian += 2 * M_PI;
        return radian;
    }
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "control_gimbal_to_material_node");
    ros::NodeHandle nh;
    ControlGimbalToMaterial node(nh);
    ros::spin();
    return 0;
}
