#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class Bond
{
public:
    ros::Publisher voice_switch_pub_;
    ros::Publisher material_position_pub_;
    ros::Publisher take_off_pub_;   // 切换物资识别模型
    ros::Publisher check_area_pub_; // 发布要检查的区域
    ros::Publisher remove_material_pub_;
    ros::Publisher gimbal_cruise_pub_;
    ros::Publisher hold_material_pub_;

private:
    ros::NodeHandle &nh_;
    ros::NodeHandle private_nh_{"~"};

    ros::Subscriber closest_material_sub_;
    ros::Subscriber detected_material_sub_; // 订阅视觉识别到的物资类别
    ros::Subscriber check_area_result_sub_; // 订阅区域检查结果
    ros::Subscriber go_to_material_sub_;    // 订阅是否需要去物资点位的指令

    ros::Timer control_timer_;

    uint8_t closest_material_id_ = 0; // 最近的物资编号
    uint8_t target_material_id_ = 0;

    bool goToMaterial_ = false;      // 是否需要去物资点位
    bool detected_material_ = false; // 是否接收到物资识别结果
    bool hold_material = false;

public:
    Bond(ros::NodeHandle &nh) : nh_(nh)
    {
        voice_switch_pub_ = nh_.advertise<std_msgs::UInt8>("/voice_switch", 1);
        material_position_pub_ = nh_.advertise<std_msgs::UInt8>("/gimbal_control/material_position", 1);
        take_off_pub_ = nh_.advertise<std_msgs::String>("/take_off", 1); // 切换物资识别模型
        check_area_pub_ = nh_.advertise<std_msgs::UInt8>("/area_checker/check_area", 1);
        remove_material_pub_ = nh_.advertise<std_msgs::UInt8>("remove_material", 1);
        gimbal_cruise_pub_ = nh_.advertise<std_msgs::UInt8>("/gimbal_cruise", 1);
        hold_material_pub_ = nh_.advertise<std_msgs::UInt8>("/hold_material", 1);

        closest_material_sub_ = nh_.subscribe("/closest_material", 10, &Bond::closestMaterialCB, this, ros::TransportHints().tcpNoDelay());
        detected_material_sub_ = nh_.subscribe("/detected_material", 10, &Bond::detectedMaterialCB, this, ros::TransportHints().tcpNoDelay());
        check_area_result_sub_ = nh_.subscribe("/area_checker/check_area_result", 10, &Bond::checkAreaResultCB, this, ros::TransportHints().tcpNoDelay());
        go_to_material_sub_ = nh_.subscribe("/goto_material", 10, &Bond::goToMaterialCB, this, ros::TransportHints().tcpNoDelay());

        control_timer_ = nh_.createTimer(ros::Duration(1.0 / 10.0), &Bond::controlTimer, this);
    }

    void controlTimer(const ros::TimerEvent &)
    {
        static tf::TransformListener tf_listener;
        if (hold_material)
        {
            tf::StampedTransform tf_map_base;
            try
            {
                // 获取从 map 到 odom 的变换
                tf_listener.lookupTransform("map", "base_center", ros::Time(0), tf_map_base);

                double xy_err_2 = pow(tf_map_base.getOrigin().x(), 2) + pow(tf_map_base.getOrigin().y(), 2);
                // double roll, pitch, yaw;
                // tf::Quaternion quat = tf_map_base.getRotation();
                // tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w())).getRPY(roll, pitch, yaw);

                if (xy_err_2 <= 0.3 * 0.3) // 小于 tolerance 再删除物资
                {
                    std_msgs::UInt8 remove_material_msg;
                    remove_material_msg.data = target_material_id_; // 发布最近物资编号
                    remove_material_pub_.publish(remove_material_msg);
                    hold_material = false;
                    std_msgs::UInt8 hold_material_msg;
                    hold_material_msg.data = 0;
                    hold_material_pub_.publish(hold_material_msg);
                }
            }
            catch (tf::TransformException &ex)
            {
                // ROS_WARN("Failed to get transform: %s", ex.what());
            }
        }
    }

    // 调用该函数启用流程
    void startRun()
    {
        std::cout << "startRun()" << std::endl;
        detected_material_ = false;

        // 切换语音播报地点
        std_msgs::UInt8 voice_switch_msg;
        if (target_material_id_ < 100) // 己方
            voice_switch_msg.data = target_material_id_ + 200;
        else // 敌方
            voice_switch_msg.data = target_material_id_ - 100 + 220;
        voice_switch_pub_.publish(voice_switch_msg);

        // 发布要检查的区域
        std_msgs::UInt8 check_area_msg;
        check_area_msg.data = target_material_id_;
        check_area_pub_.publish(check_area_msg);
    }

    void stopRun()
    {
        std::cout << "stopRun()" << std::endl;
        detected_material_ = false; // 重置检测状态

        std::cout << "已停止去物资点位的需求。" << std::endl;

        std_msgs::UInt8 material_position_msg;
        material_position_msg.data = 0; // 发布0表示不需要朝向物资点位
        material_position_pub_.publish(material_position_msg);

        std_msgs::String take_off_msg;
        take_off_msg.data = "2"; // 切换回装甲板识别模型
        take_off_pub_.publish(take_off_msg);

        std_msgs::UInt8 check_area_msg;
        check_area_msg.data = 0; // 发布0表示不需要检查区域
        check_area_pub_.publish(check_area_msg);

        std_msgs::UInt8 gimbal_cruise_msg;
        gimbal_cruise_msg.data = 1; // 恢复巡航模式
        gimbal_cruise_pub_.publish(gimbal_cruise_msg);
    }

    void goToMaterialCB(const std_msgs::Bool::ConstPtr msg)
    {
        // if (goToMaterial_ && msg->data)
        // return; // 如果已经为true，直接返回
        goToMaterial_ = msg->data;

        std::cout << "接收到去物资点位的请求：" << (goToMaterial_ ? "开启" : "关闭") << std::endl;

        if (goToMaterial_ && closest_material_id_ == 0)
        {
            ROS_WARN("最近的物资编号为0，无法去物资点位。");
            goToMaterial_ = false; // 重置状态
        }
        else if (goToMaterial_)
        {
            target_material_id_ = closest_material_id_;
            std::cout << "最近的物资编号：" << static_cast<int>(target_material_id_) << std::endl;
            startRun();
        }
        else
        {
            stopRun();
            std::cout << "已关闭去物资点位的需求。" << std::endl;
        }
    }

    /**
     * @brief 接收最近的物资编号的回调函数。
     * @note 编号在发布时已经进行处理，1-20为己方；101-120为敌方
     */
    void closestMaterialCB(const std_msgs::UInt8::ConstPtr msg)
    {
        if (msg->data == closest_material_id_)
            return; // 如果是重复的最近物资编号，直接返回
        else if (msg->data == 0)
            return; // 无效物资编号，直接返回
        closest_material_id_ = msg->data;

        std::cout << "接收到最近的物资编号：" << static_cast<int>(closest_material_id_) << std::endl;
    }

    void detectedMaterialCB(const std_msgs::UInt8::ConstPtr msg)
    {
        static int detected_count = 0;             // 记录检测到物资的次数
        static uint8_t last_detected_material = 0; // 上次检测到的物资编号
        if (msg->data == last_detected_material)
            detected_count++;
        else
            detected_count = 0;             // 重置计数
        last_detected_material = msg->data; // 更新上次检测到的物资编号

        if (!detected_material_ && detected_count >= 0)
        {
            detected_material_ = true; // 标记已检测到物资
            std::cout << "检测到物资类别：" << static_cast<int>(msg->data) << std::endl;

            std_msgs::UInt8 voice_switch_msg;
            voice_switch_msg.data = msg->data; // 直接使用物资编号作为语音播报内容
            voice_switch_pub_.publish(voice_switch_msg);

            if (target_material_id_ < 100) // 己方物资不需要检测回到基地
            {
                std_msgs::UInt8 remove_material_msg;
                remove_material_msg.data = target_material_id_; // 发布最近物资编号
                remove_material_pub_.publish(remove_material_msg);
            }
            else
            {
                hold_material = true;
                std_msgs::UInt8 hold_material_msg;
                hold_material_msg.data = 1;
                hold_material_pub_.publish(hold_material_msg);
            }

            sleep(0.1); // 等待语音播报传输到下位机

            stopRun();
        }
    }

    void checkAreaResultCB(const std_msgs::Bool::ConstPtr msg)
    {
        static ros::Time enter_area_time; // 进入物资识别区的时间
        static bool in_area = false;      // 是否在物资识别区
        if (!msg->data || !goToMaterial_)
        {
            in_area = false;
            return; // 如果不在感兴趣的区域，直接返回
        }
        if (!in_area) // 刚进入物资区，记录时间
        {
            enter_area_time = ros::Time::now();
            in_area = true;
        }
        std_msgs::UInt8 gimbal_cruise_msg;
        gimbal_cruise_msg.data = 0; // 关闭巡航
        gimbal_cruise_pub_.publish(gimbal_cruise_msg);

        // 进到物资识别区时，发布物资位置控制云台朝向
        std_msgs::UInt8 material_position_msg;
        material_position_msg.data = target_material_id_;
        material_position_pub_.publish(material_position_msg);

        if ((ros::Time::now() - enter_area_time).toSec() > 1.0) // 等待云台旋转
        {
            std_msgs::String take_off_msg;
            take_off_msg.data = "1"; // 切换物资识别模型的指令
            take_off_pub_.publish(take_off_msg);
        }
    }
};

std::unique_ptr<Bond> nodePtr_;
void mySigintHandler(int sig)
{
    // 这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("bond shutting down...");
    std_msgs::UInt8 material_position_msg;
    material_position_msg.data = 0; // 发布0表示不需要朝向物资点位
    nodePtr_->material_position_pub_.publish(material_position_msg);

    std_msgs::String take_off_msg;
    take_off_msg.data = "2"; // 切换回装甲板识别模型
    nodePtr_->take_off_pub_.publish(take_off_msg);

    std_msgs::UInt8 check_area_msg;
    check_area_msg.data = 0; // 发布0表示不需要检查区域
    nodePtr_->check_area_pub_.publish(check_area_msg);

    std_msgs::UInt8 gimbal_cruise_msg;
    gimbal_cruise_msg.data = 0; // 关闭巡航
    nodePtr_->gimbal_cruise_pub_.publish(gimbal_cruise_msg);
    ros::shutdown();
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "bond_node");
    ros::NodeHandle nh;
    Bond node(nh);
    nodePtr_ = std::make_unique<Bond>(nh);
    signal(SIGINT, mySigintHandler);
    ros::spin();
    return 0;
}
