#ifndef SUBSCRIBER_CALLBACKS_HPP
#define SUBSCRIBER_CALLBACKS_HPP

#include <behaviortree_cpp_v3/blackboard.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg,
                  const std::shared_ptr<BT::Blackboard> &blackboard)
{
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.orientation.z;
    double w = msg->pose.orientation.w;

    blackboard->set("goal_x", x);
    blackboard->set("goal_y", y);
    blackboard->set("goal_orientation_z", z);
    blackboard->set("goal_orientation_w", w);

    ROS_INFO("Received new goal and updated blackboard.");
}

void cloestMaterialCB(const std_msgs::UInt8::ConstPtr &msg,
                      const std::shared_ptr<BT::Blackboard> &blackboard)
{
    static int goal_id = 0;
    if (goal_id == int(msg->data))
        return;
    goal_id = msg->data;
    blackboard->set("material_goal_id", goal_id);
    ROS_INFO("Received closest material goal ID: %d", goal_id);
}

void hpAndHitmsgCB(const std_msgs::UInt8MultiArray::ConstPtr &msg,
                   const std::shared_ptr<BT::Blackboard> &blackboard)
{
    auto received_time = ros::Time::now();
    static int last_being_hit = 16;

    int enemy_hp = msg->data[0];
    int self_hp = msg->data[1];
    int being_hit = msg->data[2];
    int shooting_count = msg->data[3];
    int ammo = msg->data[4];

    blackboard->set("enemy_hp", enemy_hp);             // 对方血量
    blackboard->set("self_hp", self_hp);               // 自身血量
    blackboard->set("being_hit", being_hit);           // 是否被敌方击打 0x01左方,0x02前方，0x03右方，0x04后方,16未被击打
    blackboard->set("shooting_count", shooting_count); // 已射击次数
    blackboard->set("ammo", ammo);                     // 剩余弹药数量

    if (being_hit != 16)
    {
        blackboard->set("last_being_hit", last_being_hit);
        blackboard->set("last_being_hit_time", received_time.toSec()); // 更新上一次被击打的时间
        last_being_hit = being_hit;                                    // 更新上一次被击打的装甲板
    }

    // ROS_INFO("Updated HP and hit information on the blackboard.");
    // ROS_INFO("Enemy HP: %d, Self HP: %d, Being Hit: %d, Shooting Count: %d, Ammo: %d, Last Being Hit: %d, Last Being Hit Time: %f",
    //  enemy_hp, self_hp, being_hit, shooting_count, ammo, last_being_hit, blackboard->get<double>("last_being_hit_time"));
}

void holdMaterialCB(const std_msgs::UInt8::ConstPtr &msg,
                    const std::shared_ptr<BT::Blackboard> &blackboard)
{
    blackboard->set("hold_material", static_cast<int>(msg->data));
}

void chassisModeCB(const std_msgs::UInt8::ConstPtr &msg,
                   const std::shared_ptr<BT::Blackboard> &blackboard)
{
    static int same_mode_count = 0;
    static int last_mode = 1;
    if (last_mode == static_cast<int>(msg->data))
    {
        same_mode_count++;
        if (same_mode_count > 100)
        {
            // ROS_INFO("Chassis mode is stable: %d", static_cast<int>(msg->data));
            blackboard->set("chassis_mode", static_cast<int>(msg->data));
        }
    }
    else
    {
        same_mode_count = 0; // 重置计数
    }
    last_mode = static_cast<int>(msg->data);
    // ROS_INFO("Chassis mode updated to: %d", static_cast<int>(msg->data));
}

void selfMaterialNumberCB(const std_msgs::UInt8MultiArray::ConstPtr &msg,
                          const std::shared_ptr<BT::Blackboard> &blackboard)
{
    // 消息的第一个元素是正确数量
    if (msg->data.size() >= 1)
    {
        int correct_count = msg->data.size();
        blackboard->set("correct_material", correct_count);
        ROS_INFO("Correct det material count:%d", correct_count);
    }
}

#endif // SUBSCRIBER_CALLBACKS_HPP