#ifndef IS_AIMING_TARGET_ACTION_HPP
#define IS_AIMING_TARGET_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

class IsAimingAction : public BT::StatefulActionNode
{
private:
    ros::NodeHandle nh_;
    ros::Publisher take_off_pub_;
    ros::Subscriber offset_center_sub_;
    ros::Timer spinTimer_;
    bool is_target_lost_;
    bool check_enable_ = false;

public:
    explicit IsAimingAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        // 初始化 ROS 节点句柄（如果尚未初始化）
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "bt_is_aiming_target");
        }
        is_target_lost_ = true;
        take_off_pub_ = nh_.advertise<std_msgs::String>("/take_off", 1);
        offset_center_sub_ = nh_.subscribe<std_msgs::Int32MultiArray>("/offset_center", 1, &IsAimingAction::offsetCenterCB, this);
        spinTimer_ = nh_.createTimer(ros::Duration(1 / 30.0), &IsAimingAction::spinLoop, this);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    void spinLoop(const ros::TimerEvent &)
    {
        if (!check_enable_)
            return;
        ros::spinOnce();
    }

    void offsetCenterCB(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        // if (check_enable_)
        { // ROS_INFO("Waiting for target...");

            if (msg->data[2] == 1)
            {
                is_target_lost_ = false;
                // ROS_INFO("Target is aiming");
            }
            else if (msg->data[2] == 0)
            {
                is_target_lost_ = true;
                // ROS_INFO("Target is lost");
            }
        }
    }

    BT::NodeStatus onStart() override
    {
        // 可选初始化逻辑
        ROS_INFO("Waiting for target...");
        check_enable_ = true;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!is_target_lost_)
        {
            return BT::NodeStatus::RUNNING;
        }
        else
        {
            std::cout << "is not not not aiming" << std::endl;
            check_enable_ = false;
            return BT::NodeStatus::FAILURE;
        }
    }

    void onHalted() override
    {
        check_enable_ = false;
        ROS_INFO("IsAimingAction halted");
        std_msgs::String take_off_msg;
        take_off_msg.data = "3";    // 暂时关闭装甲板识别
        take_off_pub_.publish(take_off_msg);
        // StatefulActionNode::onHalted();
    }
};

#endif // IS_AIMING_TARGET_ACTION_HPP