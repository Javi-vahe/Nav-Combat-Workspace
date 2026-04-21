#ifndef IF_DETECT_TARGET_ACTION_HPP
#define IF_DETECT_TARGET_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

class IfDetectTargetAction : public BT::SyncActionNode
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber offset_center_sub_;

    bool is_detect_target_ = false;

public:
    explicit IfDetectTargetAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "bt_check_if_detect");
        }

        offset_center_sub_ = nh_.subscribe<std_msgs::Int32MultiArray>("/offset_center", 1, &IfDetectTargetAction::offsetCenterCB, this);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        ros::spinOnce();
        if (is_detect_target_)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    void offsetCenterCB(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        if (msg->data[2] == 1)
            is_detect_target_ = true;
        else if (msg->data[2] == 0)
            is_detect_target_ = false;
    }
};

#endif // IF_DETECT_TARGET_ACTION_HPP