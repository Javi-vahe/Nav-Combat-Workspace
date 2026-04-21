#ifndef GIMBAL_CRUISE_ACTION_HPP
#define GIMBAL_CRUISE_ACTION_HPP

// #include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

class GimbalCruiseAction : public BT::SyncActionNode
{
public:
    explicit GimbalCruiseAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        // 初始化 ROS 节点句柄（如果尚未初始化）
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "bt_pub_gimbal_cruise");
        }

        // 创建发布者
        pub_ = nh_.advertise<std_msgs::UInt8>("/gimbal_cruise", 5);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("cmd", 0, "0: stop, 1: start")};
    }

    BT::NodeStatus tick() override
    {
        int int_value;

        // 获取输入端口值
        if (!getInput<int>("cmd", int_value))
        {
            throw BT::RuntimeError("missing required input [cmd]");
        }

        std_msgs::UInt8 msg;
        msg.data = static_cast<uint8_t>(int_value);
        pub_.publish(msg);
        std::cout << "pub /gimbal_cruise msg: " << int(msg.data) << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};

#endif // GIMBAL_CRUISE_ACTION_HPP