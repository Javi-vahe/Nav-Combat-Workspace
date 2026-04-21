#ifndef PUB_GOTO_MATERIAL_ACTION_HPP
#define PUB_GOTO_MATERIAL_ACTION_HPP

// #include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

class PubGotoMaterialAction : public BT::SyncActionNode
{
public:
    explicit PubGotoMaterialAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        // 初始化 ROS 节点句柄（如果尚未初始化）
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "bt_pub_goto_material");
        }

        // 创建发布者
        pub_ = nh_.advertise<std_msgs::Bool>("/goto_material", 5, true);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("int", 0, "0: stop, 1: start")};
    }

    BT::NodeStatus tick() override
    {
        int int_value;

        // 获取输入端口值
        if (!getInput<int>("int", int_value))
        {
            throw BT::RuntimeError("missing required input [int]");
        }

        std_msgs::Bool msg;
        msg.data = static_cast<bool>(int_value);
        pub_.publish(msg);
        std::cout << "pub /goto_material msg: " << int(msg.data) << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};

#endif // PUB_GOTO_MATERIAL_ACTION_HPP