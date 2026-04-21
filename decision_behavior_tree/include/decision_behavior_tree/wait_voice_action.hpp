#ifndef WAIT_VOICE_ACTION_HPP
#define WAIT_VOICE_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

using namespace BT;

class WaitVoiceAction : public StatefulActionNode
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber voice_sub_;

    bool waiting = false;

public:
    WaitVoiceAction(const std::string &name, const NodeConfiguration &config)
        : StatefulActionNode(name, config)
    {
        // 初始化 ROS 节点句柄（如果尚未初始化）
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "bt_wait_detect_material");
        }

        voice_sub_ = nh_.subscribe("/voice_switch", 1, &WaitVoiceAction::detectedMaterialCB, this, ros::TransportHints().tcpNoDelay());
    }

    void detectedMaterialCB(const std_msgs::UInt8::ConstPtr msg)
    {
        if (waiting && (msg->data <= 115))
        {
            waiting = false;
        }
    }

    static PortsList providedPorts()
    {
        return {};
    }

    NodeStatus onStart() override
    {
        waiting = true;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if (!waiting)
        {
            std::cout << "wait detect material success!" << std::endl;
            return NodeStatus::SUCCESS;
        }
        // std::cout << "waitting detect material" << std::endl;
        return NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << name() << " halted" << std::endl;
    }
};

#endif // WAIT_VOICE_ACTION_HPP