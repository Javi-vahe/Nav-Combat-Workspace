#ifndef DELAY_SECONDS_ACTION_HPP
#define DELAY_SECONDS_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <chrono>
#include <iostream>

using namespace BT;
using namespace std::chrono;

class DelayAction : public StatefulActionNode
{
public:
    DelayAction(const std::string &name, const NodeConfiguration &config)
        : StatefulActionNode(name, config)
    {
    }

    static PortsList providedPorts()
    {
        return {InputPort<double>("seconds", 1.0, "Delay duration in seconds")};
    }

    NodeStatus onStart() override
    {
        // 获取延时时间
        double seconds = 1.0;
        if (!getInput("seconds", seconds))
        {
            throw BT::RuntimeError("Missing required input 'seconds'");
        }

        _start = steady_clock::now();
        _duration = std::chrono::duration<double>(seconds);
        std::cout << "start delay " << seconds << std::endl;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        auto now = steady_clock::now();
        if (now - _start >= _duration)
        {
            std::cout << "delay success!" << std::endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << name() << " halted" << std::endl;
        // StatefulActionNode::onHalted();
    }

private:
    steady_clock::time_point _start;
    std::chrono::duration<double> _duration;
};

#endif // DELAY_SECONDS_ACTION_HPP