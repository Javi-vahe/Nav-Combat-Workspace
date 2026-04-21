#ifndef WAIT_SHOOT_ACTION_HPP
#define WAIT_SHOOT_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace BT;
using namespace std::chrono;

class WaitShootAction : public StatefulActionNode
{
public:
    WaitShootAction(const std::string &name, const NodeConfiguration &config)
        : StatefulActionNode(name, config)
    {
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "bt_wait_shoot");
        }
        pub_temp_take_off_ = nh_.advertise<std_msgs::String>("/take_off", 1);
    }

    static PortsList providedPorts()
    {
        return {InputPort<int>("shoot_count", 10, "shoot count to wait for"),
                InputPort<double>("timeout", 10.0, "timeout in seconds")};
    }

    NodeStatus onStart() override
    {
        auto blackboard = config().blackboard;
        // 获取延时时间
        double seconds = 10.0;
        int shoot_count = 10;
        if (!getInput("timeout", seconds))
        {
            throw BT::RuntimeError("Missing required input 'timeout'");
        }
        if (!getInput("shoot_count", shoot_count))
        {
            throw BT::RuntimeError("Missing required input 'shoot_count'");
        }
        start_ammo_ = blackboard->get<int>("ammo");
        std::cout << "11111 start ammo: " << start_ammo_ << std::endl;
        if (start_ammo_ < shoot_count)
        {
            std::cout << "[WARN] Not enough ammo to shoot" << start_ammo_ << std::endl;
        }

        _start = steady_clock::now();
        _duration = std::chrono::duration<double>(seconds);
        std::cout << "start wait shoot for " << seconds << "s" << std::endl;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        auto blackboard = config().blackboard;
        std::cout << "22222 start ammo: " << start_ammo_ << std::endl;
        auto now = steady_clock::now();
        if (blackboard->get<int>("ammo") == 0)
        {
            std::cout << "[WARN] Ammo is empty, cannot shoot. return SUCCESS" << std::endl;
            pubTempTakeoff();
            return NodeStatus::SUCCESS;
        }
        else if (blackboard->get<int>("ammo") <= start_ammo_ - 10)
        {
            std::cout << "shot " << start_ammo_ << " SUCCESS" << std::endl;
            pubTempTakeoff();
            return NodeStatus::SUCCESS;
        }
        else if (now - _start >= _duration)
        {
            std::cout << "delay success!" << std::endl;
            pubTempTakeoff();
            return NodeStatus::FAILURE;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << name() << "wait shoot halted" << std::endl;
        // StatefulActionNode::onHalted();
    }

    void pubTempTakeoff()
    {
        std_msgs::String take_off_msg;
        take_off_msg.data = "3"; // 暂时关闭装甲板识别
        pub_temp_take_off_.publish(take_off_msg);
    }

private:
    steady_clock::time_point _start;
    std::chrono::duration<double> _duration;
    int start_ammo_ = 10; // 初始弹药数量
    ros::NodeHandle nh_;
    ros::Publisher pub_temp_take_off_;
};

#endif // WAIT_SHOOT_ACTION_HPP