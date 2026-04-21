#ifndef PRINT_ACTION_HPP
#define PRINT_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <iostream>

class PrintAction : public BT::SyncActionNode
{
private:
    /* data */
public:
    PrintAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("content", "none", "print content")};
    }

    BT::NodeStatus tick() override
    {
        std::string content;
        if (!getInput<std::string>("content", content))
        {
            std::cout << "[PrintAction]missing required input [content]" << std::endl;
        }
        else
        {
            std::cout << content << std::endl;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

#endif // PRINT_ACTION_HPP