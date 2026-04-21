#ifndef COMPARE_BIGGER_INT_ACTION_HPP
#define COMPARE_BIGGER_INT_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>

class CompareBiggerIntAction : public BT::SyncActionNode
{
private:
public:
    explicit CompareBiggerIntAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("a", 0, "a value"),
                BT::InputPort<int>("b", 0, "b value"),
                BT::InputPort<std::string>("cout", "", "cout content")};
    }

    BT::NodeStatus tick() override
    {
        int a, b;
        std::string cout_content;
        // 获取输入端口值
        if (!getInput<int>("a", a))
        {
            throw BT::RuntimeError("missing required input [a]");
        }
        if (!getInput<int>("b", b))
        {
            throw BT::RuntimeError("missing required input [b]");
        }
        if (getInput<std::string>("cout", cout_content))
        {
            std::cout << cout_content << "\t";
        }

        if (a > b)
        {
            std::cout << a << ">" << b << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout << a << "<=" << b << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }
};

#endif // COMPARE_BIGGER_INT_ACTION_HPP