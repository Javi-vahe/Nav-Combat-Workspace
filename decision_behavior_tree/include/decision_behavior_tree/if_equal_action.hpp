#ifndef IF_EUQAL_ACTION_HPP
#define IF_EUQAL_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>

class IfEqualAction : public BT::SyncActionNode
{
private:
public:
    explicit IfEqualAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("a", 0.0, "a value"),
                BT::InputPort<double>("b", 0.0, "b value")};
    }

    BT::NodeStatus tick() override
    {
        double a, b;
        // 获取输入端口值
        if (!getInput<double>("a", a))
        {
            throw BT::RuntimeError("missing required input [a]");
        }
        if (!getInput<double>("b", a))
        {
            throw BT::RuntimeError("missing required input [b]");
        }
        // 检查 a 和 b 是否相等
        if (a == b)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
};

#endif // IF_EUQAL_ACTION_HPP