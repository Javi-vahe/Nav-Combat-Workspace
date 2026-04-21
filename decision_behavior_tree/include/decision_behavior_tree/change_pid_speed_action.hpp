#ifndef CHANGE_PID_SPEED_ACTION_HPP
#define CHANGE_PID_SPEED_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class ChangePIDSpeedAction : public BT::SyncActionNode
{
public:
    explicit ChangePIDSpeedAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        // 初始化 ROS 节点句柄（如果尚未初始化）
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "bt_change_pid_speed");
        }

        // 创建发布者
        pub_ = nh_.advertise<geometry_msgs::Twist>("/pid_speed", 1, true);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("max_speed", 0.0, "Max speed for pid"),
            BT::InputPort<double>("p_value", 0.0, "P value for pid")};
    }

    BT::NodeStatus tick() override
    {
        double max_speed, p_value;

        // 获取输入端口值
        if (!getInput<double>("max_speed", max_speed))
        {
            throw BT::RuntimeError("missing required input [max_speed]");
        }
        if (!getInput<double>("p_value", p_value))
        {
            throw BT::RuntimeError("missing required input [p_value]");
        }

        geometry_msgs::Twist msg;
        msg.linear.x = max_speed;
        msg.linear.z = p_value;

        pub_.publish(msg);

        ROS_INFO("Published new speed for pid_follow_planner: xy=%.2f, yaw=%.2f", max_speed, p_value);

        return BT::NodeStatus::SUCCESS;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};

#endif // CHANGE_PID_SPEED_ACTION_HPP