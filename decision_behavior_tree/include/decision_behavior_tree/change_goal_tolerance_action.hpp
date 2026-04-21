#ifndef CHANGE_GOAL_TOLERANCE_ACTION_HPP
#define CHANGE_GOAL_TOLERANCE_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>

class ChangeGoalToleranceAction : public BT::SyncActionNode
{
public:
    explicit ChangeGoalToleranceAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        // 初始化 ROS 节点句柄（如果尚未初始化）
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "bt_change_goal_tolerance");
        }

        // 创建发布者
        pub_ = nh_.advertise<geometry_msgs::Pose2D>("/change_goal_tolerance", 1, true);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("xy", 0.0, "XY tolerance offset"),
            BT::InputPort<double>("yaw", 0.0, "Yaw tolerance offset")};
    }

    BT::NodeStatus tick() override
    {
        double xy, yaw;

        // 获取输入端口值
        if (!getInput<double>("xy", xy))
        {
            throw BT::RuntimeError("missing required input [xy]");
        }
        if (!getInput<double>("yaw", yaw))
        {
            throw BT::RuntimeError("missing required input [yaw]");
        }

        // 构造并发布 Pose2D 消息
        geometry_msgs::Pose2D msg;
        msg.x = xy;
        msg.y = 0.0;
        msg.theta = yaw; // 可选字段，不使用或根据需求设置

        pub_.publish(msg);

        ROS_INFO("Published new goal tolerance: xy=%.2f, yaw=%.2f", xy, yaw);

        return BT::NodeStatus::SUCCESS;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};

#endif // CHANGE_GOAL_TOLERANCE_ACTION_HPP