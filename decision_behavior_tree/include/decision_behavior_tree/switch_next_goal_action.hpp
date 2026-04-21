#ifndef MY_PACKAGE_PATH
#define MY_PACKAGE_PATH "/home/duzhong/zllc_ws/src/decision_behavior_tree"
#endif

#ifndef SWITCH_NEXT_GOAL_ACTION
#define SWITCH_NEXT_GOAL_ACTION

#include <behaviortree_cpp_v3/action_node.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>

class SwitchNextGoalAction : public BT::SyncActionNode
{
private:
    struct Goal
    {
        double x = 0.0;
        double y = 0.0;
        double orientation_z = 0.0;
        double orientation_w = 1.0;
    } __attribute__((packed));

    std::vector<Goal> goals_;
    size_t current_index_;

public:
    explicit SwitchNextGoalAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        // 加载 YAML 中的所有目标
        std::string yaml_path = std::string(MY_PACKAGE_PATH) + "/param/" + "seq_goal_poses.yaml";
        loadGoalsFromYAML(yaml_path);
        if (goals_.empty())
        {
            throw BT::RuntimeError("No goals found in the YAML file: " + yaml_path);
        }
        current_index_ = 0;
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        const auto &goal = goals_[current_index_];

        // 获取当前节点的黑板
        auto blackboard = config().blackboard;

        // 更新黑板中的目标值
        blackboard->set("goal_x", goal.x);
        blackboard->set("goal_y", goal.y);
        blackboard->set("goal_orientation_z", goal.orientation_z);
        blackboard->set("goal_orientation_w", goal.orientation_w);

        ROS_INFO("switch to next goal x:%.2f\ty:%.2f\tz:%.2f\tw:%.2f", goal.x, goal.y, goal.orientation_z, goal.orientation_w);

        // 切换到下一个目标（循环）
        current_index_ = (current_index_ + 1) % goals_.size();

        return BT::NodeStatus::SUCCESS;
    }

    void loadGoalsFromYAML(const std::string &path)
    {
        try
        {
            int goal_count = 0;
            YAML::Node yaml_node = YAML::LoadFile(path);

            for (const auto &pair : yaml_node)
            {
                YAML::Node value_node = pair.second;
                if (value_node["x"] && value_node["y"])
                {
                    Goal goal;
                    goal.x = value_node["x"].as<double>();
                    goal.y = value_node["y"].as<double>();
                    goal.orientation_z = value_node["z"].as<double>();
                    goal.orientation_w = value_node["w"].as<double>();
                    goals_.push_back(goal);
                    std::cout << "loaded goal " << ++goal_count
                              << "\tx:" << goal.x
                              << "\ty:" << goal.y
                              << "\tz:" << goal.orientation_z
                              << "\tw:" << goal.orientation_w
                              << std::endl;
                }
            }
        }
        catch (const YAML::Exception &ex)
        {
            std::cerr << "YAML loading error: " << ex.what() << std::endl;
        }
    }
};

#endif // SWITCH_NEXT_GOAL_ACTION