#ifndef MY_PACKAGE_PATH
#define MY_PACKAGE_PATH "/home/duzhong/zllc_ws/src/decision_behavior_tree"
#endif

#ifndef SWITCH_GOAL_TO_ACTION
#define SWITCH_GOAL_TO_ACTION

#include <behaviortree_cpp_v3/action_node.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>

class SwitchGoalToAction : public BT::SyncActionNode
{
private:
    struct Goal
    {
        float x = 0.0;
        float y = 0.0;
        float orientation_z = 0.0;
        float orientation_w = 1.0;
    } __attribute__((packed));

    std::map<std::string, Goal> material_goals_;
    Goal home_goal_;
    bool loaded_goals = false;

public:
    explicit SwitchGoalToAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        // 加载 YAML 中的所有目标
        std::string yaml_path = std::string(MY_PACKAGE_PATH) + "/param/" + "material_goal_poses.yaml";
        loadGoalsFromYAML(yaml_path, material_goals_);
        if (material_goals_.size() < 40)
        {
            throw BT::RuntimeError("No enough material goals found in the YAML file: " + yaml_path);
        }
        home_goal_.x = 0.0;
        home_goal_.y = 0.0;
        home_goal_.orientation_z = 0.0;
        home_goal_.orientation_w = 1.0;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("goal_id", 0, "1-20 self, 101-120 enemy, 0 home")};
    }

    BT::NodeStatus tick() override
    {
        int goal_id;
        if (!getInput<int>("goal_id", goal_id))
        {
            throw BT::RuntimeError("missing required input [goal_id]");
        }
        std::cout << "Get material goal id: " << static_cast<int>(goal_id) << std::endl;

        // 获取当前节点的黑板
        auto blackboard = config().blackboard;
        if (goal_id == 0)
        {
            // 如果目标 ID 为 0，则使用 home_goal_
            // 更新黑板
            blackboard->set("material_goal_x", static_cast<double>(home_goal_.x));
            blackboard->set("material_goal_y", static_cast<double>(home_goal_.y));
            blackboard->set("material_goal_z", static_cast<double>(home_goal_.orientation_z));
            blackboard->set("material_goal_w", static_cast<double>(home_goal_.orientation_w));
            std::cout << "Switch to home goal." << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else if (goal_id > 0 && goal_id <= 20) // 己方物资点
        {
            auto it = material_goals_.find("g" + std::to_string(goal_id));
            if (it != material_goals_.end())
            {
                const Goal &goal = it->second;
                // 更新黑板
                blackboard->set("material_goal_x", static_cast<double>(goal.x));
                blackboard->set("material_goal_y", static_cast<double>(goal.y));
                blackboard->set("material_goal_z", static_cast<double>(goal.orientation_z));
                blackboard->set("material_goal_w", static_cast<double>(goal.orientation_w));
                std::cout << "Switch to self material goal: " << goal_id << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
        }
        else if (goal_id > 100 && goal_id <= 120) // 敌方物资点
        {
            auto it = material_goals_.find("g" + std::to_string(goal_id));
            if (it != material_goals_.end())
            {
                const Goal &goal = it->second;
                // 更新黑板
                blackboard->set("material_goal_x", static_cast<double>(goal.x));
                blackboard->set("material_goal_y", static_cast<double>(goal.y));
                blackboard->set("material_goal_z", static_cast<double>(goal.orientation_z));
                blackboard->set("material_goal_w", static_cast<double>(goal.orientation_w));
                std::cout << "Switch to enemy material goal: " << (goal_id - 100) << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
        }

        return BT::NodeStatus::FAILURE;
    }

    void loadGoalsFromYAML(const std::string &path, std::map<std::string, Goal> &goals_)
    {
        try
        {
            int goal_count = 0;
            YAML::Node yaml_node = YAML::LoadFile(path);

            for (const auto &pair : yaml_node)
            {
                YAML::Node value_node = pair.second;
                if (value_node["x"] && value_node["y"] && value_node["z"] && value_node["w"])
                {
                    auto goal_name = pair.first.as<std::string>();
                    Goal goal;
                    goal.x = value_node["x"].as<float>();
                    goal.y = value_node["y"].as<float>();
                    goal.orientation_z = value_node["z"].as<float>();
                    goal.orientation_w = value_node["w"].as<float>();
                    goals_[goal_name] = goal;
                    std::cout << "loaded goal " << ++goal_count << " :" << pair.first.as<std::string>()
                              << "\tx:" << goal.x
                              << "\ty:" << goal.y
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

#endif // SWITCH_GOAL_TO_ACTION