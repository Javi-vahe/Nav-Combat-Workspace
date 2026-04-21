#ifndef MOVE_BASE_ACTION_NODE_HPP
#define MOVE_BASE_ACTION_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Empty.h>

class MoveBaseActionNode : public BT::StatefulActionNode
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pid_stop_pub_;

public:
    using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

    explicit MoveBaseActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), client_("move_base", true)
    {
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "bt_move_base_client");
        }
        ROS_INFO("Waiting for move_base action server...");
        client_.waitForServer();
        ROS_INFO("Connected to move_base action server.");

        pid_stop_pub_ = nh_.advertise<std_msgs::Empty>("/pid_stop_plan", 10);
    }

    static BT::PortsList providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<double>("goal_x"));
        ports_list.insert(BT::InputPort<double>("goal_y"));
        ports_list.insert(BT::InputPort<double>("goal_orientation_z"));
        ports_list.insert(BT::InputPort<double>("goal_orientation_w"));
        return ports_list;
    }

    BT::NodeStatus onStart() override
    {
        double goal_x, goal_y, goal_orient_z, goal_orient_w;

        if (!getInput<double>("goal_x", goal_x) ||
            !getInput<double>("goal_y", goal_y) ||
            !getInput<double>("goal_orientation_z", goal_orient_z) ||
            !getInput<double>("goal_orientation_w", goal_orient_w))
        {
            ROS_ERROR("Failed to get all goal input ports.");
            return BT::NodeStatus::FAILURE;
        }

        // 构造目标点
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = goal_x;
        goal.target_pose.pose.position.y = goal_y;
        goal.target_pose.pose.orientation.z = goal_orient_z;
        goal.target_pose.pose.orientation.w = goal_orient_w;

        ROS_INFO("Sending goal to move_base: x=%.2f, y=%.2f, orient_z=%.2f, orient_w=%.2f",
                 goal_x, goal_y, goal_orient_z, goal_orient_w);

        client_.sendGoal(goal);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Navigation succeeded.");
            return BT::NodeStatus::SUCCESS;
        }
        else if (client_.getState().isDone())
        {
            ROS_WARN("Navigation failed: %s", client_.getState().toString().c_str());
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING; // still navigating
    }

    void onHalted() override
    {
        ROS_INFO("Canceling current goal.");
        std_msgs::Empty stop_msg;
        pid_stop_pub_.publish(stop_msg);
        client_.cancelAllGoals();
    }

private:
    MoveBaseClient client_;
};

#endif // MOVE_BASE_ACTION_NODE_HPP