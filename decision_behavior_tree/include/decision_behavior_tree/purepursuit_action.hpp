#ifndef PUREPURSUIT_ACTION_HPP
#define PUREPURSUIT_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

class PurePursuitAction : public BT::StatefulActionNode
{
    enum PlannerStatus
    {
        PLANNER_IDLE = 0, // 空闲状态
        PLANNER_RUNNING = 1,
        PLANNER_FINISHED = 2,
        PLANNER_FAILED = 3,
        PLANNER_ABORTED = 4
    };

private:
    ros::NodeHandle nh_;
    ros::Publisher planner_stop_pub_, path_file_pub_;
    ros::Subscriber planner_status_sub_;

    int start_position_ = 0; // 用于记录起始位置
    int goal_position_ = 0;  // 用于记录目标位置

    std_msgs::UInt8 planner_status_;
    ros::Time planner_start_time_;

public:
    explicit PurePursuitAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "bt_pp_client");
        }

        planner_stop_pub_ = nh_.advertise<std_msgs::Bool>("/pure_pursuit/plan_enable", 10);
        path_file_pub_ = nh_.advertise<std_msgs::String>("/pure_pursuit/load_path", 1);

        planner_status_sub_ = nh_.subscribe("/pure_pursuit/status", 10, &PurePursuitAction::plannerStatusCB,
                                            this, ros::TransportHints().tcpNoDelay());
    }

    static BT::PortsList providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<int>("start_pos"));
        ports_list.insert(BT::InputPort<int>("goal_pos"));
        return ports_list;
    }

    BT::NodeStatus onStart() override
    {
        static int now_material_id_ = 0; // 当前材料ID
        if (!getInput<int>("start_pos", start_position_) ||
            !getInput<int>("goal_pos", goal_position_))
        {
            ROS_ERROR("Failed to get start or goal input ports.");
            return BT::NodeStatus::FAILURE;
        }
        if (goal_position_ == 0 && start_position_ != goal_position_) // 最后一个自家物资返回时
        {
        }
        else if (start_position_ == 0 && goal_position_ == 21)
        {
        }
        else if ((start_position_ < 0) || (goal_position_ < 0) ||
                 (start_position_ > 20) || (goal_position_ > 20) ||
                 (start_position_ == goal_position_))
        {
            ROS_ERROR("Start {%d} or goal {%d} positions invalid.", start_position_, goal_position_);
            return BT::NodeStatus::FAILURE;
        }
        std::cout << "PP start:" << start_position_ << "\tgoal:" << goal_position_ << std::endl;

        /***** 一定要先调用去物资点，再调用从该物资点返回基地。成对调用才不会出bug *****/
        if (start_position_ == 0) // 从起点去物资点
        {
            now_material_id_ = goal_position_; // 记录去了哪个物资点
            std::cout << "aaa now_material_id_=" << now_material_id_ << std::endl;
        }
        else if (goal_position_ == 0) // 返程
        {
            std::cout << "bbb now_material_id_=" << now_material_id_ << std::endl;
            start_position_ = now_material_id_; // 从物资点开始返回基地
        }
        std::cout << "ccc now_material_id_=" << now_material_id_ << std::endl;
        std_msgs::String pathFileName;
        pathFileName.data = std::to_string(start_position_) + "_" + std::to_string(goal_position_) + ".txt";
        path_file_pub_.publish(pathFileName);
        std::cout << "published " << pathFileName.data << std::endl;
        planner_start_time_ = ros::Time::now();

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        ros::spinOnce();
        switch (planner_status_.data)
        {
        case PLANNER_RUNNING:
            return BT::NodeStatus::RUNNING; // still navigating
            break;
        case PLANNER_FINISHED:
            if (ros::Time::now() - planner_start_time_ < ros::Duration(1.0))
            {
                ROS_WARN("PP Navigation finished too quickly: %d to %d", start_position_, goal_position_);
                break;
            }
            ROS_INFO("PP Navigation finished: %d to %d", start_position_, goal_position_);
            return BT::NodeStatus::SUCCESS;
            break;
        case PLANNER_FAILED:
            ROS_ERROR("PP Navigation failed: %d to %d", start_position_, goal_position_);
            return BT::NodeStatus::FAILURE;
            break;
        case PLANNER_ABORTED:
            ROS_WARN("PP Navigation aborted: %d to %d", start_position_, goal_position_);
            return BT::NodeStatus::FAILURE;
            break;
        default:
            break;
        }
        return BT::NodeStatus::RUNNING; // still navigating
    }

    void onHalted() override
    {
        ROS_INFO("Canceling current PP goal.");
        std_msgs::Bool stop_msg;
        stop_msg.data = false;
        planner_stop_pub_.publish(stop_msg);
    }

    void plannerStatusCB(const std_msgs::UInt8::ConstPtr msg)
    {
        planner_status_ = *msg;
    }
};

#endif // PUREPURSUIT_ACTION_HPP