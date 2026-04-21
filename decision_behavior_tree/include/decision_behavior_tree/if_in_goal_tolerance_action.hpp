#ifndef IF_IN_GOAL_TOLERANCE_ACTION
#define IF_IN_GOAL_TOLERANCE_ACTION

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class IfInGoalToleranceAction : public BT::SyncActionNode
{
private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double tolerance_xy_, tolerance_yaw_;

public:
    IfInGoalToleranceAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), tf_listener_(tf_buffer_)
    {
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "bt_check_if_in_goal");
        }
        tolerance_xy_ = 0.35;
        tolerance_yaw_ = 2 * M_PI;
    }

    static BT::PortsList providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<double>("goal_x"));
        ports_list.insert(BT::InputPort<double>("goal_y"));
        ports_list.insert(BT::InputPort<double>("goal_orientation_z"));
        ports_list.insert(BT::InputPort<double>("goal_orientation_w"));
        ports_list.insert(BT::InputPort<double>("tolerance_xy"));
        ports_list.insert(BT::InputPort<double>("tolerance_yaw"));
        return ports_list;
    }

    BT::NodeStatus tick() override
    {
        double goal_x, goal_y, goal_orient_z, goal_orient_w;

        if (!getInput<double>("goal_x", goal_x) ||
            !getInput<double>("goal_y", goal_y) ||
            !getInput<double>("goal_orientation_z", goal_orient_z) ||
            !getInput<double>("goal_orientation_w", goal_orient_w) ||
            !getInput<double>("tolerance_xy", tolerance_xy_) ||
            !getInput<double>("tolerance_yaw", tolerance_yaw_))
        {
            ROS_ERROR("Failed to get all input ports.");
            return BT::NodeStatus::FAILURE;
        }

        try
        {
            geometry_msgs::TransformStamped transform_;
            transform_ = tf_buffer_.lookupTransform(
                "map", "base_link", ros::Time(0), ros::Duration(1.0));
            double distance_2 = pow(goal_x - transform_.transform.translation.x, 2) +
                                pow(goal_y - transform_.transform.translation.y, 2);

            double roll, pitch, yaw_true, yaw_goal;
            tf2::Quaternion q_true(0, 0, transform_.transform.rotation.z, transform_.transform.rotation.w);
            tf2::Quaternion q_goal(0, 0, goal_orient_z, goal_orient_w);
            tf2::Matrix3x3 m_true(q_true);
            m_true.getRPY(roll, pitch, yaw_true);
            tf2::Matrix3x3 m_goal(q_goal);
            m_goal.getRPY(roll, pitch, yaw_goal);

            if (distance_2 <= tolerance_xy_ * tolerance_xy_ &&
                yaw_true - yaw_goal <= tolerance_yaw_)
            {
                ROS_INFO("now is in goal tolerance");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_WARN("now is not in goal tolerance");
                return BT::NodeStatus::FAILURE;
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failed to get TF transform: %s", ex.what());
            return BT::NodeStatus::FAILURE;
        }
    }
};

#endif // IF_IN_GOAL_TOLERANCE_ACTION