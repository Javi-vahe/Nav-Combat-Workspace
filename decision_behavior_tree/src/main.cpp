// #include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
// #include <behaviortree_cpp_v3/utils/shared_library.h>
// #include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ros/ros.h>
#include <decision_behavior_tree/move_base_action_node.hpp>
#include <decision_behavior_tree/subscriber_callbacks.hpp>
#include <decision_behavior_tree/change_pid_speed_action.hpp>
#include <decision_behavior_tree/delay_seconds_action.hpp>
#include <decision_behavior_tree/if_detect_target_action.hpp>
#include <decision_behavior_tree/is_aiming_target_action.hpp>
#include <decision_behavior_tree/switch_next_goal_action.hpp>
#include <decision_behavior_tree/if_in_goal_tolerance_action.hpp>
#include <decision_behavior_tree/switch_goal_to_action.hpp>
#include <decision_behavior_tree/if_equal_int_action.hpp>
#include <decision_behavior_tree/compare_bigger_int_action.hpp>
#include <decision_behavior_tree/pub_goto_material_action.hpp>
#include <decision_behavior_tree/wait_voice_action.hpp>
#include <decision_behavior_tree/change_goal_tolerance_action.hpp>
#include <decision_behavior_tree/print_action.hpp>
#include <decision_behavior_tree/purepursuit_action.hpp>
#include <decision_behavior_tree/gimbal_cruise_action.hpp>
#include <decision_behavior_tree/wait_shoot_action.hpp>

void initBlackboard(BT::Blackboard::Ptr blackboard)
{
    blackboard->set("goal_x", 0.0);
    blackboard->set("goal_y", 0.0);
    blackboard->set("goal_orientation_z", 0.0);
    blackboard->set("goal_orientation_w", 1.0);

    blackboard->set("material_goal_x", 0.0);
    blackboard->set("material_goal_y", 0.0);
    blackboard->set("material_goal_z", 0.0);
    blackboard->set("material_goal_w", 1.0);
    blackboard->set("material_goal_id", 0);

    blackboard->set("enemy_hp", 10);             // 对方血量
    blackboard->set("self_hp", 10);              // 自身血量
    blackboard->set("being_hit", 16);            // 是否被敌方击打,16未被击打
    blackboard->set("shooting_count", 0);        // 已射击次数
    blackboard->set("ammo", 10);                 // 剩余弹药数量
    blackboard->set("last_being_hit", 16);       // 上一次被击打的装甲板
    blackboard->set("last_being_hit_time", 0.0); // 上一次被击打的时间

    blackboard->set("hold_material", 0);
    blackboard->set("chassis_mode", 1); // 0上位机 1下位机

    blackboard->set("correct_material", 0);
}

int main(int argc, char **argv)
{
    std::string xml_file = "main_tree.xml";
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "decision_behavior_tree_node");
    ros::NodeHandle nh("~");
    nh.param<std::string>("bt_xml_file", xml_file, "main_tree.xml");
    std::cout << "bt_xml_file: " << xml_file << std::endl;

    // 创建共享黑板
    auto blackboard = BT::Blackboard::create();
    initBlackboard(blackboard);

    ros::Subscriber sub_goal = nh.subscribe<geometry_msgs::PoseStamped>(
        "/test/goal", 1,
        boost::bind(goalCallback, _1, blackboard), ros::VoidPtr());
    ros::Subscriber sub_closest_material = nh.subscribe<std_msgs::UInt8>(
        "/closest_material", 1,
        boost::bind(cloestMaterialCB, _1, blackboard), ros::VoidPtr());
    ros::Subscriber sub_hp_and_hitmsg = nh.subscribe<std_msgs::UInt8MultiArray>(
        "/HpAndHitmsg", 1,
        boost::bind(hpAndHitmsgCB, _1, blackboard), ros::VoidPtr(), ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_hold_material = nh.subscribe<std_msgs::UInt8>(
        "/hold_material", 1,
        boost::bind(holdMaterialCB, _1, blackboard), ros::VoidPtr());
    ros::Subscriber sub_chassis_mode = nh.subscribe<std_msgs::UInt8>(
        "/chassis_mode", 10,
        boost::bind(chassisModeCB, _1, blackboard), ros::VoidPtr());
    ros::Subscriber sub_material_status = nh.subscribe<std_msgs::UInt8MultiArray>(
        "/self_Material_Number", 1,
        boost::bind(selfMaterialNumberCB, _1, blackboard), ros::VoidPtr());

    // 创建行为树工厂并注册自定义节点
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<MoveBaseActionNode>("MoveBaseAction");
    factory.registerNodeType<ChangeGoalToleranceAction>("ChangeGoalTolerance");
    factory.registerNodeType<ChangePIDSpeedAction>("ChangePIDSpeed");
    factory.registerNodeType<DelayAction>("DelayAction");
    factory.registerNodeType<IfDetectTargetAction>("IfDetectTarget");
    factory.registerNodeType<IsAimingAction>("IsAiming");
    factory.registerNodeType<SwitchNextGoalAction>("SwitchNextGoal");
    factory.registerNodeType<IfInGoalToleranceAction>("IfInGoalTolerance");
    factory.registerNodeType<SwitchGoalToAction>("SwitchGoalTo");
    factory.registerNodeType<IfEqualIntAction>("IfEqualInt");
    factory.registerNodeType<CompareBiggerIntAction>("CompareBiggerInt");
    factory.registerNodeType<PubGotoMaterialAction>("PubGotoMaterial");
    factory.registerNodeType<WaitVoiceAction>("WaitVoice");
    factory.registerNodeType<PrintAction>("PrintAction");
    factory.registerNodeType<PurePursuitAction>("PurePursuitAction");
    factory.registerNodeType<GimbalCruiseAction>("GimbalCruise");
    factory.registerNodeType<WaitShootAction>("WaitShoot");

    // XML 行为树文件路径
    xml_file = std::string(MY_PACKAGE_PATH) + "/behavior_tree/" + xml_file;
    ROS_INFO("Behavior tree XML file: %s", xml_file.c_str());

    auto tree = factory.createTreeFromFile(xml_file, blackboard);

    // auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

    // sleep(3);

    ros::Rate rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        tree.tickRoot();
        rate.sleep();
    }

    ros::shutdown();

    return 0;
}