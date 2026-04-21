#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>

class TebToleranceChanger
{
public:
    TebToleranceChanger() : nh_("~")
    {
        // 获取参数服务名称（默认为TEB的标准路径）
        nh_.param<std::string>("teb_config_service", teb_config_service_, 
                              "/move_base/TebLocalPlannerROS/set_parameters");
        
        // 创建服务客户端
        dr_client_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(teb_config_service_);
        
        // 等待服务可用
        if (!dr_client_.waitForExistence(ros::Duration(5.0))) {
            ROS_WARN("Timed out waiting for dynamic reconfigure service: %s", 
                    teb_config_service_.c_str());
        } else {
            ROS_INFO("Connected to dynamic reconfigure service: %s", teb_config_service_.c_str());
        }

        // 订阅目标容忍度更新话题
        sub_ = nh_.subscribe("/change_goal_tolerance", 1, &TebToleranceChanger::toleranceCallback, this);
        ROS_INFO("TEB Tolerance Changer initialized. Listening on /change_goal_tolerance");
    }

    void toleranceCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        ROS_INFO("Received new tolerance values: xy=%.3f, yaw=%.3f", msg->x, msg->theta);
        
        // 创建参数配置请求
        dynamic_reconfigure::ReconfigureRequest request;
        dynamic_reconfigure::ReconfigureResponse response;
        
        // 添加 xy_goal_tolerance 参数
        dynamic_reconfigure::DoubleParameter xy_param;
        xy_param.name = "xy_goal_tolerance";
        xy_param.value = msg->x;
        request.config.doubles.push_back(xy_param);
        
        // 添加 yaw_goal_tolerance 参数
        dynamic_reconfigure::DoubleParameter yaw_param;
        yaw_param.name = "yaw_goal_tolerance";
        yaw_param.value = msg->theta;
        request.config.doubles.push_back(yaw_param);
        
        // 发送请求
        if (dr_client_.call(request, response)) {
            ROS_INFO("Successfully updated TEB parameters!");
        } else {
            ROS_ERROR("Failed to update TEB parameters via service: %s", teb_config_service_.c_str());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::ServiceClient dr_client_;
    std::string teb_config_service_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teb_tolerance_changer");
    TebToleranceChanger changer;
    ros::spin();
    return 0;
}