#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <cctype>

class AreaRecorder
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber clicked_point_sub_;
    ros::Publisher marker_pub_;

    std::string save_path_;
    std::thread user_thread_;
    std::mutex mutex_;

    // 当前正在记录的区域
    std::string current_area_name_;
    std::vector<geometry_msgs::Point> current_area_points_;

    // 所有已记录的区域
    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Point>>> recorded_areas_;

    int now_maker_id_ = 0; // 用于标识当前Marker的ID
public:
    AreaRecorder(ros::NodeHandle &nh) : nh_(nh), nh_private_("~")
    {
        // 获取保存路径参数
        nh_private_.param<std::string>("save_path", save_path_, "~/recorded_areas.yaml");
        ROS_INFO("Areas will be saved to: %s", save_path_.c_str());

        // 初始化发布者和订阅者
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("recorded_area_marker", 10, true);
        clicked_point_sub_ = nh.subscribe("/clicked_point", 10, &AreaRecorder::pointCallback, this);

        // 启动用户交互线程
        user_thread_ = std::thread(&AreaRecorder::userInteractionThread, this);
    }

    ~AreaRecorder()
    {
        if (user_thread_.joinable())
        {
            user_thread_.join();
        }
    }

private:
    void userInteractionThread()
    {
        ROS_INFO("Area Recorder started. Commands:");
        ROS_INFO("  Enter area name (format: self_xxx or enemy_xxx)");
        ROS_INFO("  Press 's' to save current area");
        ROS_INFO("  Press 'c' to clear current area");
        ROS_INFO("  Press 'q' to quit");

        while (ros::ok())
        {
            // 提示用户输入
            std::cout << "\nEnter area name (self_xxx or enemy_xxx), 'q' to quit: ";
            std::string input;
            std::getline(std::cin, input);

            // 处理退出命令
            if (input == "q")
            {
                ROS_INFO("Exiting Area Recorder");
                ros::shutdown();
                return;
            }

            // 处理保存命令
            if (input == "s")
            {
                saveCurrentArea();
                continue;
            }

            // 处理清除命令
            if (input == "c")
            {
                clearCurrentArea();
                continue;
            }

            // 验证区域名称格式
            if (!isValidAreaName(input))
            {
                ROS_WARN("Invalid area name: %s. Must start with 'self_' or 'enemy_'", input.c_str());
                continue;
            }

            // 设置新区域
            {
                std::lock_guard<std::mutex> lock(mutex_);
                current_area_name_ = input;
                current_area_points_.clear();
                ROS_INFO("Recording new area: %s. Click points in RViz...", current_area_name_.c_str());
            }

            // 等待用户完成区域绘制
            waitForAreaCompletion();
        }
    }

    bool isValidAreaName(const std::string &name)
    {
        // 检查名称是否以"self_"或"enemy_"开头
        return (name.find("self_") == 0) || (name.find("enemy_") == 0);
    }

    void waitForAreaCompletion()
    {
        ROS_INFO("Click points in RViz to define the polygon. Press 's' when done to save.");

        while (ros::ok())
        {
            std::cout << "> ";
            std::string cmd;
            std::getline(std::cin, cmd);

            if (cmd == "s")
            {
                saveCurrentArea();
                return;
            }
            else if (cmd == "c")
            {
                clearCurrentArea();
                return;
            }
            else if (cmd == "q")
            {
                ROS_INFO("Exiting Area Recorder");
                ros::shutdown();
                return;
            }
            else
            {
                std::cout << "Unknown command. Press 's' to save, 'c' to clear, 'q' to quit\n";
            }
        }
    }

    void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!current_area_name_.empty())
        {
            geometry_msgs::Point p = msg->point;
            current_area_points_.push_back(p);
            ROS_INFO("Added point %lu: (%.2f, %.2f)", current_area_points_.size(), p.x, p.y);

            // 更新可视化
            visualizeCurrentArea();
        }
    }

    void visualizeCurrentArea()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "recorded_area";
        marker.id = now_maker_id_;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;

        // 设置颜色: self为绿色, enemy为红色
        if (current_area_name_.find("self_") == 0)
        {
            marker.color.g = 1.0;
            marker.color.a = 1.0;
        }
        else
        {
            marker.color.r = 1.0;
            marker.color.a = 1.0;
        }

        // 添加点
        marker.points = current_area_points_;

        // 如果点数大于等于3，则闭合多边形
        if (!current_area_points_.empty() && current_area_points_.size() >= 3)
        {
            marker.points.push_back(current_area_points_[0]); // 闭合图形
        }

        marker_pub_.publish(marker);
    }

    void saveCurrentArea()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (current_area_name_.empty())
        {
            ROS_WARN("No current area to save");
            return;
        }

        if (current_area_points_.size() < 3)
        {
            ROS_WARN("Area must have at least 3 points. Current points: %lu", current_area_points_.size());
            return;
        }

        // 确定类别 (self 或 enemy)
        std::string category = (current_area_name_.find("self_") == 0) ? "self" : "enemy";
        std::string area_name = current_area_name_.substr(category.size() + 1); // 移除前缀

        // 添加到记录的区域
        recorded_areas_[category][area_name] = current_area_points_;

        // 保存到文件
        saveToYaml();

        ROS_INFO("Saved area: %s with %lu points", current_area_name_.c_str(), current_area_points_.size());

        // 重置当前区域
        current_area_name_.clear();
        current_area_points_.clear();

        // 清除可视化
        // visualization_msgs::Marker marker;
        // marker.header.frame_id = "map";
        // marker.header.stamp = ros::Time::now();
        // marker.ns = "recorded_area";
        // marker.id = 0;
        // marker.action = visualization_msgs::Marker::DELETE;
        // marker_pub_.publish(marker);
        now_maker_id_++; // 更新Marker ID
    }

    void clearCurrentArea()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        ROS_INFO("Clearing current area: %s", current_area_name_.c_str());
        current_area_name_.clear();
        current_area_points_.clear();

        // 清除可视化
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "recorded_area";
        marker.id = now_maker_id_;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub_.publish(marker);
    }

    void saveToYaml()
    {
        try
        {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;

            // 添加self区域
            if (recorded_areas_.find("self") != recorded_areas_.end())
            {
                emitter << YAML::Key << "self";
                emitter << YAML::Value << YAML::BeginMap;

                for (const auto &area : recorded_areas_["self"])
                {
                    emitter << YAML::Key << area.first;
                    emitter << YAML::Value << YAML::BeginSeq;

                    for (const auto &point : area.second)
                    {
                        emitter << YAML::Flow << YAML::BeginSeq << point.x << point.y << YAML::EndSeq;
                    }

                    emitter << YAML::EndSeq;
                }

                emitter << YAML::EndMap;
            }

            // 添加enemy区域
            if (recorded_areas_.find("enemy") != recorded_areas_.end())
            {
                emitter << YAML::Key << "enemy";
                emitter << YAML::Value << YAML::BeginMap;

                for (const auto &area : recorded_areas_["enemy"])
                {
                    emitter << YAML::Key << area.first;
                    emitter << YAML::Value << YAML::BeginSeq;

                    for (const auto &point : area.second)
                    {
                        emitter << YAML::Flow << YAML::BeginSeq << point.x << point.y << YAML::EndSeq;
                    }

                    emitter << YAML::EndSeq;
                }

                emitter << YAML::EndMap;
            }

            emitter << YAML::EndMap;

            // 写入文件
            std::ofstream fout(save_path_);
            if (fout.is_open())
            {
                fout << emitter.c_str();
                ROS_INFO("Successfully saved areas to: %s", save_path_.c_str());
            }
            else
            {
                ROS_ERROR("Failed to open file for writing: %s", save_path_.c_str());
            }
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("YAML error: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "area_recorder_node");
    ros::NodeHandle nh;

    AreaRecorder recorder(nh);

    // 使用多线程spinner处理ROS回调
    ros::MultiThreadedSpinner spinner(2); // 使用2个线程
    spinner.spin();

    return 0;
}