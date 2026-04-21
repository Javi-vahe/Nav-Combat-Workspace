#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <fstream>
#include <sstream>
#include <string>

class PathLoader
{
public:
    PathLoader()
    {
        // 获取参数：默认 frame_id
        nh_.param<std::string>("frame_id", frame_id_, "map");
        nh_.param<std::string>("file_folder", file_folder_, "/home/duzhong/dzacs/src/resource/road_paths");

        // 创建发布者（latch=true，发布一次即可）
        path_pub_ = nh_.advertise<nav_msgs::Path>("/pure_pursuit/my_path", 1, true);

        // 订阅 string 话题来触发路径加载
        sub_ = nh_.subscribe("/pure_pursuit/load_path", 1, &PathLoader::loadPathCallback, this);

        ROS_INFO("PathLoader ready. Subscribed to 'load_path' topic. Awaiting string messages...");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Subscriber sub_;
    std::string frame_id_;
    std::string file_folder_;

    void loadPathCallback(const std_msgs::String::ConstPtr &msg)
    {
        const std::string &file_path = file_folder_ + "/" + msg->data;

        ROS_INFO("Received request to load path from: %s", file_path.c_str());

        nav_msgs::Path path_msg;
        path_msg.header.frame_id = frame_id_;
        path_msg.header.stamp = ros::Time(0); // 时间戳设为 0，也可用 ros::Time::now()

        std::ifstream file(file_path);
        if (!file.is_open())
        {
            ROS_ERROR("Could not open file: %s", file_path.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line))
        {
            // 去除行尾空格和逗号
            line.erase(line.find_last_not_of(" \t\n\r,") + 1);
            if (line.empty())
                continue;

            std::stringstream ss(line);
            std::string x_str, y_str, oz_str, ow_str;

            if (std::getline(ss, x_str, ',') &&
                std::getline(ss, y_str, ',') &&
                std::getline(ss, oz_str, ',') &&
                std::getline(ss, ow_str, ','))
            {

                try
                {
                    double x = std::stod(x_str);
                    double y = std::stod(y_str);
                    double oz = std::stod(oz_str);
                    double ow = std::stod(ow_str);

                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header = path_msg.header; // 继承 frame_id 和 stamp
                    pose_stamped.pose.position.x = x;
                    pose_stamped.pose.position.y = y;
                    pose_stamped.pose.position.z = 0.0;
                    pose_stamped.pose.orientation.x = 0.0;
                    pose_stamped.pose.orientation.y = 0.0;
                    pose_stamped.pose.orientation.z = oz;
                    pose_stamped.pose.orientation.w = ow;

                    path_msg.poses.push_back(pose_stamped);
                }
                catch (const std::exception &e)
                {
                    ROS_WARN("Failed to parse line: %s, error: %s", line.c_str(), e.what());
                }
            }
            else
            {
                ROS_WARN("Invalid line format (expected x,y,oz,ow): %s", line.c_str());
            }
        }

        file.close();

        if (path_msg.poses.empty())
        {
            ROS_WARN("No valid poses loaded from %s", file_path.c_str());
            return;
        }

        // 发布路径（只发布一次）
        path_pub_.publish(path_msg);
        ROS_INFO("Published path with %zu poses from %s", path_msg.poses.size(), file_path.c_str());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_loader_node");
    PathLoader loader;
    ros::spin();
    return 0;
}