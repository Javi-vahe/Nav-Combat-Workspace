#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <geometry_msgs/PoseStamped.h>

namespace costmap_2d
{
static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;
}

class PathChecker
{
private:
    nav_msgs::Path global_path_;
    nav_msgs::OccupancyGrid local_costmap_;
    bool path_set_ = false;
    bool map_set_ = false;

public:
    PathChecker() {}

    void setGlobalPath(const nav_msgs::Path &path)
    {
        global_path_ = path;
        path_set_ = true;
    }

    void setCostmap(const nav_msgs::OccupancyGrid &costmap)
    {
        local_costmap_ = costmap;
        map_set_ = true;
    }

    bool checkPath(const int idx)
    {
        if (!path_set_ || !map_set_)
        {
            ROS_WARN("Path or costmap not set yet.");
            return false;
        }

        if (global_path_.poses.empty())
        {
            ROS_WARN("Empty global path.");
            return false;
        }

        const auto &info = local_costmap_.info;
        ROS_INFO("Checking path from index %d to %lu", idx, global_path_.poses.size());

        for (size_t i = idx; i < global_path_.poses.size(); ++i)
        {
            const auto &pose = global_path_.poses[i].pose;
            int mx, my;

            if (!worldToMap(pose.position.x, pose.position.y, mx, my))
            {
                // 点超出地图边界，跳过，不检查
                // ROS_WARN("Point [%f, %f] out of map", pose.position.x, pose.position.y);
                continue;
            }

            int index = my * info.width + mx;
            if (index >= 0 && index < static_cast<int>(local_costmap_.data.size()))
            {
                int cost = local_costmap_.data[index];
                if (cost == costmap_2d::NO_INFORMATION)
                {
                    // continue;
                }
                else if (cost >= 30)
                {
                    ROS_INFO("Path blocked at index %lu (map [%d, %d]) with cost: %d", i, mx, my, cost);
                    return false;
                }
                if (cost != 0)
                    ROS_INFO("Pose %lu at map [%d, %d] has cost: %d", i, mx, my, cost);
            }
        }

        return true;
    }

private:
    bool worldToMap(double wx, double wy, int &mx, int &my)
    {
        double origin_x = local_costmap_.info.origin.position.x;
        double origin_y = local_costmap_.info.origin.position.y;
        double resolution = local_costmap_.info.resolution;

        mx = static_cast<int>((wx - origin_x) / resolution);
        my = static_cast<int>((wy - origin_y) / resolution);

        // 检查是否越界
        return (mx >= 0 && mx < static_cast<int>(local_costmap_.info.width) &&
                my >= 0 && my < static_cast<int>(local_costmap_.info.height));
    }
};
