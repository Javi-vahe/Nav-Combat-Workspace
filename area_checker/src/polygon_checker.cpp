#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cmath>
#include <XmlRpcValue.h>
#include <map>
#include <iostream>

class PolygonChecker
{
public:
    PolygonChecker(ros::NodeHandle &nh) : nh_(nh), nh_private_("~")
    {
        // 订阅和发布
        clicked_point_sub_ = nh.subscribe("/clicked_point", 10, &PolygonChecker::pointCallback, this);
        polygon_pub_ = nh.advertise<visualization_msgs::Marker>("polygon_marker", 10, true);
        marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("polygon_array", 10, true);
        initial_pose_sub_ = nh.subscribe("/initialpose", 10, &PolygonChecker::poseCallback, this);
        goal_sub_ = nh.subscribe("/move_base_simple/goal", 10, &PolygonChecker::goalCallback, this);

        // 从参数服务器加载预定义多边形
        loadPolygonsFromParam();

        // 可视化所有多边形
        visualizeAllPolygons();
        std::cout << "area num: " << predefined_polygons_.size() << std::endl;
        auto it = predefined_polygons_.find("self/t1");
        if (it != predefined_polygons_.end())
        {
            std::cout << "found self/t1" << std::endl;;
        }   else
        {
            std::cout << "not found self/t1" << std::endl;
        }

        it = predefined_polygons_.find("self/a1");
        if (it != predefined_polygons_.end())
        {
            std::cout << "found self/a1" << std::endl;;
        }   else
        {
            std::cout << "not found self/a1" << std::endl;
        }
    }

    // 从YAML加载多边形
    void loadPolygonsFromParam()
    {
        ROS_INFO("Loading predefined polygons from parameters...");
        // 加载self区域
        XmlRpc::XmlRpcValue self_areas;
        if (nh_private_.getParam("self", self_areas))
        {
            parsePolygonParam(self_areas, "self", predefined_polygons_);
        }
        else
        {
            ROS_WARN("No 'self' polygons defined in parameters.");
        }

        // 加载enemy区域
        XmlRpc::XmlRpcValue enemy_areas;
        if (nh_private_.getParam("enemy", enemy_areas))
        {
            parsePolygonParam(enemy_areas, "enemy", predefined_polygons_);
        }
        else
        {
            ROS_WARN("No 'enemy' polygons defined in parameters.");
        }

        ROS_INFO("Loaded %lu predefined polygons", predefined_polygons_.size());
    }

    // 解析多边形参数
    void parsePolygonParam(XmlRpc::XmlRpcValue &param_value,
                           const std::string &category,
                           std::map<std::string, std::vector<geometry_msgs::Point>> &output_map)
    {
        if (param_value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("Invalid polygon structure for category: %s", category.c_str());
            return;
        }

        for (auto it = param_value.begin(); it != param_value.end(); ++it)
        {
            const std::string &area_name = it->first;
            XmlRpc::XmlRpcValue &points = it->second;

            if (points.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_WARN("Points for area %s/%s not an array", category.c_str(), area_name.c_str());
                continue;
            }

            std::vector<geometry_msgs::Point> polygon;
            for (int i = 0; i < points.size(); ++i)
            {
                XmlRpc::XmlRpcValue &point = points[i];
                if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() != 2)
                {
                    ROS_WARN("Invalid point data in %s/%s[%d]", category.c_str(), area_name.c_str(), i);
                    continue;
                }

                // 处理整数和浮点数类型
                double x = 0.0, y = 0.0;
                if (point[0].getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    x = static_cast<int>(point[0]);
                }
                else if (point[0].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    x = static_cast<double>(point[0]);
                }

                if (point[1].getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    y = static_cast<int>(point[1]);
                }
                else if (point[1].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    y = static_cast<double>(point[1]);
                }

                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = 0.0;
                polygon.push_back(p);
            }

            if (polygon.size() >= 3)
            {
                std::string full_name = category + "/" + area_name;
                output_map[full_name] = polygon;
                ROS_INFO("Loaded %s with %lu points", full_name.c_str(), polygon.size());
            }
            else
            {
                ROS_WARN("Area %s/%s discarded - needs at least 3 points (has %d)",
                         category.c_str(), area_name.c_str(), int(polygon.size()));
            }
        }
    }

    // 可视化所有多边形
    void visualizeAllPolygons()
    {
        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;

        // 预定义多边形可视化
        for (const auto &polygon_pair : predefined_polygons_)
        {
            const std::string &name = polygon_pair.first;
            const std::vector<geometry_msgs::Point> &points = polygon_pair.second;

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "predefined_polygons";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;

            // 设置颜色: self为绿色, enemy为红色
            if (name.find("self") != std::string::npos)
            {
                marker.color.g = 1.0;
                marker.color.a = 0.7;
            }
            else
            {
                marker.color.r = 1.0;
                marker.color.a = 0.7;
            }

            // 添加多边形点（闭合多边形）
            marker.points = points;
            if (!points.empty())
            {
                marker.points.push_back(points[0]); // 闭合图形
            }

            marker_array.markers.push_back(marker);
        }

        // 发布可视化
        if (!marker_array.markers.empty())
        {
            marker_array_pub_.publish(marker_array);
            ROS_INFO("Published %lu polygon markers", marker_array.markers.size());
        }
        else
        {
            ROS_WARN("No polygons to visualize");
        }
    }

    // 判断点是否在多边形内（射线交叉法）
    bool isPointInPolygon(const geometry_msgs::Point &p, const std::vector<geometry_msgs::Point> &polygon)
    {
        if (polygon.size() < 3)
            return false;

        int crossCount = 0;
        double x = p.x;
        double y = p.y;

        for (size_t i = 0; i < polygon.size(); ++i)
        {
            double x1 = polygon[i].x;
            double y1 = polygon[i].y;
            double x2 = polygon[(i + 1) % polygon.size()].x;
            double y2 = polygon[(i + 1) % polygon.size()].y;

            // 点在线段上
            if ((std::min(x1, x2) <= x && x <= std::max(x1, x2)) &&
                (std::min(y1, y2) <= y && y <= std::max(y1, y2)))
            {
                double cross = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1);
                if (fabs(cross) < 1e-6)
                {
                    return true;
                }
            }

            if (y1 == y2)
                continue; // 水平边跳过

            if (y < std::min(y1, y2) || y >= std::max(y1, y2))
                continue;

            double xIntersect = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
            if (x < xIntersect)
                crossCount++;
        }

        return crossCount % 2 == 1;
    }

private:
    void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        geometry_msgs::Point p = msg->point;
        polygon_points_.push_back(p);
        updatePolygonMarker();
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        geometry_msgs::Point p = msg->pose.pose.position;

        // 检查手动绘制的多边形
        if (!polygon_points_.empty())
        {
            if (isPointInPolygon(p, polygon_points_))
            {
                ROS_INFO("Point (%.2f, %.2f) is INSIDE the manual polygon.", p.x, p.y);
            }
            else
            {
                ROS_INFO("Point (%.2f, %.2f) is OUTSIDE the manual polygon.", p.x, p.y);
            }
        }

        // 检查所有预定义多边形
        bool found = false;
        for (const auto &polygon_pair : predefined_polygons_)
        {
            if (isPointInPolygon(p, polygon_pair.second))
            {
                ROS_INFO("Point (%.2f, %.2f) is INSIDE %s", p.x, p.y, polygon_pair.first.c_str());
                found = true;
            }
        }

        if (!found)
        {
            ROS_INFO("Point (%.2f, %.2f) is not in any predefined area.", p.x, p.y);
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & /*msg*/)
    {
        polygon_points_.clear();
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "polygon";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::DELETE;
        polygon_pub_.publish(marker);
    }

    void updatePolygonMarker()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "polygon";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.points = polygon_points_;
        if (!polygon_points_.empty() && polygon_points_.size() >= 3)
        {
            marker.points.push_back(polygon_points_[0]); // 闭合图形
        }

        polygon_pub_.publish(marker);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_; // 用于访问私有参数
    ros::Subscriber clicked_point_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher polygon_pub_;
    ros::Publisher marker_array_pub_;

    std::vector<geometry_msgs::Point> polygon_points_;                             // 手动绘制的多边形
    std::map<std::string, std::vector<geometry_msgs::Point>> predefined_polygons_; // 预定义多边形
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "polygon_checker_node");
    ros::NodeHandle nh;
    PolygonChecker checker(nh);
    ros::spin();
    return 0;
}