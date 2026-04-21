#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cmath>
#include <XmlRpcValue.h>
#include <map>
#include <iostream>

class AreaChecker
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_; // 用于访问私有参数
    ros::Publisher marker_array_pub_;
    ros::Publisher checking_area_pub_;     // 发布目前检查区域
    ros::Publisher check_area_result_pub_; // 发布检查结果
    ros::Subscriber check_area_sub_;       // 订阅需要check的区域
    ros::Timer checkerTimer_;
    tf::TransformListener tf_listener;
    std::string base_frame_ = "base_link"; // 需要检查的坐标系

    std::map<std::string, std::vector<geometry_msgs::Point>> predefined_polygons_; // 预定义多边形

    bool ifCheck_ = false; // 是否进行区域检查
    bool foundAreaInYaml_ = false;
    uint8_t area2Check_ = 0; // 需要检查的区域编号 001-020表示self区域，101-120表示enemy区域
    std::map<std::string, std::vector<geometry_msgs::Point>>::iterator area_2_check_it_;

public:
    AreaChecker(ros::NodeHandle &nh) : nh_(nh), private_nh_("~")
    {
        private_nh_.param<std::string>("base_frame", base_frame_, "base_center"); // 获取需要检查的坐标系
        // 订阅和发布
        marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/area_checker/polygon_array", 1, true);
        checking_area_pub_ = nh.advertise<std_msgs::UInt8>("/area_checker/checking_area", 1, true);        // 发布目前检查区域
        check_area_result_pub_ = nh.advertise<std_msgs::Bool>("/area_checker/check_area_result", 1, true); // 发布检查结果

        check_area_sub_ = nh.subscribe("/area_checker/check_area", 2, &AreaChecker::checkAreaCallback, this); // 订阅需要check的区域
        checkerTimer_ = nh_.createTimer(ros::Duration(1.0 / 10.0), &AreaChecker::check_timer, this);

        loadPolygonsFromParam(); // 从参数服务器加载预定义多边形
        visualizeAllPolygons();  // 可视化所有多边形
    }

    void check_timer(const ros::TimerEvent &)
    {
        if (!ifCheck_)
        {
            std_msgs::UInt8 check_area_msg;
            check_area_msg.data = 0;
            checking_area_pub_.publish(check_area_msg); // 发布当前未在检查信息
            return;
        }
        if (!foundAreaInYaml_) // （首次）还没在yaml中找到对应区域
        {
            std::string areaIndexStr = "self/a";          // 默认为 self 区域
            uint8_t tempIndex = 0;                        // 默认为 self 区域
            if (area2Check_ >= 101 && area2Check_ <= 120) // enemy区域
            {
                areaIndexStr = "enemy/a"; // enemy区域
                tempIndex = 100;
            }

            std::cout << "test area2Check_: " << std::to_string(area2Check_ - tempIndex)
                      << " string: " << areaIndexStr + std::to_string(area2Check_ - tempIndex) << std::endl;
            auto it = predefined_polygons_.find(areaIndexStr + std::to_string(area2Check_ - tempIndex));

            if (it != predefined_polygons_.end()) // 成功找到区域
            {
                area_2_check_it_ = it;   // 保存迭代器
                foundAreaInYaml_ = true; // 找到对应区域
                ROS_INFO("Found area: %s", it->first.c_str());
            }
            else // 找不到对应区域
            {
                ROS_ERROR("Area %d not found in predefined polygons.", area2Check_);
                return; // 如果没有找到对应区域，直接返回
            }
        }
        else // 已经在yaml中找到对应区域
        {
            std_msgs::Bool isInArea; // 是否在区域内topic消息
            isInArea.data = false;   // 默认不在区域内
            tf::StampedTransform transform;
            try
            {
                tf_listener.waitForTransform("map", base_frame_, ros::Time(0), ros::Duration(1.0));
                tf_listener.lookupTransform("map", base_frame_, ros::Time(0), transform);
                geometry_msgs::Point p;
                p.x = transform.getOrigin().x();
                p.y = transform.getOrigin().y();
                if (isPointInPolygon(p, area_2_check_it_->second))
                {
                    // std::cout << "in in in" << std::endl;
                    ROS_INFO("Robot is in area: %s", area_2_check_it_->first.c_str());
                    isInArea.data = true;
                }
                else
                {
                    // std::cout << "out out out" << std::endl;
                    ROS_WARN("Robot is NOT in area: %s", area_2_check_it_->first.c_str());
                }
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("lookupTransform failed: %s", ex.what());
            }

            check_area_result_pub_.publish(isInArea); // 发布检查结果
            std_msgs::UInt8 check_area_msg;
            check_area_msg.data = area2Check_;
            checking_area_pub_.publish(check_area_msg); // 发布当前检查区域
        }
    }

    void checkAreaCallback(const std_msgs::UInt8::ConstPtr &msg)
    {
        if (msg->data == area2Check_)
            return;
        area2Check_ = msg->data;
        foundAreaInYaml_ = false; // 重置状态
        if ((area2Check_ > 0 && area2Check_ <= 20) || (area2Check_ >= 101 && area2Check_ <= 120))
        {
            ifCheck_ = true; // 开始检查
            ROS_INFO("Checking area: %d", area2Check_);
        }
        else
        {
            ifCheck_ = false; // 停止检查
            ROS_WARN("Invalid area code: %d. Stopping checks.", area2Check_);
        }
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        geometry_msgs::Point p = msg->pose.pose.position;

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

    // 从YAML加载多边形
    void loadPolygonsFromParam()
    {
        ROS_INFO("Loading predefined polygons from parameters...");

        // 加载self区域
        XmlRpc::XmlRpcValue self_areas;
        if (private_nh_.getParam("self", self_areas))
            parsePolygonParam(self_areas, "self", predefined_polygons_);
        else
            ROS_WARN("No 'self' polygons defined in parameters.");

        // 加载enemy区域
        XmlRpc::XmlRpcValue enemy_areas;
        if (private_nh_.getParam("enemy", enemy_areas))
            parsePolygonParam(enemy_areas, "enemy", predefined_polygons_);
        else
            ROS_WARN("No 'enemy' polygons defined in parameters.");

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
            marker.color.a = 0.7;

            // 设置颜色: self为绿色, enemy为红色
            if (name.find("self") != std::string::npos)
                marker.color.g = 1.0;
            else
                marker.color.r = 1.0;

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
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "polygon_checker_node");
    ros::NodeHandle nh;
    AreaChecker checker(nh);
    ros::spin();
    return 0;
}