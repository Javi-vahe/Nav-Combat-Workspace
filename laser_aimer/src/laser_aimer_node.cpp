#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <opencv2/opencv.hpp>

class LaserAimer
{
private:
    ros::NodeHandle &nh_;
    ros::NodeHandle private_nh_{"~"};
    ros::Publisher mapCloudPub_;
    ros::Publisher laserCloudROIPub_, laserCloudFilteredPub_;
    ros::Publisher markerPub_;
    ros::Publisher targetPub_; // 发布base_link系下的目标点
    ros::Subscriber laserScanSub_;
    ros::Subscriber mapSub_;

    nav_msgs::OccupancyGrid::ConstPtr static_map_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    sensor_msgs::PointCloud2 cloudInMap_;

    std::vector<geometry_msgs::Point> roiPolygon_;

    std::string baseFrame_ = "base_link";
    std::string laserFrame_ = "base_laser";

    bool enableClustering_ = true; // 是否启用聚类
    bool useFixedROI_ = false;     // 是否使用固定的ROI多边形
    double differentialTolerance_ = 0.1;

public:
    LaserAimer(ros::NodeHandle &nh) : nh_(nh), tf_buffer_(ros::Duration(1.0)), // 设置缓存时间
                                      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
    {
        // 读取参数
        std::string laser_topic;
        private_nh_.param<std::string>("laser_topic", laser_topic, "scan");
        private_nh_.param<std::string>("base_frame", baseFrame_, "base_link");
        private_nh_.param<std::string>("laser_frame", laserFrame_, "base_laser");
        private_nh_.param<bool>("enable_clustering", enableClustering_, true);
        private_nh_.param<bool>("use_fixed_roi", useFixedROI_, false);                    // false 使用地图差分提取roi
        private_nh_.param<double>("differential_tolerance", differentialTolerance_, 0.1); // 差分容忍度
        if (useFixedROI_)
        {
            ROS_INFO("使用固定区域作roi");
            getROIPolygon();
        }
        else
        {
            ROS_INFO("使用地图差分提取roi, 容忍度: %.2f 米", differentialTolerance_);
        }

        mapCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_in_map", 1);
        laserCloudROIPub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_roi", 1);
        laserCloudFilteredPub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_roi_filtered", 1);
        markerPub_ = nh_.advertise<visualization_msgs::Marker>("roi_circle_marker", 1);
        targetPub_ = nh_.advertise<geometry_msgs::PointStamped>("target_point_laser", 10);

        // 初始化订阅
        laserScanSub_ = nh_.subscribe(laser_topic, 10, &LaserAimer::laserScanCB, this, ros::TransportHints().tcpNoDelay());
        if (!useFixedROI_)
            mapSub_ = nh_.subscribe("/map", 1, &LaserAimer::mapCallback, this, ros::TransportHints().tcpNoDelay());
    }
    void laserScanCB(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        // ros::Time start_time = ros::Time::now();
        laserFrame_ = msg->header.frame_id;
        if (!useFixedROI_ && !static_map_)
        {
            return;
        }
        // 把雷达点转为map下的点云
        if (!laserScanToCloudInMap(msg))
        {
            return;
        }
        // 在map系提取roi,然后转回雷达坐标系
        pcl::PointCloud<pcl::PointXYZ> roi_cloud_in_laser;
        if (!getPointsInROI(roi_cloud_in_laser))
        {
            return;
        }

        // 半径滤波
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(roi_cloud_in_laser.makeShared());
        outrem.setRadiusSearch(0.2);       // 搜索半径，单位：米
        outrem.setMinNeighborsInRadius(1); // 至少要有1个邻居（即：聚集 ≥2 个点）
        outrem.filter(filtered_cloud);

        // 发布滤波后的点云
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(filtered_cloud, filtered_msg);
        filtered_msg.header.frame_id = laserFrame_;
        filtered_msg.header.stamp = msg->header.stamp;
        laserCloudFilteredPub_.publish(filtered_msg);

        if (enableClustering_) // 聚类
        {
            getTargetsCluters(filtered_cloud);
        }
        else // 不聚类
        {
            getTarget(filtered_cloud);
        }
        // ros::Time end_time = ros::Time::now();
        // ROS_INFO("Processing time: %.3f ms", (end_time - start_time).toSec() * 1000.0);
    }

    // 过滤后的点云聚类提取多目标,然后构造最小外接圆
    void getTargetsCluters(const pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud.makeShared());

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.25); // 聚类半径（单位：米）
        ec.setMinClusterSize(2);      // 最小点数
        ec.setMaxClusterSize(180);     // 可调
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud.makeShared());
        ec.extract(cluster_indices);

        // 拆分出各个 cluster
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ> cluster;
            for (int idx : indices.indices)
                cluster.push_back(cloud.points[idx]);
            clusters.push_back(cluster);
        }

        int max_points = 0;
        int main_cluster_idx = -1;
        // 先找出点数最多的聚类
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            if (clusters[i].size() > max_points)
            {
                max_points = clusters[i].size();
                main_cluster_idx = i;
            }
        }

        // 再遍历并处理每个聚类
        int marker_id = 0;
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            const auto &cluster = clusters[i];
            std::vector<cv::Point2f> pts2D;
            for (const auto &pt : cluster.points)
                pts2D.emplace_back(pt.x, pt.y);

            if (pts2D.size() < 2)
                continue; // 小于2个点的聚类不提取圆

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(pts2D, center, radius);

            // ROS_INFO("Cluster #%d: center(%.2f, %.2f), radius=%.2f", marker_id, center.x, center.y, radius);

            // 计算 yaw（相对于 base_link）
            geometry_msgs::PointStamped center_laser, center_base;
            center_laser.header.frame_id = laserFrame_;
            center_laser.header.stamp = cloudInMap_.header.stamp;
            center_laser.point.x = center.x;
            center_laser.point.y = center.y;
            center_laser.point.z = 0.0;
            try
            {
                tf_buffer_.transform(center_laser, center_base, baseFrame_, ros::Duration(0.1));
                double yaw = std::atan2(center_base.point.y, center_base.point.x);
                // ROS_INFO("base_link: center(%.2f, %.2f), yaw=%.1f°",
                //          center_base.point.x, center_base.point.y, yaw * 180.0 / M_PI);
                if (i == main_cluster_idx)
                {
                    ROS_INFO_STREAM("best of " << clusters.size() << " center("
                                               << center_base.point.x << ", " << center_base.point.y
                                               << "), yaw = " << yaw * 180.0 / M_PI << "°");
                    geometry_msgs::PointStamped target_point;
                    target_point.header.frame_id = baseFrame_;
                    target_point.header.stamp = cloudInMap_.header.stamp;
                    target_point.point.x = center_base.point.x;
                    target_point.point.y = center_base.point.y;
                    target_point.point.z = yaw; // 将 yaw 存储在 z 分量中
                    targetPub_.publish(target_point);
                    publishMarker(center, radius, marker_id, true);
                }
                else
                {
                    publishMarker(center, radius, marker_id, false);
                }
                marker_id++;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("TF failed: %s", ex.what());
            }
        }
    }

    // 直接构造最小外接圆
    void getTarget(const pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
        //  提取最小外接圆
        std::vector<cv::Point2f> points2D;
        for (const auto &pt : cloud.points)
            points2D.emplace_back(pt.x, pt.y);

        if (!points2D.empty())
        {
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(points2D, center, radius);
            // ROS_INFO("外接圆：x=%.3f y=%.3f  半径=%.3f", center.x, center.y, radius);
            // 可视化
            publishMarker(center, radius);

            // 圆心坐标在雷达系
            geometry_msgs::PointStamped center_in_laser;
            center_in_laser.header.frame_id = laserFrame_;
            center_in_laser.header.stamp = cloudInMap_.header.stamp;
            center_in_laser.point.x = center.x;
            center_in_laser.point.y = center.y;
            center_in_laser.point.z = 0.0;

            geometry_msgs::PointStamped center_in_base;
            try
            {
                tf_buffer_.transform(center_in_laser, center_in_base, baseFrame_, ros::Duration(0.1));

                double yaw = std::atan2(center_in_base.point.y, center_in_base.point.x);
                ROS_INFO("圆心在 base_link 系下坐标: [%.3f, %.3f], yaw: %.2f°",
                         center_in_base.point.x,
                         center_in_base.point.y,
                         yaw * 180.0 / M_PI);
                // 发布目标点
                geometry_msgs::PointStamped target_point;
                target_point.header.frame_id = baseFrame_;
                target_point.header.stamp = cloudInMap_.header.stamp;
                target_point.point.x = center_in_base.point.x;
                target_point.point.y = center_in_base.point.y;
                target_point.point.z = yaw; // 将 yaw 存储在 z 分量中
                targetPub_.publish(target_point);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("TF transform failed: %s", ex.what());
            }
        }
    }

    bool laserScanToCloudInMap(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        // LaserScan → PointCloud2
        laser_geometry::LaserProjection projector;
        sensor_msgs::PointCloud2 cloud_in_laser;
        projector.projectLaser(*msg, cloud_in_laser);

        // 点云从雷达坐标系 → map 坐标系
        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("map", laserFrame_, msg->header.stamp, ros::Duration(0.1));
            tf2::doTransform(cloud_in_laser, cloudInMap_, transform);
            // 发布 map 坐标系下的点云
            cloudInMap_.header.frame_id = "map";
            cloudInMap_.header.stamp = msg->header.stamp;
            mapCloudPub_.publish(cloudInMap_);

            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform to map failed: %s", ex.what());
            return false;
        }
    }

    bool getPointsInROI(pcl::PointCloud<pcl::PointXYZ> &roi_cloud_in_laser)
    {
        if (cloudInMap_.data.empty())
        {
            // ROS_WARN("No point cloud data in map.");
            return false;
        }
        // 将 PointCloud2 转为 PCL 格式
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud_map;
        pcl::fromROSMsg(cloudInMap_, pcl_cloud_map);

        pcl::PointCloud<pcl::PointXYZ> roi_cloud_in_map;

        // 遍历点云，筛选出在 ROI 多边形内的点
        if (useFixedROI_) // 使用固定区域roi
        {
            for (const auto &pt : pcl_cloud_map.points)
            {
                if (pointInROIPolygon(pt.x, pt.y))
                {
                    roi_cloud_in_map.points.push_back(pt);
                }
            }
        }
        else // 使用地图差分提取roi
        {
            for (const auto &pt : pcl_cloud_map.points)
            {
                if (!isPointInStaticMapWithTolerance(pt.x, pt.y, 0.1)) // 0.1米容忍
                {
                    roi_cloud_in_map.points.push_back(pt);
                }
            }
        }

        if (roi_cloud_in_map.points.empty())
        {
            // ROS_WARN("No points found in the ROI polygon.");
            return false;
        }

        // 点云从 map → 雷达坐标系
        sensor_msgs::PointCloud2 cloud_back_to_laser;
        try
        {
            geometry_msgs::TransformStamped transform_back = tf_buffer_.lookupTransform(laserFrame_, "map", cloudInMap_.header.stamp, ros::Duration(0.1));
            pcl_ros::transformPointCloud(roi_cloud_in_map, roi_cloud_in_laser, transform_back.transform);

            // 转回 ROS 消息并发布
            sensor_msgs::PointCloud2 roi_msg;
            pcl::toROSMsg(roi_cloud_in_laser, roi_msg);
            roi_msg.header.frame_id = laserFrame_;
            roi_msg.header.stamp = cloudInMap_.header.stamp;
            laserCloudROIPub_.publish(roi_msg);
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform back to laser failed: %s", ex.what());
            return false;
        }
    }

    void publishMarker(cv::Point2f center, double radius)
    {
        visualization_msgs::Marker circle_marker;
        circle_marker.header.frame_id = laserFrame_;
        circle_marker.header.stamp = cloudInMap_.header.stamp;
        circle_marker.ns = "aim_circle";
        circle_marker.id = 0;
        circle_marker.type = visualization_msgs::Marker::CYLINDER;
        circle_marker.action = visualization_msgs::Marker::ADD;
        circle_marker.pose.position.x = center.x;
        circle_marker.pose.position.y = center.y;
        circle_marker.pose.position.z = 0.0;
        circle_marker.pose.orientation.w = 1.0;
        circle_marker.scale.x = radius * 2;
        circle_marker.scale.y = radius * 2;
        circle_marker.scale.z = 0.01;
        circle_marker.color.a = 0.6;
        circle_marker.color.r = 0.0;
        circle_marker.color.g = 1.0;
        circle_marker.color.b = 0.0;
        markerPub_.publish(circle_marker);
    }

    void publishMarker(cv::Point2f center, double radius, int marker_id, bool is_main = false)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = laserFrame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "aim_circles";
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = center.x;
        marker.pose.position.y = center.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = radius * 2;
        marker.scale.y = radius * 2;
        marker.scale.z = 0.01;
        if (is_main)
        {
            marker.color.a = 0.6;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
        }
        marker.lifetime = ros::Duration(0.5);
        markerPub_.publish(marker);
    }

    // 从参数中读取roi多边形顶点
    void getROIPolygon()
    {
        XmlRpc::XmlRpcValue roi_list;
        if (private_nh_.getParam("roi_polygon", roi_list) && roi_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_INFO("Loaded roi_polygon parameter with %d vertices", roi_list.size());
            for (int i = 0; i < roi_list.size(); ++i)
            {
                if (roi_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray &&
                    roi_list[i].size() == 2 &&
                    roi_list[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
                    roi_list[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    geometry_msgs::Point pt;
                    pt.x = static_cast<double>(roi_list[i][0]);
                    pt.y = static_cast<double>(roi_list[i][1]);
                    pt.z = 0.0;
                    roiPolygon_.push_back(pt);
                    ROS_INFO("Added ROI vertex: (%.2f, %.2f)", pt.x, pt.y);
                }
                else
                {
                    ROS_WARN("Invalid ROI polygon vertex format at index %d", i);
                }
            }
        }
        else
        {
            ROS_WARN("Failed to load roi_polygon parameter or format incorrect. Use default polygon.");
            geometry_msgs::Point pt;
            pt.x = 0.09;
            pt.y = 0.09;
            roiPolygon_.push_back(pt);
            pt.x = 3.31;
            pt.y = 0.09;
            roiPolygon_.push_back(pt);
            pt.x = 3.31;
            pt.y = 3.31;
            roiPolygon_.push_back(pt);
            pt.x = 0.09;
            pt.y = 3.31;
            roiPolygon_.push_back(pt);
        }
    }

    // 判断点是否是在静态地图中的障碍物
    bool isPointInStaticMapWithTolerance(double x, double y, double tolerance = 0.1)
    {
        if (!static_map_)
            return false;

        int width = static_map_->info.width;
        int height = static_map_->info.height;
        double resolution = static_map_->info.resolution;
        double origin_x = static_map_->info.origin.position.x;
        double origin_y = static_map_->info.origin.position.y;

        int mx = static_cast<int>((x - origin_x) / resolution);
        int my = static_cast<int>((y - origin_y) / resolution);

        int range = static_cast<int>(std::ceil(tolerance / resolution)); // 例如 0.1m 容忍范围 → ±1 个格

        for (int dx = -range; dx <= range; ++dx)
        {
            for (int dy = -range; dy <= range; ++dy)
            {
                int nx = mx + dx;
                int ny = my + dy;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                    continue;

                int index = ny * width + nx;
                int value = static_map_->data[index];

                if (value > 50) // 被认为是占据障碍物
                    return true;
            }
        }

        return false; // 周围都不是障碍物
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        static_map_ = msg;
    }

    // 判断点是否在多边形内部（二维点）
    bool pointInROIPolygon(double x, double y)
    {
        int count = 0;
        size_t n = roiPolygon_.size();
        for (size_t i = 0; i < n; ++i)
        {
            const auto &p1 = roiPolygon_[i];
            const auto &p2 = roiPolygon_[(i + 1) % n];
            // 交点算法，射线法判断
            if (((p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y)) &&
                (x < (p2.x - p1.x) * (y - p1.y) / (p2.y - p1.y + 1e-6) + p1.x))
            {
                count++;
            }
        }
        return count % 2 == 1;
    }
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "laser_aimer_node");
    ros::NodeHandle nh;
    LaserAimer node(nh);
    ros::spin();

    return 0;
}