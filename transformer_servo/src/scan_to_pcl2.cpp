#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

class ScanToCloud {
public:
    ScanToCloud() {
        // 订阅激光雷达数据
        scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
            "/scan", 10, &ScanToCloud::scanCallback, this);

        // 发布点云数据
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/scan_pointcloud2", 10);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
        // 转换为PointCloud2
        sensor_msgs::PointCloud2 cloud;
        projector_.transformLaserScanToPointCloud(scan_in->header.frame_id, *scan_in, cloud, tfListener_);

        // 设置新的frame_id（可选）
        cloud.header.frame_id = scan_in->header.frame_id;

        // 发布点云
        cloud_pub_.publish(cloud);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_to_pointcloud");

    ScanToCloud converter;

    ros::spin();

    return 0;
}