#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PointStamped.h>

// 定义回调函数，当两个话题的消息在时间上接近时会被调用
void callback(const geometry_msgs::PointStampedConstPtr& target_point,
              const geometry_msgs::PointStampedConstPtr& points)
{
    // ROS_INFO("收到同步消息：");
    // ROS_INFO("target_point_laser: [%f, %f, %f] at time %.3f",
    //          target_point->point.x, target_point->point.y, target_point->point.z,
    //          target_point->header.stamp.toSec());
    // ROS_INFO("points: [%f, %f, %f] at time %.3f",
    //          points->point.x, points->point.y, points->point.z,
    //          points->header.stamp.toSec());

    // 在这里可以加入你需要的处理逻辑
    ROS_INFO("laser:%.2f度\tvision:%d pixels", target_point->point.z / M_PI * 180.0, int(points->point.x));
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "point_sync_node");
    ros::NodeHandle nh;

    // 创建两个 message_filter 的 Subscriber
    message_filters::Subscriber<geometry_msgs::PointStamped> sub1(nh, "/target_point_laser", 10);
    message_filters::Subscriber<geometry_msgs::PointStamped> sub2(nh, "/points", 10);

    // 使用 ApproximateTime 策略进行同步（允许一定时间偏差）
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::PointStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);

    // 注册回调函数
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ROS_INFO("开始同步两个话题...");
    ros::spin();

    return 0;
}