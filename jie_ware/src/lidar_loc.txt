#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <jie_ware/simpleKalmanFilter.hpp>
#include <jie_ware/SimpleKalmanParamsConfig.h>

class LidarLocalization
{
private:
    ros::NodeHandle &nh_;
    ros::NodeHandle private_nh_{"~"};

    ros::Subscriber mapSub_;
    ros::Subscriber scanSub_;
    ros::Subscriber inittialPoseSub_;
    ros::Publisher resultOriPub_, resultKFPub_;
    ros::ServiceClient clearCostmapsClient_;
    ros::Timer pubTFTimer;

    nav_msgs::OccupancyGrid mapMsg_;
    cv::Mat mapCropped_;
    cv::Mat mapTemp_;
    sensor_msgs::RegionOfInterest mapRoiInfo_;
    std::vector<cv::Point2f> scanPoints_;

    std::string base_frame;
    std::string odom_frame;
    std::string laser_frame;
    std::string laser_topic;

    float lidar_x_ = 250, lidar_y_ = 250, lidar_yaw_ = 0;
    float lidar_x_tf_ = 250, lidar_y_tf_ = 250, lidar_yaw_tf_ = 0;
    float deg_to_rad_ = M_PI / 180.0;
    int clearCountdown_ = -1;
    int scanCount_ = 0;

    dynamic_reconfigure::Server<jie_ware::SimpleKalmanParamsConfig> paramServer_;
    dynamic_reconfigure::Server<jie_ware::SimpleKalmanParamsConfig>::CallbackType param_f_;

    SimpleKalmanFilter kf_x_{0.001, 0.01, 0.0}; // 使用花括号
    SimpleKalmanFilter kf_y_{0.001, 0.01, 0.0};
    SimpleKalmanFilter kf_yaw_{0.001, 0.01, 0.0};
    bool kfInitialized_ = false;
    bool pubKFResult_ = false; // 发布卡尔曼滤波结果
    bool useKF_ = true;        // 是否使用卡尔曼滤波

    geometry_msgs::TransformStamped odom_to_base_;

public:
    LidarLocalization(ros::NodeHandle &nh) : nh_(nh)
    {
        // 读取参数
        private_nh_.param<std::string>("base_frame", base_frame, "base_footprint");
        private_nh_.param<std::string>("odom_frame", odom_frame, "odom");
        private_nh_.param<std::string>("laser_frame", laser_frame, "laser_link");
        private_nh_.param<std::string>("laser_topic", laser_topic, "scan");
        double freq;
        private_nh_.param<double>("freq", freq, 30.0);
        private_nh_.param<bool>("use_kf", useKF_, true);
        if (useKF_)
        {
            private_nh_.param<bool>("pub_kf_result", pubKFResult_, false);
            double Q_x, R_x;
            double Q_y, R_y;
            double Q_yaw, R_yaw;
            private_nh_.param<double>("Q_x", Q_x, 0.001);
            private_nh_.param<double>("R_x", R_x, 0.01);
            private_nh_.param<double>("Q_y", Q_y, 0.001);
            private_nh_.param<double>("R_y", R_y, 0.01);
            private_nh_.param<double>("Q_yaw", Q_yaw, 0.001);
            private_nh_.param<double>("R_yaw", R_yaw, 0.01);

            std::cout << "freq:" << freq << std::endl;
            std::cout << "pub_kf_result:" << pubKFResult_ << std::endl;
            std::cout << "Q_x:" << Q_x << std::endl;
            std::cout << "R_x:" << R_x << std::endl;
            std::cout << "Q_y:" << Q_y << std::endl;
            std::cout << "R_y:" << R_y << std::endl;
            std::cout << "Q_yaw:" << Q_yaw << std::endl;
            std::cout << "R_yaw:" << R_yaw << std::endl;

            kf_x_.setQandR(Q_x, R_x);
            kf_y_.setQandR(Q_y, R_y);
            kf_yaw_.setQandR(Q_yaw, R_yaw);
        }

        // 初始化订阅和发布
        mapSub_ = nh_.subscribe("/map", 1, &LidarLocalization::mapCB, this);
        scanSub_ = nh_.subscribe(laser_topic, 1, &LidarLocalization::scanCB, this, ros::TransportHints().tcpNoDelay());
        inittialPoseSub_ = nh_.subscribe("/initialpose", 1, &LidarLocalization::initialPoseCB, this);
        if (pubKFResult_)
        {
            resultOriPub_ = nh_.advertise<geometry_msgs::Pose2D>("/lidar_loc/origin", 1);
            resultKFPub_ = nh_.advertise<geometry_msgs::Pose2D>("/lidar_loc/kf", 1);
        }
        // 初始化服务客户端
        clearCostmapsClient_ = nh_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");

        pubTFTimer = nh_.createTimer(ros::Duration(1.0 / freq), &LidarLocalization::poseTF, this);

        param_f_ = boost::bind(&LidarLocalization::paramCB, this, _1, _2);
        paramServer_.setCallback(param_f_);
    }

    void paramCB(jie_ware::SimpleKalmanParamsConfig &config, uint32_t level)
    {
        std::cout << "Q_x:" << config.Q_x << "\tR_x:" << config.R_x << std::endl;
        std::cout << "Q_y:" << config.Q_y << "\tR_y:" << config.R_y << std::endl;
        std::cout << "Q_yaw:" << config.Q_yaw << "\tR_yaw:" << config.R_yaw << std::endl;
        kf_x_.setQandR(config.Q_x, config.R_x);
        kf_y_.setQandR(config.Q_y, config.R_y);
        kf_yaw_.setQandR(config.Q_yaw, config.R_yaw);
    }

    void poseTF(const ros::TimerEvent &)
    {
        if (scanCount_ == 0)
            return;
        if (mapCropped_.empty() || mapMsg_.data.empty())
            return;

        // 1. 计算在裁剪地图中的实际米制坐标
        double x_meters = (lidar_x_tf_ + mapRoiInfo_.x_offset) * mapMsg_.info.resolution;
        double y_meters = (lidar_y_tf_ + mapRoiInfo_.y_offset) * mapMsg_.info.resolution;

        // 2. 考虑原始地图的原点偏移
        x_meters += mapMsg_.info.origin.position.x;
        y_meters = y_meters + mapMsg_.info.origin.position.y;

        // 3. 处理yaw角度
        // double yaw_ros = -lidar_yaw_;
        double yaw_ros = -lidar_yaw_tf_;

        // 4. 将弧度转换为四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_ros);

        // 5. 计算 base_footprint 在 map 中的位置
        double base_x = x_meters;
        double base_y = y_meters;

        // 6. 查询 odom 到 base_frame 的变换
        geometry_msgs::TransformStamped &odom_to_base = odom_to_base_;

        // 7. 计算 map 到 odom 的变换
        tf2::Transform map_to_base, odom_to_base_tf2;
        map_to_base.setOrigin(tf2::Vector3(base_x, base_y, 0));
        map_to_base.setRotation(q);

        tf2::fromMsg(odom_to_base.transform, odom_to_base_tf2);
        tf2::Transform map_to_odom = map_to_base * odom_to_base_tf2.inverse();

        // 8. 发布 map 到 odom 的变换
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped map_to_odom_msg;

        map_to_odom_msg.header.stamp = ros::Time::now();
        map_to_odom_msg.header.frame_id = "map";
        map_to_odom_msg.child_frame_id = odom_frame;
        map_to_odom_msg.transform = tf2::toMsg(map_to_odom);

        if (!useKF_)
        {
            // 如果不使用卡尔曼滤波，直接发布变换
            br.sendTransform(map_to_odom_msg);
            return;
        }

        // 计算 yaw 角度
        tf2::Quaternion q_tf2(
            map_to_odom_msg.transform.rotation.x,
            map_to_odom_msg.transform.rotation.y,
            map_to_odom_msg.transform.rotation.z,
            map_to_odom_msg.transform.rotation.w);
        tf2::Matrix3x3 m(q_tf2);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // kf
        if (!kfInitialized_)
        {
            // 第一次收到数据时用真实值初始化滤波器
            kf_x_.reset(map_to_odom_msg.transform.translation.x);
            kf_y_.reset(map_to_odom_msg.transform.translation.y);
            kf_yaw_.reset(yaw);
            kfInitialized_ = true;
        }

        // 滤波处理
        double filtered_x = kf_x_.update(map_to_odom_msg.transform.translation.x);
        double filtered_y = kf_y_.update(map_to_odom_msg.transform.translation.y);
        double filtered_yaw = kf_yaw_.update(yaw);

        tf2::Quaternion filtered_q;
        filtered_q.setRPY(0, 0, filtered_yaw);

        if (pubKFResult_) // kf可视化
        {
            geometry_msgs::Pose2D result_to_pub_msg;
            result_to_pub_msg.x = map_to_odom_msg.transform.translation.x;
            result_to_pub_msg.y = map_to_odom_msg.transform.translation.y;
            result_to_pub_msg.theta = yaw;
            resultOriPub_.publish(result_to_pub_msg);
            result_to_pub_msg.x = filtered_x;
            result_to_pub_msg.y = filtered_y;
            result_to_pub_msg.theta = filtered_yaw;
            resultKFPub_.publish(result_to_pub_msg);
        }

        map_to_odom_msg.transform.translation.x = filtered_x;
        map_to_odom_msg.transform.translation.y = filtered_y;
        map_to_odom_msg.transform.rotation = tf2::toMsg(filtered_q);
        br.sendTransform(map_to_odom_msg);
    }

    void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        static tf2_ros::Buffer tfBuffer;
        static tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped odom_to_base;
        try
        {
            // 获取雷达帧时刻的变换
            odom_to_base = tfBuffer.lookupTransform(odom_frame, laser_frame, msg->header.stamp, ros::Duration(1.0)); // 最后一个参数是等待超时时间
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        scanPoints_.clear();
        double angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max)
            {
                float x = msg->ranges[i] * cos(angle) / mapMsg_.info.resolution;
                float y = -msg->ranges[i] * sin(angle) / mapMsg_.info.resolution;
                scanPoints_.push_back(cv::Point2f(x, y));
            }
            angle += msg->angle_increment;
        }
        if (scanCount_ == 0)
            scanCount_++;

        while (ros::ok())
        {
            if (!mapCropped_.empty())
            {
                // 计算三种情况下的雷达点坐标数组
                std::vector<cv::Point2f> transform_points, clockwise_points, counter_points;

                int max_sum = 0;
                float best_dx = 0, best_dy = 0, best_dyaw = 0;

                for (const auto &point : scanPoints_)
                {
                    // 情况一：原始角度
                    float rotated_x = point.x * cos(lidar_yaw_) - point.y * sin(lidar_yaw_);
                    float rotated_y = point.x * sin(lidar_yaw_) + point.y * cos(lidar_yaw_);
                    transform_points.push_back(cv::Point2f(rotated_x + lidar_x_, lidar_y_ - rotated_y));

                    // 情况二：顺时针旋转1度
                    float clockwise_yaw = lidar_yaw_ + deg_to_rad_;
                    rotated_x = point.x * cos(clockwise_yaw) - point.y * sin(clockwise_yaw);
                    rotated_y = point.x * sin(clockwise_yaw) + point.y * cos(clockwise_yaw);
                    clockwise_points.push_back(cv::Point2f(rotated_x + lidar_x_, lidar_y_ - rotated_y));

                    // 情况三：逆时针旋转1度
                    float counter_yaw = lidar_yaw_ - deg_to_rad_;
                    rotated_x = point.x * cos(counter_yaw) - point.y * sin(counter_yaw);
                    rotated_y = point.x * sin(counter_yaw) + point.y * cos(counter_yaw);
                    counter_points.push_back(cv::Point2f(rotated_x + lidar_x_, lidar_y_ - rotated_y));
                }

                // 计算15种变换方式的匹配值
                std::vector<cv::Point2f> offsets = {{0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}};
                std::vector<std::vector<cv::Point2f>> point_sets = {transform_points, clockwise_points, counter_points};
                std::vector<float> yaw_offsets = {0, deg_to_rad_, -deg_to_rad_};

                for (int i = 0; i < offsets.size(); ++i)
                {
                    for (int j = 0; j < point_sets.size(); ++j)
                    {
                        int sum = 0;
                        for (const auto &point : point_sets[j])
                        {
                            float px = point.x + offsets[i].x;
                            float py = point.y + offsets[i].y;
                            if (px >= 0 && px < mapTemp_.cols && py >= 0 && py < mapTemp_.rows)
                            {
                                sum += mapTemp_.at<uchar>(py, px);
                            }
                        }
                        if (sum > max_sum)
                        {
                            max_sum = sum;
                            best_dx = offsets[i].x;
                            best_dy = offsets[i].y;
                            best_dyaw = yaw_offsets[j];
                        }
                        // std::cout << "offsets[" << i << "] point_sets[" << j << "]  :sum: " << sum << std::endl;
                    }
                }

                // 更新雷达位置和角度
                lidar_x_ += best_dx;
                lidar_y_ += best_dy;
                lidar_yaw_ += best_dyaw;

                // 判断匹配循环是否可以终止
                if (check(lidar_x_, lidar_y_, lidar_yaw_))
                {
                    lidar_x_tf_ = lidar_x_;
                    lidar_y_tf_ = lidar_y_;
                    lidar_yaw_tf_ = lidar_yaw_;
                    odom_to_base_ = odom_to_base; // 更新 全局odom_to_base_
                    break;
                }
            }
            else
            {
                break;
            }
        }

        if (clearCountdown_ > -1)
            clearCountdown_--;
        if (clearCountdown_ == 0)
        {
            std_srvs::Empty srv;
            clearCostmapsClient_.call(srv);
            // clearCountdown_ = 15;
        }
    }
    bool check(float x, float y, float yaw)
    {
        static std::deque<std::tuple<float, float, float>> data_queue; // for func check
        static const size_t max_size = 10;                             // for func check

        if (x == 0 && y == 0 && yaw == 0)
        {
            data_queue.clear();
            return true;
        }

        // 添加新数据
        data_queue.push_back(std::make_tuple(x, y, yaw));

        // 如果队列超过最大大小，移除最旧的数据
        if (data_queue.size() > max_size)
        {
            data_queue.pop_front();
        }

        // 如果队列已满，检查第一个和最后一个元素
        if (data_queue.size() == max_size)
        {
            auto &first = data_queue.front();
            auto &last = data_queue.back();

            float dx = std::abs(std::get<0>(last) - std::get<0>(first));
            float dy = std::abs(std::get<1>(last) - std::get<1>(first));
            float dyaw = std::abs(std::get<2>(last) - std::get<2>(first));

            // 如果所有差值的绝对值都小于5，清空队列退出循环
            if (dx < 1 && dy < 1 && dyaw < 1 * deg_to_rad_)
            {
                data_queue.clear();
                return true;
            }
        }
        return false;
    }

    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        mapMsg_ = *msg;
        cropMap();
        processMap();
        // 可视化处理后的地图
        // cv::imshow("Processed Map", mapTemp_);
        // cv::waitKey();
    }
    void cropMap()
    {
        // 显示地图信息
        std_msgs::Header header = mapMsg_.header;
        nav_msgs::MapMetaData info = mapMsg_.info;

        // 用来统计地图有效区域的变量
        int xMax, xMin, yMax, yMin;
        xMax = xMin = info.width / 2;
        yMax = yMin = info.height / 2;
        bool bFirstPoint = true;

        // 把地图数据转换成图片
        cv::Mat map_raw(info.height, info.width, CV_8UC1, cv::Scalar(128)); // 灰色背景

        for (int y = 0; y < info.height; y++)
        {
            for (int x = 0; x < info.width; x++)
            {
                int index = y * info.width + x;

                // 直接使用map_msg.data中的值
                map_raw.at<uchar>(y, x) = static_cast<uchar>(mapMsg_.data[index]);

                // 统计有效区域
                if (mapMsg_.data[index] == 100)
                {
                    if (bFirstPoint)
                    {
                        xMax = xMin = x;
                        yMax = yMin = y;
                        bFirstPoint = false;
                        continue;
                    }
                    xMin = std::min(xMin, x);
                    xMax = std::max(xMax, x);
                    yMin = std::min(yMin, y);
                    yMax = std::max(yMax, y);
                }
            }
        }
        // 计算有效区域的中心点坐标
        int cen_x = (xMin + xMax) / 2;
        int cen_y = (yMin + yMax) / 2;

        // 按照有效区域对地图进行裁剪
        int new_half_width = abs(xMax - xMin) / 2 + 50;
        int new_half_height = abs(yMax - yMin) / 2 + 50;
        int new_origin_x = cen_x - new_half_width;
        int new_origin_y = cen_y - new_half_height;
        int new_width = new_half_width * 2;
        int new_height = new_half_height * 2;

        if (new_origin_x < 0)
            new_origin_x = 0;
        if ((new_origin_x + new_width) > info.width)
            new_width = info.width - new_origin_x;
        if (new_origin_y < 0)
            new_origin_y = 0;
        if ((new_origin_y + new_height) > info.height)
            new_height = info.height - new_origin_y;

        cv::Rect roi(new_origin_x, new_origin_y, new_width, new_height);
        cv::Mat roi_map = map_raw(roi).clone();

        // 可视化原始地图
        // cv::imshow("Original Map", map_raw);
        // cv::waitKey();
        // cv::imshow("roi Map(mapCropped_)", roi_map);
        // cv::waitKey();

        mapCropped_ = roi_map;

        // 地图的裁减信息
        mapRoiInfo_.x_offset = new_origin_x;
        mapRoiInfo_.y_offset = new_origin_y;
        mapRoiInfo_.width = new_width;
        mapRoiInfo_.height = new_height;

        geometry_msgs::PoseWithCovarianceStamped init_pose;
        init_pose.pose.pose.position.x = 0.0;
        init_pose.pose.pose.position.y = 0.0;
        init_pose.pose.pose.position.y = 0.0;
        init_pose.pose.pose.orientation.x = 0.0;
        init_pose.pose.pose.orientation.y = 0.0;
        init_pose.pose.pose.orientation.z = 0.0;
        init_pose.pose.pose.orientation.w = 1.0;

        geometry_msgs::PoseWithCovarianceStamped::ConstPtr init_pose_ptr(new geometry_msgs::PoseWithCovarianceStamped(init_pose));
        initialPoseCB(init_pose_ptr);
    }
    void processMap()
    {
        if (mapCropped_.empty())
            return;

        mapTemp_ = cv::Mat::zeros(mapCropped_.size(), CV_8UC1);
        cv::Mat gradient_mask = createGradientMask(101); // 创建一个101x101的渐变掩模
        // cv::imshow("Gradient Mask", gradient_mask);
        // cv::waitKey();

        for (int y = 0; y < mapCropped_.rows; y++)
        {
            for (int x = 0; x < mapCropped_.cols; x++)
            {
                if (mapCropped_.at<uchar>(y, x) == 100) // 障碍物
                {
                    int left = std::max(0, x - 50);
                    int top = std::max(0, y - 50);
                    int right = std::min(mapCropped_.cols - 1, x + 50);
                    int bottom = std::min(mapCropped_.rows - 1, y + 50);

                    cv::Rect roi(left, top, right - left + 1, bottom - top + 1);
                    cv::Mat region = mapTemp_(roi);

                    int mask_left = 50 - (x - left);
                    int mask_top = 50 - (y - top);
                    cv::Rect mask_roi(mask_left, mask_top, roi.width, roi.height);
                    cv::Mat mask = gradient_mask(mask_roi);

                    cv::max(region, mask, region);
                }
            }
        }
        // // 可视化裁剪后的地图
        // cv::imshow("Cropped Map", mapCropped_);
        // cv::waitKey();
    }

    cv::Mat createGradientMask(int size)
    {
        cv::Mat mask(size, size, CV_8UC1);
        int center = size / 2;
        for (int y = 0; y < size; y++)
        {
            for (int x = 0; x < size; x++)
            {
                double distance = std::hypot(x - center, y - center);
                int value = cv::saturate_cast<uchar>(255 * std::max(0.0, 1.0 - distance / center));
                mask.at<uchar>(y, x) = value;
            }
        }
        return mask;
    }

    void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        // 1. 直接从输入消息中提取 map 坐标系下的位置和方向信息
        double map_x = msg->pose.pose.position.x;
        double map_y = msg->pose.pose.position.y;
        tf2::Quaternion q;
        // 使用 tf2::fromMsg 将 geometry_msgs::Quaternion 转换为 tf2::Quaternion
        tf2::fromMsg(msg->pose.pose.orientation, q);

        // 2. 将四元数转换为 yaw 角度 (相对于 map 坐标系)
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 3. 检查 mapMsg_ 是否有效
        if (mapMsg_.info.resolution <= 0)
        {
            ROS_ERROR("地图信息无效或未接收");
            return;
        }

        // 4. 将地图坐标转换为裁切后的地图栅格坐标
        lidar_x_ = (map_x - mapMsg_.info.origin.position.x) / mapMsg_.info.resolution - mapRoiInfo_.x_offset;
        lidar_y_ = (map_y - mapMsg_.info.origin.position.y) / mapMsg_.info.resolution - mapRoiInfo_.y_offset;

        // 5. 设置 yaw 角度
        lidar_yaw_ = -yaw;

        // 6. 设置倒计时
        clearCountdown_ = 30;

        // reset ekf
        kfInitialized_ = false;
    }
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "lidar_loc");
    ros::NodeHandle nh;
    LidarLocalization node(nh);
    ros::spin();

    return 0;
}