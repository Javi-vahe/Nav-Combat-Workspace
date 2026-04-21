#include <ros/ros.h>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

class MaterialProcessor
{
private:
    ros::NodeHandle &nh_;
    ros::NodeHandle private_nh_{"~"};

    ros::Publisher sorted_material_pub_;      // 发布排序好的物资顺序
    ros::Publisher closest_material_pub_;     // 发布最近的物资序号
    ros::Subscriber all_Material_Number_sub_; // 订阅裁判系统发布的所有物资编号
    ros::Subscriber remove_material_sub_;     // 订阅被移除的物资编号

    // 定义从近到远的数字顺序
    // 红方
    const std::vector<uint8_t> red_location_order = {
        17, 5, 3, 4, 8, 18, 20, 1, 19, 13, 16, 12, 14, 9, 7, 6, 2, 15, 11, 10,
        111, 115, 114, 112, 110, 106, 103, 105, 102, 109, 107, 113, 119, 116, 118, 108, 104, 101, 117, 120};
    // 蓝方
    const std::vector<uint8_t> blue_location_order = {
        117, 105, 103, 104, 108, 118, 120, 101, 119, 113, 116, 112, 114, 109, 107, 106, 102, 115, 111, 110,
        11, 15, 14, 12, 10, 6, 3, 5, 2, 9, 7, 13, 19, 16, 18, 8, 4, 1, 17, 20};

    std::unordered_map<uint8_t, uint8_t> priority_map_; // 排序顺序
    int self_team_ = 0;                                 // 0: 红方, 1: 蓝方

    std_msgs::UInt8MultiArray all_material_;
    std::vector<uint8_t> sorted_locations_;

    // std::unordered_set<uint8_t> removed_locations_;      // 手动清除的物资+裁判系统清除的物资
    // std::unordered_set<uint8_t> real_removed_locations_; // 裁判系统清除的物资

public:
    MaterialProcessor(ros::NodeHandle &nh) : nh_(nh)
    {
        private_nh_.param("self_team", self_team_, 0);
        if (self_team_ == 0)
        {
            ROS_INFO("当前机器人为红方 red red red");
            priority_map_ = createPriorityMap(red_location_order);
        }
        else
        {
            ROS_INFO("当前机器人为蓝方 blue blue blue");
            priority_map_ = createPriorityMap(blue_location_order);
        }

        closest_material_pub_ = nh.advertise<std_msgs::UInt8>("closest_material", 1);
        sorted_material_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("sorted_material", 1);
        all_Material_Number_sub_ = nh.subscribe("all_Material_Number", 1, &MaterialProcessor::allMaterialNumberCB,
                                                this, ros::TransportHints().tcpNoDelay());
        remove_material_sub_ = nh.subscribe("remove_material", 1, &MaterialProcessor::removeMaterialCB,
                                            this, ros::TransportHints().tcpNoDelay());
    }

    void allMaterialNumberCB(const std_msgs::UInt8MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() == 8)
        {
            if (all_material_ != *msg)
            {
                all_material_ = *msg; // 接收到裁判系统发布的所有物资编号

                std::vector<uint8_t> received_locations;
                for (uint8_t number : msg->data)
                {
                    received_locations.push_back(number);
                }
                if (!received_locations.empty())
                {
                    std::cout << "接收到的原始数据：";
                    for (uint8_t loc : received_locations)
                    {
                        std::cout << static_cast<int>(loc) << " ";
                    }
                    std::cout << std::endl;

                    // 排序
                    sortLocations(received_locations);

                    std::cout << "排序后的地点编号（从近到远）：";
                    for (uint8_t loc : received_locations)
                    {
                        std::cout << static_cast<int>(loc) << " ";
                    }
                    std::cout << std::endl;
                }
                sorted_locations_ = received_locations;
                if (self_team_ != 0) // 蓝方需要互换编号
                {
                    for (uint8_t &loc : sorted_locations_)
                    {
                        if (loc < 100)
                            loc += 100;
                        else
                            loc -= 100;
                    }
                    std::cout << "蓝方互换后的地点编号（从近到远）：";
                    for (uint8_t loc : sorted_locations_)
                    {
                        std::cout << static_cast<int>(loc) << " ";
                    }
                    std::cout << std::endl;
                }
            }
        }
        else // 有物资被识别
        {
            // 创建排序后的副本
            std::vector<uint8_t> all_sorted = all_material_.data; // 已排序或复制后排序
            std::vector<uint8_t> msg_sorted = msg->data;
            std::sort(all_sorted.begin(), all_sorted.end());
            std::sort(msg_sorted.begin(), msg_sorted.end());
            // 存储缺少的元素
            std::vector<uint8_t> missing;
            missing.reserve(all_sorted.size()); // 预分配空间

            // 计算 set difference: all_sorted - msg_sorted
            std::set_difference(
                all_sorted.begin(), all_sorted.end(),
                msg_sorted.begin(), msg_sorted.end(),
                std::back_inserter(missing));
            // 遍历 missing，为每个值创建 std_msgs::UInt8 消息并调用 removeMaterialCB
            for (const auto &material_id : missing)
            {
                auto single_msg = boost::make_shared<std_msgs::UInt8>();
                single_msg->data = material_id;
                if (self_team_ != 0) // 蓝方需要互换编号
                {
                    if (single_msg->data < 100)
                        single_msg->data += 100;
                    else
                        single_msg->data -= 100;
                }
                // real_removed_locations_.insert(single_msg->data); // 记录被裁判系统移除的物资编号,使用 set 来避免重复
                removeMaterialCB(single_msg);
            }
        }

        publishSortedLocations();
    }

    void removeMaterialCB(const std_msgs::UInt8::ConstPtr msg)
    {
        uint8_t removed_location = 0;
        // static uint8_t removed_location = 0;
        // if (msg->data == removed_location)
        // {
        //     return; // 如果是重复的移除请求，直接返回
        // }
        removed_location = msg->data;
        // std::cout << "接收到被移除的物资编号：" << static_cast<int>(removed_location) << std::endl;

        // 从排序后的列表中移除被移除的物资
        sorted_locations_.erase(std::remove(sorted_locations_.begin(), sorted_locations_.end(), removed_location),
                                sorted_locations_.end());

        // 记录被移除的物资编号
        // removed_locations_.insert(removed_location); // 使用 set 来避免重复

        // 重新发布排序后的点位
        publishSortedLocations();
    }

    void publishSortedLocations()
    {
        // 发布最近的点位,若无点位则会发布0
        std_msgs::UInt8 closest_msg;
        if (!sorted_locations_.empty())
        {
            closest_msg.data = sorted_locations_.front();
        }
        else
            closest_msg.data = static_cast<uint8_t>(0);
        closest_material_pub_.publish(closest_msg);

        // 发布排序后的点位,若无点位则发布空vector
        std_msgs::UInt8MultiArray sorted_msg;
        for (uint8_t sort_num : sorted_locations_)
        {
            sorted_msg.data.push_back(sort_num);
        }
        sorted_material_pub_.publish(sorted_msg);
    }

    // 创建一个用于快速查找顺序的映射表
    std::unordered_map<uint8_t, uint8_t> createPriorityMap(const std::vector<uint8_t> &order)
    {
        std::unordered_map<uint8_t, uint8_t> map;
        for (size_t i = 0; i < order.size(); ++i)
        {
            map[order[i]] = static_cast<uint8_t>(i);
        }
        return map;
    }

    // 排序函数：按照预定义顺序排序
    void sortLocations(std::vector<uint8_t> &input_locations)
    {
        std::sort(input_locations.begin(), input_locations.end(),
                  [&](uint8_t a, uint8_t b)
                  {
                      uint8_t pa = priority_map_.count(a) ? priority_map_.at(a) : 1000;
                      uint8_t pb = priority_map_.count(b) ? priority_map_.at(b) : 1000;
                      return pa < pb;
                  });
    }
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "custom_sort_node");
    ros::NodeHandle nh;
    MaterialProcessor node(nh);
    ros::spin();
    return 0;
}
