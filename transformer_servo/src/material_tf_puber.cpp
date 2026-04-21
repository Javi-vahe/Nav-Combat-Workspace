#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
class MaterialTFPuber
{
private:
    ros::NodeHandle &nh_;
    ros::NodeHandle private_nh_{"~"};

    tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    geometry_msgs::TransformStamped self_material_tf_[20], enemy_material_tf_[20];

    bool useSameHeight_ = true;    // 是否使用相同高度
    double material_height_ = 0.0; // 通用高度

public:
    MaterialTFPuber(ros::NodeHandle &nh) : nh_(nh)
    {
        initParams();
        publishTFs();
    }

    void initParams()
    {
        private_nh_.param<bool>("use_same_height", useSameHeight_, true);
        private_nh_.param<double>("material_height", material_height_, 0.0);
        for (int i = 0; i < sizeof(self_material_tf_) / sizeof(self_material_tf_[0]); ++i)
        {
            std::string self_material_id = "self_material_" + std::to_string(i + 1);
            private_nh_.param<double>(self_material_id + "_x", self_material_tf_[i].transform.translation.x, 0.0);
            private_nh_.param<double>(self_material_id + "_y", self_material_tf_[i].transform.translation.y, 0.0);
            private_nh_.param<double>(self_material_id + "_z", self_material_tf_[i].transform.translation.z, 0.0);
            private_nh_.param<double>(self_material_id + "_q_z", self_material_tf_[i].transform.rotation.z, 0.0);
            private_nh_.param<double>(self_material_id + "_q_w", self_material_tf_[i].transform.rotation.w, 1.0);
            if (useSameHeight_)
            {
                self_material_tf_[i].transform.translation.z = material_height_;
            }
        }
        for (int i = 0; i < sizeof(self_material_tf_) / sizeof(self_material_tf_[0]); ++i)
        {
            std::string enemy_material_id = "enemy_material_" + std::to_string(i + 1);
            private_nh_.param<double>(enemy_material_id + "_x", enemy_material_tf_[i].transform.translation.x, 0.0);
            private_nh_.param<double>(enemy_material_id + "_y", enemy_material_tf_[i].transform.translation.y, 0.0);
            private_nh_.param<double>(enemy_material_id + "_z", enemy_material_tf_[i].transform.translation.z, 0.0);
            private_nh_.param<double>(enemy_material_id + "_q_z", enemy_material_tf_[i].transform.rotation.z, 0.0);
            private_nh_.param<double>(enemy_material_id + "_q_w", enemy_material_tf_[i].transform.rotation.w, 1.0);
            if (useSameHeight_)
            {
                enemy_material_tf_[i].transform.translation.z = material_height_;
            }
        }
    }

    void publishTFs()
    {
        for (int i = 0; i < sizeof(self_material_tf_) / sizeof(self_material_tf_[0]); ++i)
        {
            self_material_tf_[i].header.frame_id = "map";
            self_material_tf_[i].child_frame_id = "self_material_" + std::to_string(i + 1);
            static_broadcaster_.sendTransform(self_material_tf_[i]);
        }
        for (int i = 0; i < sizeof(enemy_material_tf_) / sizeof(enemy_material_tf_[0]); ++i)
        {
            enemy_material_tf_[i].header.frame_id = "map";
            enemy_material_tf_[i].child_frame_id = "enemy_material_" + std::to_string(i + 1);
            static_broadcaster_.sendTransform(enemy_material_tf_[i]);
        }
    }
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "material_tf_puber");
    ros::NodeHandle nh;
    MaterialTFPuber node(nh);
    ros::spin();

    return 0;
}