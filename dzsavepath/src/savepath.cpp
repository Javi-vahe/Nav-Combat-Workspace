#include "savepath.h"
#include <filesystem>

savepath::savepath(ros::NodeHandle nh)
{

  receivePose_flag = false;
  nh.param("/dzsavepath/mappinmgsavePath", mappinmgsavePath, std::string("/home/duzhong/dzacs/src/resource/mapping.txt"));

  // 检查文件是否存在，并删除
  if (std::filesystem::exists(mappinmgsavePath))
  {
    std::filesystem::remove(mappinmgsavePath); // 删除文件
    std::cout << FRED("File exists and has been removed: ") << mappinmgsavePath << std::endl;
  }
  else
  {
    std::cout << FGRN("File does not exist: ") << mappinmgsavePath << std::endl;
  }

  std::cout << FRED("Copyright©2016-2020 duzhong robot. All rights reserved ") << std::endl;
  std::cout << FYEL("*****dzsavepath:parameters*******************") << std::endl;
  std::cout << FGRN("mappinmgsavePath: ") << mappinmgsavePath << std::endl;
  std::cout << FYEL("*****dzsavepath:parameters end***************") << std::endl;

}

savepath::~savepath()
{
}

void savepath::run()
{
  tf::TransformListener tf_listener;
  ros::Rate rate(20);
  // ros::Time current_time, last_time;
  while (ros::ok())
  {
    ros::spinOnce();
    tf::StampedTransform transform;
    try
    {
      // 获取从 map 到 odom 的变换
      tf_listener.lookupTransform("map", "base_link", ros::Time(0), transform);
      geometry_msgs::Pose current_position;
      // 设置位置
      current_position.position.x = transform.getOrigin().x();
      current_position.position.y = transform.getOrigin().y();
      current_position.position.z = transform.getOrigin().z();

      // 设置方向（四元数）
      tf::Quaternion quat = transform.getRotation();
      current_position.orientation.x = quat.x();
      current_position.orientation.y = quat.y();
      current_position.orientation.z = quat.z();
      current_position.orientation.w = quat.w();

      if (distance(current_position, last_position) >= 0.02)
      {
        fp = fopen(mappinmgsavePath.c_str(), "a");
        fprintf(fp, "%.4lf,%.4lf,%.4lf,%.4lf,\n",
                current_position.position.x,
                current_position.position.y,
                current_position.orientation.z,
                current_position.orientation.w);
        fclose(fp);
        last_position = current_position;
      }

      
    }
    catch (tf::TransformException &ex)
    {
      // ROS_WARN("Failed to get transform: %s", ex.what());
    }

    rate.sleep();
  }
}

double savepath::distance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
{
  return hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y);
}
