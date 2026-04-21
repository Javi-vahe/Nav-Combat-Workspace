#ifndef DECAYING_LAYER_H_
#define DECAYING_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace decaying_layer
{

class TimeCoord
{
public:
    TimeCoord(ros::Time t): time(t), x(0.0), y(0.0), coord(-1) {}
    ros::Time time;
    double x,y;
    int coord;
};


class Compare
{
public:
    bool operator() (TimeCoord a, TimeCoord b)
    {
        return a.time > b.time;
    }
};


class DecayingObstacleLayer : public costmap_2d::ObstacleLayer
{
public:
  DecayingObstacleLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);

private:
  std::priority_queue<TimeCoord, std::vector<TimeCoord>, Compare > pq;
  std::map<int, ros::Time> status_map;
  ros::Duration timeout;
  
};
}
#endif
