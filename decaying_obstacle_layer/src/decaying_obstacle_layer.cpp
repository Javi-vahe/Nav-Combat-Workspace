#include<decaying_obstacle_layer/decaying_obstacle_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(decaying_layer::DecayingObstacleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace decaying_layer
{

DecayingObstacleLayer::DecayingObstacleLayer() {}

void DecayingObstacleLayer::onInitialize()
{
  ObstacleLayer::onInitialize();
  ros::NodeHandle nh("~/" + name_);
  double decay_time;
  nh.param("decay_time", decay_time, 5.0);
  std::cout << "decay_time: " << decay_time << std::endl;
  std::cout << "decay_time: " << decay_time << std::endl;
  std::cout << "decay_time: " << decay_time << std::endl;
  std::cout << "decay_time: " << decay_time << std::endl;
  std::cout << "decay_time: " << decay_time << std::endl;
  timeout = ros::Duration(decay_time);
  current_ = true;
}

using costmap_2d::Observation;

void DecayingObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  //get the marking observations
  current = current && getMarkingObservations(observations);

  //get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  //update the global current status
  current_ = current;
  
  TimeCoord tc(ros::Time::now());

  //raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  //place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;

    // const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*(obs.cloud_), cloud);

    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    for (unsigned int i = 0; i < cloud.points.size(); ++i)
    {
      double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

      //if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_)
      {
        ROS_DEBUG("The point is too high");
        continue;
      }

      //compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
          + (pz - obs.origin_.z) * (pz - obs.origin_.z);

      //if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range)
      {
        ROS_DEBUG("The point is too far away");
        continue;
      }

      //now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my))
      {
        ROS_DEBUG("Computing map coords failed");
        continue;
      }

      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      tc.coord = index;
      tc.x = px;
      tc.y = py;
      pq.push(tc);
      status_map[index] = tc.time;
      
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }
  
  while( pq.size()>0 && pq.top().time + timeout < tc.time ){
      TimeCoord x = pq.top();
      pq.pop();
      int index = x.coord;
      if( costmap_[index]==LETHAL_OBSTACLE && status_map[index]==x.time){
        costmap_[index]=costmap_2d::NO_INFORMATION;
        touch(x.x,x.y,min_x,min_y,max_x,max_y);
      }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

} // end namespace
