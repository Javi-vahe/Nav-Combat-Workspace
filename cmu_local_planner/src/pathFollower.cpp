#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;

nav_msgs::Path path;

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = odomIn->header.stamp.toSec();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = odomIn->header.stamp.toSec();
  }
}

nav_msgs::Path originPath_;
void updatePathFrame()
{
  int pathSize = originPath_.poses.size();
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.waitForTransform("base_link", originPath_.header.frame_id, ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("base_link", originPath_.header.frame_id, ros::Time(0), transform);
  } catch (tf::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  for (int i = 0; i < pathSize; i++) {
    tf::Vector3 point(originPath_.poses[i].pose.position.x, originPath_.poses[i].pose.position.y, originPath_.poses[i].pose.position.z);
    tf::Vector3 transformedPoint = transform * point;
    path.poses[i].pose.position.x = transformedPoint.x();
    path.poses[i].pose.position.y = transformedPoint.y();
    path.poses[i].pose.position.z = transformedPoint.z();
  }
}

void pathHandler(const nav_msgs::Path::ConstPtr& pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  originPath_ = *pathIn;
  updatePathFrame();

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

// void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
// {
//   joyTime = ros::Time::now().toSec();

//   joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
//   joySpeed = joySpeedRaw;
//   if (joySpeed > 1.0) joySpeed = 1.0;
//   if (joy->axes[4] == 0) joySpeed = 0;
//   joyYaw = joy->axes[3];
//   if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

//   if (joy->axes[4] < 0 && !twoWayDrive) {
//     joySpeed = 0;
//     joyYaw = 0;
//   }

//   if (joy->axes[2] > -0.1) {
//     autonomyMode = false;
//   } else {
//     autonomyMode = true;
//   }
// }

void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::Int8::ConstPtr& stop)
{
  safetyStop = stop->data;
}

void publishLookAheadPoint(ros::Publisher& pubLAPoint, float x, float y, int id,
                           float a, float r, float g, float b)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "lookAheadPoint";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = a; // Don't forget to set the alpha!
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  // 持久显示
  marker.lifetime = ros::Duration(); // 0表示永远存在

  pubLAPoint.publish(marker);
}

int getLookAheadPointFwd(const float LAPx, const float LAPy)
{
  float LAPDir = atan2(LAPy, LAPx);
  if (LAPDir > -PI / 2 && LAPDir < PI / 2) {
    return 1; // 前方
  } else if (LAPDir > PI / 2 || LAPDir < -PI / 2) {
    return -1; // 后方
  } else {
    return 0; // 无效
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathFollower");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("pubSkipNum", pubSkipNum);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("lookAheadDis", lookAheadDis);
  nhPrivate.getParam("yawRateGain", yawRateGain);
  nhPrivate.getParam("stopYawRateGain", stopYawRateGain);
  nhPrivate.getParam("maxYawRate", maxYawRate);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("maxAccel", maxAccel);
  nhPrivate.getParam("switchTimeThre", switchTimeThre);
  nhPrivate.getParam("dirDiffThre", dirDiffThre);
  nhPrivate.getParam("stopDisThre", stopDisThre);
  nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre);
  nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow);
  nhPrivate.getParam("inclRateThre", inclRateThre);
  nhPrivate.getParam("slowRate1", slowRate1);
  nhPrivate.getParam("slowRate2", slowRate2);
  nhPrivate.getParam("slowTime1", slowTime1);
  nhPrivate.getParam("slowTime2", slowTime2);
  nhPrivate.getParam("useInclToStop", useInclToStop);
  nhPrivate.getParam("inclThre", inclThre);
  nhPrivate.getParam("stopTime", stopTime);
  nhPrivate.getParam("noRotAtStop", noRotAtStop);
  nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, odomHandler);
  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path> ("/path", 5, pathHandler);
  // ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);
  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);
  ros::Subscriber subStop = nh.subscribe<std_msgs::Int8> ("/stop", 5, stopHandler);

  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 5);
  ros::Publisher pubIsReached = nh.advertise<std_msgs::Bool> ("/is_reached", 5);
  ros::Publisher pubLAPoint = nh.advertise<visualization_msgs::Marker>("/lookAheadPoint", 10);

  geometry_msgs::Twist cmd_vel;
  std_msgs::Bool isReached;

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;  // 一般为1

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    // updatePathFrame();

    if (pathInit) {
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);
      // publishLookAheadPoint(pubLAPoint, vehicleXRel, vehicleYRel, 1,
      //                       0.5, 0.0, 1.0, 0.0);

      int pathSize = path.poses.size();
      std::cout << "ptbbb pathSize: " << pathSize << std::endl;
      std::cout << "ptbbb pathPointID: " << pathPointID << std::endl;
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);
      // publishLookAheadPoint(pubLAPoint, endDisX, 
      //                       endDisY, 2,
      //                       0.5, 0.0, 0.0, 1.0);

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        std::cout << "ptbbb dis: " << dis << " endDis: "<< endDis << std::endl;
        static bool nearEnd = false;
        if (endDis <= 0.2){
          // navFwd 相当于上一次的 lookAheadPointFwd
          int LAPtoward = getLookAheadPointFwd(disX, disY);
          if (dis > 0.04){
            break;
          } else{
            if (LAPtoward == -1 && navFwd) {
              pathPointID++;
            } else if (LAPtoward == 1 && !navFwd) {
              pathPointID++;
            } else {
              break;
            }
          }
          nearEnd = true;
        }
        else if (dis < lookAheadDis) {
          nearEnd = false;
          pathPointID++;
        } else {
          break;
        }
      }
      // publishLookAheadPoint(pubLAPoint, path.poses[pathPointID].pose.position.x, path.poses[pathPointID].pose.position.y, 0,
      //                       0.5, 1.0, 0.0, 0.0);
      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      publishLookAheadPoint(pubLAPoint, disX, disY, 3,
                            0.5, 0.5, 0.5, 0.0);
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      if (twoWayDrive) {
        double time = ros::Time::now().toSec();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;  // joySpeed2 == autonomySpeed
      if (!navFwd) {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
      else vehicleYawRate = -yawRateGain * dirDiff;

      if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;

      if (joySpeed2 == 0 && !autonomyMode) {   // autonomySpeed 不设为0时，不会进入该if
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
        isReached.data = true;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) joySpeed3 *= slowRate2;

      if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;

        std::cout << "dis:" << dis << " stopDisThre:" << stopDisThre << std::endl;///////////////////////////////////////////
        if(dis > 0.04 || dirDiff > 5.0/180.0*M_PI)
        {
          if (navFwd && vehicleSpeed < 0.1) vehicleSpeed = dis;
          else if (!navFwd && vehicleSpeed > -0.1) vehicleSpeed = -dis;

          // bool oscillation = false;
          // static int oscillationCount = 0;
          // static double lastJudgeTime = ros::Time::now().toSec();
          // static int lastLAPFwd = getLookAheadPointFwd(disX, disY);
          // if (lastLAPFwd == 1 && getLookAheadPointFwd(disX, disY) == -1) oscillationCount++;
          // else if (lastLAPFwd == -1 && getLookAheadPointFwd(disX, disY) == 1) oscillationCount++;
          // std::cout << "oscillationCount: " << oscillationCount << std::endl;
          // if (ros::Time::now().toSec() - lastJudgeTime > 1.5) {
          //   if (oscillationCount >= 2) {
          //     oscillation = true;
          //   }
          //   oscillationCount = 0;
          //   lastJudgeTime = ros::Time::now().toSec();
          // }
          // lastLAPFwd = getLookAheadPointFwd(disX, disY);
          // if (oscillation && dirDiff <= 5.0/180.0*M_PI) vehicleSpeed = 0;
          
          if (dis <= 0.05 && dirDiff <= 5.0/180.0*M_PI) vehicleSpeed = 0;
          // if (navFwd && vehicleSpeed < 0.1) vehicleSpeed += maxAccel / 50.0;
          // else if (!navFwd && vehicleSpeed > -0.1) vehicleSpeed = vehicleSpeed -= maxAccel / 50.0;

          if (vehicleSpeed != 0.0)
          {
            if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
            else vehicleYawRate = -yawRateGain * dirDiff;
          }
        }
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }
      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        if (fabs(vehicleSpeed) <= maxAccel / 100.0) {
          cmd_vel.linear.x = 0;
          }
        else {
          cmd_vel.linear.x = vehicleSpeed;
          isReached.data = false;
          }
        cmd_vel.angular.z = vehicleYawRate;
        pubSpeed.publish(cmd_vel);
        pubIsReached.publish(isReached);
        if(isReached.data)  isReached.data = false;

        pubSkipCount = pubSkipNum;
      }
    }

    rate.sleep();
  }

  return 0;
}
