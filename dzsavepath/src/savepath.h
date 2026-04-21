#ifndef NODE_EXAMPLE_TALKER_H
#define NODE_EXAMPLE_TALKER_H

// c++ lib
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/assert.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <fstream>
#include <filesystem> // C++17 的标准库


// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include <sys/time.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

using namespace std;

#define HDR1 0xff
#define HDR2 0xa5
#define HDR3 0x5a
#define MAX_ZMQ_ARRAYSIZE 10250

#define OFFLINETIME 3 // 3s
#define DISTANCE 1.2  // 单位m

#define RST "\x1B[0m"
#define KRED "\x1B[31m"
#define KGRN "\x1B[32m"
#define KYEL "\x1B[33m"
#define KBLU "\x1B[34m"
#define KMAG "\x1B[35m"
#define KCYN "\x1B[36m"
#define KWHT "\x1B[37m"
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST
#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST

#define EARTHRADIUS 6378137.0 // 6378245

typedef struct Ars_408
{
  double x;
  double y;
  double velx;
  double vely;
} Ars_408;

class savepath
{
public:
  //! Constructor.
  savepath(ros::NodeHandle nh);
  ~savepath();

  void run();
  FILE *fp;

  std::string mappinmgsavePath;
  geometry_msgs::Pose current_position, last_position;
  bool receivePose_flag;
  double distance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2);
};

#endif // NODE_EXAMPLE_TALKER_H
