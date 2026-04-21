

#include "savepath.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dzsavepath");
  ros::NodeHandle nh;
  savepath node(nh);
  node.run();
  return 0;
}

