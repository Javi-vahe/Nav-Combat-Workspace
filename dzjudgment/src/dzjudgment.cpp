

#include"judgment.h"


int main(int argc, char** argv){
  ros::init(argc,argv,"dzjudgment");
  ros::NodeHandle nh;
  dzjudgment* dznode = new dzjudgment(nh);
  dznode->run();
  ros::spin();
}
