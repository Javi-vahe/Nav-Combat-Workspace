
#ifndef _DZjudgment_H
#define _DZjudgment_H

#include <iostream>
#include <math.h>
#include <string.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>

#include "serial/serial.h"
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

/* FOREGROUND */

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

#define PI 3.14159265
using namespace std;
using namespace boost::asio;
using namespace boost;
#define MAX_VALID_DATA_LENGTH 100
typedef struct sMartcarControl
{
  int TargetAngle;
  int TargetSpeed;
  int TargetAngleDir;
  int TargetModeSelect;
  int TargetShiftPosition;
} sMartcarControl;

class dzjudgment
{
public:
  dzjudgment(ros::NodeHandle handle);
  ~dzjudgment();

  void callback_LaserShot_Command(const std_msgs::UInt8::ConstPtr &msg);
  void run();

public:
  int m_baudrate;
  std::string m_serialport;
  ros::NodeHandle m_handle;
  serial::Serial ser;
  // msg
  ros::Subscriber sub_LaserShot_Command;
  ros::Publisher pub_hpandhismsg;
  ros::Publisher pub_all_Material_Number;
  ros::Publisher pub_enemy_Material_Number;
  ros::Publisher pub_self_Material_Number;

  void recvESP32Info();
  void sendLaserShot();
  void processData(uint8_t *charArray, uint8_t dataLength);
  bool reconnectSerial();

  // 定义存储数据的数组
  uint8_t all_Material_Number[MAX_VALID_DATA_LENGTH];
  uint8_t enemy_Material_Number[MAX_VALID_DATA_LENGTH];
  uint8_t self_Material_Number[MAX_VALID_DATA_LENGTH];
  uint8_t validDataLength = 0;
  uint8_t self_hp = 0;        // 己方血量
  uint8_t targethp = 0;       // 对方血量
  uint8_t Being_hit = 0x10;    // 是否被敌方击打  0x01左方,0x02前方，0x03右方，0x04后方,0x10未被击打
  uint8_t ammo = 10;          // 子弹个数
  uint8_t Shooting_Count = 0; // 总的射击次数
  uint8_t previousHeader2 = 0;
};

#endif
