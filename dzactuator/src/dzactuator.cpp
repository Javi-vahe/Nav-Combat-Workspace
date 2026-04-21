
#include "dzactuator.h"
#include "Quaternion_Solution.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <zllc_msgs/Motors.h>

sensor_msgs::Imu Mpu6050;

const int motor0_center_position_fixed = 2047; // 电机0云台俯仰中心位置修正值
const int motor1_center_position_fixed = 2047; // 电机1云台偏航中心位置修正值

const int GIMBAL_MOTOR_SPEED_MAX = 3073; // 云台电机最大速度

const double g = 9.80665;

turn_on_robot::turn_on_robot() : Power_voltage(0)
{

  // Clear the data
  // 清空数据
  linear_Speed = 0;
  ThetaSpeed = 0;
  ticksPerMeter = 0;
  ticksPer2PI = 0;
  leftDistance = 0;
  rightDistance = 0;
  calibrate_lineSpeed = 0;
  last_Battery_Percentage = 0;
  count_B = 0;
  count_A = 0;
  count_C = 0;

  Power_max = 12;
  Power_min = 10;

  gimbal_cruise_msg = 0;
  find_center = false;
  return_center = false;
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));
  memset(&moveBaseControl, 0, sizeof(sMartcarControl));
  moveBaseControl.Position_0 = motor0_center_position_fixed;
  moveBaseControl.Position_1 = motor0_center_position_fixed;
  moveBaseControl.Speed_0 = GIMBAL_MOTOR_SPEED_MAX;
  moveBaseControl.Speed_1 = GIMBAL_MOTOR_SPEED_MAX;
  moveBaseControl.Time_0 = 0;
  moveBaseControl.Time_1 = 0;
  moveBaseControl.VoiceSwitch = 0xFF;
  moveBaseControl.TargetAngle = 60;
  moveBaseControl.SpeedDirection = 0x00;
  moveBaseControl.TargetSpeed = 0;

  ros::NodeHandle private_nh("~"); // Create a node handle //创建节点句柄
  // The private_nh.param() entry parameter corresponds to the initial value of the name of the parameter variable on the parameter server
  // private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  private_nh.param<std::string>("serial_port_name", serial_port_name, "/dev/ttyACM0"); // Fixed serial port number //固定串口号
  private_nh.param<int>("serial_baud_rate", serial_baud_rate, 115200);                 // Communicate baud rate 115200 to the lower machine //和下位机通信波特率115200
  private_nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");               // The odometer topic corresponds to the parent TF coordinate //里程计话题对应父TF坐标
  private_nh.param<std::string>("robot_frame_id", robot_frame_id, "base_link");        // The odometer topic corresponds to sub-TF coordinates //里程计话题对应子TF坐标
  private_nh.param<std::string>("gyro_frame_id", gyro_frame_id, "imu_link");           // IMU topics correspond to TF coordinates //IMU话题对应TF坐标
  private_nh.param("calibrate_lineSpeed", calibrate_lineSpeed, calibrate_lineSpeed);
  private_nh.param("ticksPerMeter", ticksPerMeter, ticksPerMeter);
  private_nh.param("ticksPer2PI", ticksPer2PI, ticksPer2PI);

  voltage_publisher = n.advertise<std_msgs::Float32>("/PowerVoltage", 1);            // Create a battery-voltage topic publisher //创建电池电压话题发布者
  Battery_Percentage_pub = n.advertise<std_msgs::Float32>("/Battery_Percentage", 1); // Create a battery-voltage topic publisher //创建电池电量百分比话题发布者
  odom_publisher = n.advertise<nav_msgs::Odometry>("/odom", 200);                    // Create the odometer topic publisher //创建里程计话题发布者
  imu_publisher = n.advertise<sensor_msgs::Imu>("/imu/raw", 200);                    // Create an IMU topic publisher //创建IMU话题发布者
  imu_correct_publisher = n.advertise<sensor_msgs::Imu>("/imu/correct", 200);        // imu 补偿倾角后的发布者
  // pub_diff = n.advertise<sensor_msgs::Imu>("imu_data", 200);
  // pub_imu_msg_valid = n.advertise<std_msgs::UInt8>("imu_msg_valid", 10);         // Create an IMU online flag  创建IMU在线标志位
  // pub_odom_msg_valid = n.advertise<std_msgs::UInt8>("odom_msg_valid", 10);       // Create an ODOM online flag 创建ODOM在线标志位
  pub_LaserShot_Command = n.advertise<std_msgs::UInt8>("/LaserShot_Command", 1); // Create an Laser online flag 创建Lidar在线标志位
  pub_gimbal_feedback = n.advertise<zllc_msgs::Motors>("/gimbal_feedback", 10);  // Create a gimbal feedback topic publisher //创建云台反馈话题发布者
  pub_chassis_mode = n.advertise<std_msgs::UInt8>("/chassis_mode", 1);           // 底盘模式发布
  take_off_pub_ = n.advertise<std_msgs::String>("/take_off", 1);                 // 切换物资识别模型

  // Set the velocity control command callback function
  // 速度控制命令订阅回调函数设置
  sub_movebase_angle = n.subscribe("/pursuitAngle", 1, &turn_on_robot::callback_movebase_angle, this, ros::TransportHints().tcpNoDelay()); // 速度控制回调
  // sub_cmd_vel = n.subscribe("cmd_vel", 1, &turn_on_robot::callback_cmd_vel_angle, this);                                // 速度控制回调
  sub_motor_control = n.subscribe("/gimbal_control/motor_control", 1, &turn_on_robot::callback_motor_control, this, ros::TransportHints().tcpNoDelay()); // 云台直接控制回调
  sub_offset_center = n.subscribe("/offset_center", 10, &turn_on_robot::callback_offset_center, this, ros::TransportHints().tcpNoDelay());               // 识别信息回调
  sub_gimbal_cruise_flag = n.subscribe("/gimbal_cruise", 1, &turn_on_robot::callback_gimbal_cruise_flag, this, ros::TransportHints().tcpNoDelay());      // 巡航开关回调
  sub_voice_switch = n.subscribe("/voice_switch", 1, &turn_on_robot::callback_voice_switch, this, ros::TransportHints().tcpNoDelay());                   // 语音播报回调
  // ros::Publisher det_pub = nh.advertise<std_msgs::Int32MultiArray>("/pt_det_topic", 1);
  sub_HpAndHitmsg = n.subscribe("/HpAndHitmsg", 5, &turn_on_robot::callback_HpAndHitmsg, this, ros::TransportHints().tcpNoDelay()); // 血量和被击打信息回调

  ROS_INFO_STREAM("Data ready"); // Prompt message //提示信息

  std::cout << FRED("Copyright©2016-2020 dzactuator. All rights reserved ") << std::endl;
  std::cout << FYEL("*****dzactuator:parameters*******************") << std::endl;
  std::cout << FGRN("serial_port_name:") << serial_port_name << std::endl;
  std::cout << FGRN("serial_baud_rate:") << serial_baud_rate << std::endl;
  std::cout << FGRN("calibrate_lineSpeed:") << calibrate_lineSpeed << std::endl;
  std::cout << FGRN("ticksPerMeter:") << ticksPerMeter << std::endl;
  std::cout << FGRN("ticksPer2PI:") << ticksPer2PI << std::endl;
  std::cout << FGRN("Power_max:") << Power_max << std::endl;
  std::cout << FGRN("Power_min:") << Power_min << std::endl;
  std::cout << FGRN("gyro_frame_id:") << gyro_frame_id << std::endl;
  std::cout << FYEL("*****dzactuator:parameters end***************") << std::endl;

  // RCLCPP_ERROR_STREAM(this->get_logger(), "dzactuator can not open serial port,Please check the serial port cable! "); // If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  try
  {
    // Attempts to initialize and open the serial port //尝试初始化与开启串口
    Stm32_Serial.setPort(serial_port_name);                     // Select the serial port number to enable //选择要开启的串口号
    Stm32_Serial.setBaudrate(serial_baud_rate);                 // Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(30); // Timeout //超时等待
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open(); // Open the serial port //开启串口
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("dzactuator can not open serial port,Please check the serial port cable! "); // If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if (Stm32_Serial.isOpen())
  {
    ROS_INFO_STREAM("dzactuator serial port opened"); // Serial port opened successfully //串口开启成功提示
  }
}

turn_on_robot::~turn_on_robot()
{
  // Sends the stop motion command to the lower machine before the turn_on_robot object ends
  // 对象turn_on_robot结束前向下位机发送停止运动命令
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;
  Send_Data.tx[2] = 0;

  // The target velocity of the X-axis of the robot //机器人X轴的目标线速度
  Send_Data.tx[4] = 0;
  Send_Data.tx[3] = 0;

  // The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;

  // The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度
  Send_Data.tx[8] = 0;
  Send_Data.tx[7] = 0;
  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data.tx[10] = FRAME_TAIL;
  try
  {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); // Send data to the serial port //向串口发数据
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); // If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
  Stm32_Serial.close(); // Close the serial port //关闭串口
  // RCLCPP_INFO_STREAM(this->get_logger(), "Shutting down"); // Prompt message //提示信息
}

// 循环RUN()函数
void turn_on_robot::Control()
{
  Robot_Pos.X = 0;
  Robot_Pos.Y = 0;
  Robot_Pos.Z = 0;
  last_serial_received_time = ros::Time::now();
  // int freq = 50;
  // ros::Rate rate(freq);
  while (ros::ok())
  {
    ros::spinOnce();                   // The loop waits for the callback function //循环等待回调函数
    if (true == Get_Sensor_Data_New()) // The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
    // 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
    {
      double dt = (serial_received_time - last_serial_received_time).toSec();
      last_serial_received_time = serial_received_time; // Update the last serial port data receiving time //更新上次串口数据接收时间
      // std::cout << "hz:" << 1.0 / dt << std::endl;
      // continue;

      leftDistance += Robot_Vel.Left;
      rightDistance += Robot_Vel.Right;
      // 计算左右轮的速度
      // double leftSpeed = Robot_Vel.Left / ticksPerMeter;
      // double rightSpeed = Robot_Vel.Right / ticksPerMeter;
      // double leftSpeed = Robot_Vel.Left / 70.0; // 下发1m/s，回传70增量
      // double rightSpeed = Robot_Vel.Right / 70.0;
      // double leftSpeed = Robot_Vel.Left * 1.142857;
      // double rightSpeed = Robot_Vel.Right * 1.142857;
      double leftSpeed = Robot_Vel.Left;
      double rightSpeed = Robot_Vel.Right;

      linear_Speed = (leftSpeed + rightSpeed) / 2.0;

      // ThetaSpeed = (Robot_Vel.Right - Robot_Vel.Left) * (2 * M_PI) / ticksPer2PI; // 使用打死情况下的编码器值计算角度变化
      ThetaSpeed = (rightSpeed - leftSpeed) / (0.105); // 假设左右轮距0.1

      // double theta = atan(0.155 * ThetaSpeed / linear_Speed); // 计算机器人转弯角度
      double theta = Robot_Vel.Steering_Angle / 180.0 * M_PI;
      // std::cout << "theta=" << theta / M_PI * 180.0
      //           << "\tleftSpeed=" << leftSpeed
      //           << "\trightSpeed=" << rightSpeed
      //           << std::endl;

      if (linear_Speed != 0)
      {
        montion_flag = true;

        // Robot_Pos.Z += ThetaSpeed; // The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad
        Robot_Pos.Z += linear_Speed * tan(theta) / 0.155 * dt; // The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad
        Robot_Pos.Z = normalizeAngle(Robot_Pos.Z);
        Robot_Pos.X += linear_Speed * cos(Robot_Pos.Z) * dt; // Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
        Robot_Pos.Y += linear_Speed * sin(Robot_Pos.Z) * dt;
      }
      Publish_Odom(); // Pub the speedometer topic //发布里程计话题

      if (calibrate_lineSpeed == 1)
      {
        printf("left=%.2f,right = %.2f,x=%.2f,y=%.2f,th=%.2f,linear_Speed=%f,leftDistance = %.2f,rightDistance = %.2f,Power_voltage = %.2f\n",
               Robot_Vel.Left, Robot_Vel.Right, Robot_Pos.X, Robot_Pos.Y, Robot_Pos.Z, linear_Speed, leftDistance, rightDistance, Power_voltage);
      }
      // 通过IMU绕三轴角速度与三轴加速度计算三轴姿态
      // ============================================================================
      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                          Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);

      Publish_ImuSensor(); // Pub the IMU topic //发布IMU话题
      // Publish_Voltage();            // Pub the topic of power supply voltage //发布电源电压话题
      // Publish_Battery_Percentage(); // Pub the topic of power supply voltage //发布电源电量百分比话题
      Publish_Gimbal_Feedback(); // Pub the gimbal feedback topic //发布云台反馈话题
      CaremaMotorControl();      // 云台相机控制策略
      sendCarInfoKernel();       // 向STM32发送控制指令
      // rate.sleep();
    }
  }
}

void turn_on_robot::callback_HpAndHitmsg(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
  auto received_time = ros::Time::now();
  hp_and_hit_msg_.enemy_hp = msg->data[0];       // 对方血量
  hp_and_hit_msg_.self_hp = msg->data[1];        // 自身血量
  hp_and_hit_msg_.being_hit = msg->data[2];      // 是否被敌方击打 0x01左方,0x02前方，0x03右方，0x04后方,16未被击打
  hp_and_hit_msg_.shooting_Count = msg->data[3]; // 已射击次数
  hp_and_hit_msg_.ammo = msg->data[4];           // 剩余弹药数量
  if (hp_and_hit_msg_.being_hit != 16)
  {
    hp_and_hit_msg_.last_being_hit = hp_and_hit_msg_.being_hit;
    hp_and_hit_msg_.last_being_hit_time = received_time.toSec(); // 更新上一次被击打的时间
    ROS_WARN("Being hit: %d, time: %f",
             hp_and_hit_msg_.last_being_hit, hp_and_hit_msg_.last_being_hit_time);
  }
}

/**
 * @description: The function normalizes angle values.
 * @param {double} angle
 * @return {double}  angle
 * @author: Senerity
 */
double turn_on_robot::normalizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

/**************************************
Date: January 28, 2021
Function: Data conversion function
功能: 数据转换函数
***************************************/
short turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  short transition_16 = (Data_High << 8) | Data_Low;
  return transition_16;
}

float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  float data_return;
  short transition_16 = (Data_High << 8) | Data_Low;
  data_return = transition_16; //(transition_16 / 1000)+(transition_16 % 1000)*0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
  return data_return;
}

/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void turn_on_robot::callback_movebase_angle(const geometry_msgs::Twist::ConstPtr &msg)
{
  // printf("callback_movebase_angle\n");
  float v = msg->linear.x;
  float w = msg->angular.z;

  moveBaseControl.TargetSpeed = abs(clamp(v * 23 / 0.33, -255.0, 255.0));
  moveBaseControl.TargetAngle = w;
  moveBaseControl.TargetAngle += 60;

  // printf("linear X = %.2d\n",moveBaseControl.TargetSpeed);
  if (moveBaseControl.TargetAngle < 20)
  {
    moveBaseControl.TargetAngle = 20;
  }
  if (moveBaseControl.TargetAngle > 100)
  {
    moveBaseControl.TargetAngle = 100;
  }
  // linear speed
  if (msg->linear.x > 0)
    moveBaseControl.SpeedDirection = 0x00;
  else if (msg->linear.x < 0)
    moveBaseControl.SpeedDirection = 0x01;
  else if (msg->linear.x == 0)
    moveBaseControl.SpeedDirection = 0x00;
}

/*
void turn_on_robot::callback_cmd_vel_angle(const geometry_msgs::Twist::ConstPtr &msg)
{
  // printf("callback_cmd_vel_angle\n");
  float v = msg->linear.x;
  float w = msg->angular.z;

  moveBaseControl.TargetSpeed = abs(clamp(v * 23 / 0.33, -255.0, 255.0));
  if (!v == 0)
  {
    moveBaseControl.TargetAngle = round(atan(CARL * w / v) * 57.3);
  }
  else
  {
    moveBaseControl.TargetAngle = 0;
  }
  moveBaseControl.TargetAngle += 60;

  // std::cout << " msg->linear.x = " << moveBaseControl.TargetSpeed << " msg->angular.z =  " <<  moveBaseControl.TargetAngle <<std::endl;

  // printf("linear X = %.2f,gngularZ = %.2f,%d,angle = %d\n",msg->linear.x,msg->angular.z,
  // abs(moveBaseControl.TargetSpeed),abs(moveBaseControl.TargetAngle));
  if (moveBaseControl.TargetAngle < 20)
  {
    moveBaseControl.TargetAngle = 20;
  }
  if (moveBaseControl.TargetAngle > 100)
  {
    moveBaseControl.TargetAngle = 100;
  }
  // linear speed
  if (msg->linear.x > 0)
    moveBaseControl.SpeedDirection = 0x00;
  else if (msg->linear.x < 0)
    moveBaseControl.SpeedDirection = 0x01;
  else if (msg->linear.x == 0)
    moveBaseControl.SpeedDirection = 0x00;
  // // angle
  // if (moveBaseControl.TargetAngle > 0)
  //   moveBaseControl.TargetShiftPosition = 0x20; // turn left
  // else if (moveBaseControl.TargetAngle < 0)
  //   moveBaseControl.TargetShiftPosition = 0x10; // turn right
  // else if (moveBaseControl.TargetAngle == 0)
  //   moveBaseControl.TargetShiftPosition = 0x00; // turn left
}
*/
void turn_on_robot::callback_voice_switch(const std_msgs::UInt8::ConstPtr &msg)
{
  moveBaseControl.VoiceSwitch = static_cast<int>(msg->data);
  std::cout << "moveBaseControl.VoiceSwitch = " << moveBaseControl.VoiceSwitch << std::endl;
}

void turn_on_robot::callback_gimbal_cruise_flag(const std_msgs::UInt8::ConstPtr &msg)
{
  if (static_cast<int>(msg->data) == gimbal_cruise_msg)
    return;
  gimbal_cruise_msg = static_cast<int>(msg->data);
  std::cout << "gimbal_cruise flag = " << gimbal_cruise_msg << std::endl;
  if (gimbal_cruise_msg == 11 || gimbal_cruise_msg == 12 || gimbal_cruise_msg == 13 || gimbal_cruise_msg == 14)
    custom_gimbal_cruise_time_ = ros::Time::now();
}

void turn_on_robot::callback_motor_control(const geometry_msgs::Twist::ConstPtr &msg)
{
  // moveBaseControl.Position_0 = static_cast<int>(msg->linear.x);
  // moveBaseControl.Position_1 = static_cast<int>(msg->linear.y);
  moveBaseControl.Position_0 = clamp(static_cast<int>(msg->linear.x), 1536, 2350);
  moveBaseControl.Position_1 = clamp(static_cast<int>(msg->linear.y), 0, 4096);
  // std::cout << "Position_0:" << moveBaseControl.Position_0 << " Position_1:" << moveBaseControl.Position_1 << std::endl;
  moveBaseControl.Speed_0 = GIMBAL_MOTOR_SPEED_MAX;
  moveBaseControl.Speed_1 = GIMBAL_MOTOR_SPEED_MAX;
  moveBaseControl.Time_0 = 0;
  moveBaseControl.Time_1 = 0;
  moveBaseControl.Fun = static_cast<int>(msg->linear.z);
  moveBaseControl.Orin_ID = 1;
  moveBaseControl.Set_ID_Num = 1;
  // printf("callback_motor_control\n");
}

void motor_cruise_clamp(int &motor_position, int min_position, int max_position, int &direction)
{
  if (motor_position > max_position)
  {
    motor_position = max_position;
    direction = -1;
  }
  else if (motor_position < min_position)
  {
    motor_position = min_position;
    direction = 1;
  }
}

/*
 * @brief 控制巡航速度
 */
void turn_on_robot::CaremaMotorControl()
{
  // const int motor0_pitch_cruise_max = 2350; // 电机0 pitch最低位置
  // const int motor0_pitch_cruise_min = 2100; // 电机0 pitch最高位置
  const int motor0_pitch_cruise_max = 2121; // 电机0 pitch最低位置
  const int motor0_pitch_cruise_min = 2121; // 电机0 pitch最高位置
  const int motor1_yaw_cruise_max = 4096;   // 电机1 yaw最右位置
  const int motor1_yaw_cruise_min = 0;      // 电机1 yaw最左位置
  // 左巡航
  const int motor1_yaw_cruise_max_left = 1650;
  const int motor1_yaw_cruise_min_left = 300;
  // 前巡航
  const int motor1_yaw_cruise_max_front = 2450;
  const int motor1_yaw_cruise_min_front = 1650;
  // 右巡航
  const int motor1_yaw_cruise_max_right = 4096;
  const int motor1_yaw_cruise_min_right = 2450;
  // 后巡航
  const int motor1_yaw_cruise_max_rear = 300;
  const int motor1_yaw_cruise_min_rear = 0;
  // 静态变量用于保持状态
  static int direction_0 = -1;  // 控制扫描方向：1表示正向，-1表示反向
  static int step_0 = 100;      // 每次移动的步长
  static int direction_1 = -1;  // 控制扫描方向：1表示正向，-1表示反向
  static int step_1 = 100;      // 每次移动的步长
  static int scan_interval = 0; // 控制扫描速度的计数器
  if (find_center == false)
  {
    if (gimbal_cruise_msg == 1 ||
        gimbal_cruise_msg == 11 || gimbal_cruise_msg == 12 || gimbal_cruise_msg == 13 || gimbal_cruise_msg == 14)
    {

      // 每隔一定周期调整位置，避免50Hz过于频繁地发送位置命令
      if (scan_interval++ >= 20)
      {
        scan_interval = 0;

        moveBaseControl.Position_0 += direction_0 * step_0;
        moveBaseControl.Position_1 += direction_1 * step_1;

        motor_cruise_clamp(moveBaseControl.Position_0, motor0_pitch_cruise_min, motor0_pitch_cruise_max, direction_0);
        // 受击打巡航
        if ((ros::Time::now().toSec() - hp_and_hit_msg_.last_being_hit_time) < 2.0 &&
            hp_and_hit_msg_.last_being_hit == 4) // 后面视野小，看的时间短一些
        {
          // std::cout << "rear" << std::endl;
          motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_rear, motor1_yaw_cruise_max_rear, direction_1);
        }
        else if ((ros::Time::now().toSec() - hp_and_hit_msg_.last_being_hit_time) < 3.0) // 受击打巡航
        {
          switch (hp_and_hit_msg_.last_being_hit)
          {
          case 1: // 左侧被击打
            // std::cout << "left" << std::endl;
            motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_left, motor1_yaw_cruise_max_left, direction_1);
            break;
          case 2: // 前方被击打
            // std::cout << "front" << std::endl;
            motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_front, motor1_yaw_cruise_max_front, direction_1);
            break;
          case 3: // 右侧被击打
            // std::cout << "right" << std::endl;
            motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_right, motor1_yaw_cruise_max_right, direction_1);
            break;
          default:
            break;
          }
        }
        else if ((ros::Time::now() - custom_gimbal_cruise_time_).toSec() < 3.0) // 自定义巡航
        {
          switch (gimbal_cruise_msg)
          {
          case 11: // 左巡航
            motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_left, motor1_yaw_cruise_max_left, direction_1);
            break;
          case 12: // 前巡航
            motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_front, motor1_yaw_cruise_max_front, direction_1);
            break;
          case 13: // 右巡航
            motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_right, motor1_yaw_cruise_max_right, direction_1);
            break;
          case 14: // 后巡航
            motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_rear, motor1_yaw_cruise_max_rear, direction_1);
            break;
          default:
            // std::cout << "default" << std::endl;
            motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min, motor1_yaw_cruise_max, direction_1);
            break;
          }
        }
        else if ((ros::Time::now().toSec() - hp_and_hit_msg_.last_being_hit_time) > 3.0) // 正常巡航
        {
          // std::cout << "default" << std::endl;
          motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min, motor1_yaw_cruise_max, direction_1);
        }
        else // 其他情况
        {
          // std::cout << "else" << std::endl;
          motor_cruise_clamp(moveBaseControl.Position_0, motor0_pitch_cruise_min, motor0_pitch_cruise_max, direction_0);
          motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min, motor1_yaw_cruise_max, direction_1);
        }
        // std::cout << "moveBaseControl.Position_0: " << moveBaseControl.Position_0 << std::endl;
        // std::cout << "moveBaseControl.Position_1: " << moveBaseControl.Position_1 << std::endl;
      }
      // 设置电机速度
      moveBaseControl.Speed_0 = 1200;
      moveBaseControl.Speed_1 = 1200;
    }
    else if (gimbal_cruise_msg == 0)
    {
      // moveBaseControl.Position_0 = motor0_center_position_fixed;
      // moveBaseControl.Position_1 = motor1_center_position_fixed;
      // moveBaseControl.Speed_0 = 2000;
      // moveBaseControl.Speed_1 = 2000;
    }
    else if (gimbal_cruise_msg == 2)
    {
      moveBaseControl.Position_0 = motor0_center_position_fixed;
      moveBaseControl.Position_1 = motor1_center_position_fixed;
      moveBaseControl.Speed_0 = GIMBAL_MOTOR_SPEED_MAX;
      moveBaseControl.Speed_1 = GIMBAL_MOTOR_SPEED_MAX;
    }
    else if (gimbal_cruise_msg == 3) // 前巡航
    {
      if (scan_interval++ >= 20)
      {
        scan_interval = 0;

        moveBaseControl.Position_0 += direction_0 * step_0;
        moveBaseControl.Position_1 += direction_1 * step_1;

        motor_cruise_clamp(moveBaseControl.Position_0, motor0_pitch_cruise_min, motor0_pitch_cruise_max, direction_0);
        motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_front, motor1_yaw_cruise_max_front, direction_1);
        moveBaseControl.Speed_0 = 1200;
        moveBaseControl.Speed_1 = 1200;
      }
    }
  }
  else // 自瞄有目标时
  {
    if (gimbal_cruise_msg == 3) // 自瞄时检测被击打巡航
    {
      if ((ros::Time::now().toSec() - hp_and_hit_msg_.last_being_hit_time) > 3.0) // 没被击打
      {
        return;
      }
      // 自瞄时被击打 ↓↓↓
      moveBaseControl.Speed_0 = 1200;
      moveBaseControl.Speed_1 = 1200;
      if (scan_interval++ >= 20)
      {
        scan_interval = 0;

        moveBaseControl.Position_0 += direction_0 * step_0;
        moveBaseControl.Position_1 += direction_1 * step_1;

        motor_cruise_clamp(moveBaseControl.Position_0, motor0_pitch_cruise_min, motor0_pitch_cruise_max, direction_0);
        // 受击打巡航
        if ((ros::Time::now().toSec() - hp_and_hit_msg_.last_being_hit_time) < 2.0 &&
            hp_and_hit_msg_.last_being_hit == 4) // 后面视野小，看的时间短一些
        {
          motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_rear, motor1_yaw_cruise_max_rear, direction_1);
        }
        else if ((ros::Time::now().toSec() - hp_and_hit_msg_.last_being_hit_time) < 3.0) // 受击打巡航
        {
          switch (hp_and_hit_msg_.last_being_hit)
          {
          case 1: // 左侧被击打
            motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_left, motor1_yaw_cruise_max_left, direction_1);
            break;
          case 2: // 前方被击打
            motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_front, motor1_yaw_cruise_max_front, direction_1);
            break;
          case 3: // 右侧被击打
            motor_cruise_clamp(moveBaseControl.Position_1, motor1_yaw_cruise_min_right, motor1_yaw_cruise_max_right, direction_1);
            break;
          default:
            break;
          }
        }
      }
    }
  }
}

void turn_on_robot::callback_offset_center(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  // temp_msg.data[3] 0车上靶子 1移动靶
  std_msgs::Int32MultiArray temp_msg = *msg;
  // if (temp_msg.data[2] == 1 && gimbal_cruise_msg == 1)
  if (temp_msg.data[2] == 1)
  { // 检测到目标
    find_center = true;
    // std::cout << "return = " << return_center << std::endl;
    // printf("callback--find_center =%d\n",find_center);
    static int last_ek0 = 0;
    static int last_ek1 = 0;
    // static int ek0_2 = 0;
    // static int ek1_2 = 0;
    int ek0 = clamp(temp_msg.data[0], -50, 50);
    int ek1 = clamp(temp_msg.data[1], -50, 50);
    std::cout << "ek0:" << ek0 << "\tek1:" << ek1;

    int p_out_0 = 0.25 * (ek0 - last_ek0);
    int i_out_0 = 0.7 * (ek0);
    int p_out_1 = 0.25 * (ek1 - last_ek1);
    int i_out_1 = 0.7 * (ek1);

    // 瞄准目标
    moveBaseControl.Position_0 = curYuntai_feedback_data.Position_0 + (p_out_0 + i_out_0);
    moveBaseControl.Position_1 = curYuntai_feedback_data.Position_1 + (p_out_1 + i_out_1);

    moveBaseControl.Position_0 = clamp(moveBaseControl.Position_0, 2000, 2450);
    moveBaseControl.Position_1 = clamp(moveBaseControl.Position_1, 0, 4096);

    std::cout << "\tp0:" << moveBaseControl.Position_0 << "\tp2:" << moveBaseControl.Position_1;

    last_ek0 = ek0;
    last_ek1 = ek1;
    // std::cout << "Position_0 = " << moveBaseControl.Position_0 << std::endl;
    // std::cout << "Position_1 = " << moveBaseControl.Position_1 << std::endl;

    // moveBaseControl.Speed_0 = CaremaSpeedControl0(moveBaseControl.Position_0, curYuntai_feedback_data.Position_0);
    // moveBaseControl.Speed_1 = CaremaSpeedControl1(moveBaseControl.Position_1, curYuntai_feedback_data.Position_1);
    moveBaseControl.Speed_0 = 1000;
    moveBaseControl.Speed_1 = GIMBAL_MOTOR_SPEED_MAX;
    std::cout << "\tSpeed_0 = " << moveBaseControl.Speed_0;
    std::cout << "\tSpeed_1 = " << moveBaseControl.Speed_1 << std::endl;
    static std::deque<int> yaw_position_queue;
    yaw_position_queue.push_back(moveBaseControl.Position_1);
    if (yaw_position_queue.size() > 60)
    {
      yaw_position_queue.pop_front();
    }
    int dx = yaw_position_queue.front() - yaw_position_queue.back();
    static int outOfRangeCount = 0;
    if (moveBaseControl.Position_1 >= 4095 || moveBaseControl.Position_1 <= 1)
      ++outOfRangeCount;
    else
      outOfRangeCount = 0;
    if (yaw_position_queue.size() >= 60 && dx < 10 &&
        outOfRangeCount >= 59)
    { // 卡限位了
      std_msgs::String take_off_msg;
      take_off_msg.data = "3"; // 暂时关闭装甲板识别
      take_off_pub_.publish(take_off_msg);
      yaw_position_queue.clear();
    }
    else if (abs(temp_msg.data[1]) < 10 && abs(temp_msg.data[0]) < 10)
    {
      // static ros::Time last_shot_time = ros::Time::now();
      // if (ros::Time::now() - last_shot_time >= ros::Duration(0.50))
      // {
      std_msgs::UInt8 shotdata;
      shotdata.data = 1;
      pub_LaserShot_Command.publish(shotdata);
      // last_shot_time = ros::Time::now();
      // }
      // ros::Duration(0.2).sleep();
      // shotdata.data = 0;
      // pub_LaserShot_Command.publish(shotdata);
    }
  }
  else if (temp_msg.data[2] == 0)
  { // 未检测到目标
    find_center = false;

    // std_msgs::UInt8 shotdata;
    // shotdata.data = 0;
    // pub_LaserShot_Command.publish(shotdata);
  }
}

double turn_on_robot::CaremaSpeedControl0(int target_pose, int current_pose)
{
  // 静态变量用于存储 PID 状态
  static double prev_error = 0.0;
  static double integral = 0.0;
  // static ros::Time last_t = ros::Time::now();

  static constexpr double kp = 100.0;         // 稳定的比例增益
  static constexpr double ki = 0.00;         // 更低的积分增益
  static constexpr double kd = 0.03;         // 减小的微分增益
  static constexpr double max_speed = 1000.0; // 最大速度限制

  // auto now_t = ros::Time::now();
  // auto dt = (now_t - last_t).toSec();

  // 计算误差
  double error = static_cast<double>(target_pose - current_pose);

  integral += error;
  // // 更新积分项（带限幅）
  // integral += error * sampling_time;
  // integral = clamp(integral, -max_speed / (2.0 * ki), max_speed / (2.0 * ki));

  double speed = kp * error + ki * integral + kd * (error - prev_error);

  // 对速度取绝对值
  speed = std::abs(speed);

  // 使用 std::clamp 限幅
  speed = clamp(speed, 0.0, max_speed);

  // 更新前一次误差
  prev_error = error;

  return speed;
}

double turn_on_robot::CaremaSpeedControl1(int target_pose, int current_pose)
{
  // 静态变量用于存储 PID 状态
  static double prev_error = 0.0;
  static double integral = 0.0;
  // static ros::Time last_t = ros::Time::now();

  static constexpr double kp = 100.0;         // 稳定的比例增益
  static constexpr double ki = 0.0;         // 更低的积分增益
  static constexpr double kd = 0.03;         // 减小的微分增益
  static constexpr double max_speed = 1000.0; // 最大速度限制

  // auto now_t = ros::Time::now();
  // auto dt = (now_t - last_t).toSec();

  // 计算误差
  double error = static_cast<double>(target_pose - current_pose);

  integral += error;
  // // 更新积分项（带限幅）
  // integral += error * sampling_time;
  // integral = clamp(integral, -max_speed / (2.0 * ki), max_speed / (2.0 * ki));

  double speed = kp * error + ki * integral + kd * (error - prev_error);

  // 对速度取绝对值
  speed = std::abs(speed);

  // 使用 std::clamp 限幅
  speed = clamp(speed, 0.0, max_speed);

  // 更新前一次误差
  prev_error = error;

  return speed;
}

void turn_on_robot::sendCarInfoKernel()
{
  unsigned char buf[23] = {0};
  buf[0] = 0xa5; // hdr1
  buf[1] = 0x5a; // hdr2
  buf[2] = 20;   // len - 数据长度 22 - 3 = 19字节
  // 角度 TargetAngle 转换成整形后放入 buf[3] 和 buf[4]
  int16_t angle = static_cast<int16_t>(moveBaseControl.TargetAngle * 10); // 假设角度以0.1度为单位
  buf[3] = (angle >> 8) & 0xFF;                                           // 高字节
  buf[4] = angle & 0xFF;                                                  // 低字节
  // 速度符号 SpeedDirection 放入 buf[5]
  buf[5] = static_cast<unsigned char>(moveBaseControl.SpeedDirection);
  // printf("moveBaseControl.TargetSpeed = %d\n",moveBaseControl.TargetSpeed);
  // 目标速度 TargetSpeed 放入 buf[6]（占1字节）
  buf[6] = static_cast<unsigned char>(moveBaseControl.TargetSpeed);

  // 备用
  buf[7] = 0x00;
  // 云台电机0位置 Position_O 放入 buf[8] 和 buf[9]
  buf[8] = (moveBaseControl.Position_0 >> 8) & 0xFF; // 高字节
  buf[9] = moveBaseControl.Position_0 & 0xFF;        // 低字节
  // 云台电机1位置 Position_1 放入 buf[10] 和 buf[11]
  buf[10] = (moveBaseControl.Position_1 >> 8) & 0xFF; // 高字节
  buf[11] = moveBaseControl.Position_1 & 0xFF;        // 低字节
  // 云台电机0速度 Speed_0 放入 buf[12] 和 buf[13]
  buf[12] = (moveBaseControl.Speed_0 >> 8) & 0xFF; // 高字节
  buf[13] = moveBaseControl.Speed_0 & 0xFF;        // 低字节
  // 云台电机1速度 Speed_1 放入 buf[14] 和 buf[15]
  buf[14] = (moveBaseControl.Speed_1 >> 8) & 0xFF; // 高字节
  buf[15] = moveBaseControl.Speed_1 & 0xFF;        // 低字节
  // 云台电机0时间 Time_0 放入 buf[16]
  buf[16] = static_cast<unsigned char>(moveBaseControl.Time_0);
  // 云台电机1时间 Time_1 放入 buf[17]
  buf[17] = static_cast<unsigned char>(moveBaseControl.Time_1);
  // 功能模式
  buf[18] = static_cast<unsigned char>(moveBaseControl.Fun);
  // 使用ID
  buf[19] = static_cast<unsigned char>(moveBaseControl.Orin_ID);
  // 重置ID
  buf[20] = static_cast<unsigned char>(moveBaseControl.Set_ID_Num);
  // std::cout << " moveBaseControl.Position_0  =" << moveBaseControl.Position_0 << " moveBaseControl.Position_1 = " << moveBaseControl.Position_1 << std::endl;

  // 预留位 Reservel
  buf[21] = static_cast<unsigned char>(moveBaseControl.VoiceSwitch);
  // 校验和
  unsigned char sum = 0;
  for (int i = 2; i < 22; ++i) // 从 buf[2] 到 buf[18] 计算校验和
    sum += buf[i];
  buf[22] = sum;

  // for (int j = 0; j < 24; j++)
  // {
  //   printf("buf[%d] = %d ,", j, buf[j]);
  // }
  // printf("\n");
  // 通过串口发送数据
  try
  {
    Stm32_Serial.write(buf, 23); // 发送22字节的数据
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); // 如果发送数据失败，打印错误信息
  }
}

/**************************************
Date: January 28, 2021
Function: Publish the IMU data topic
功能: 发布IMU数据话题
***************************************/
void turn_on_robot::Publish_ImuSensor()
{
  static ros::Time first_receive_time = ros::Time::now();
  if (ros::Time::now() - first_receive_time < ros::Duration(0.2))
  {
    return;
  }
  static sensor_msgs::Imu Imu_Data_Pub; // Instantiate IMU topic data //实例化IMU话题数据
  if (Imu_Data_Pub.linear_acceleration.x == Mpu6050.linear_acceleration.x &&
      Imu_Data_Pub.linear_acceleration.y == Mpu6050.linear_acceleration.y &&
      Imu_Data_Pub.linear_acceleration.z == Mpu6050.linear_acceleration.z &&
      Imu_Data_Pub.angular_velocity.x == Mpu6050.angular_velocity.x &&
      Imu_Data_Pub.angular_velocity.y == Mpu6050.angular_velocity.y &&
      Imu_Data_Pub.angular_velocity.z == Mpu6050.angular_velocity.z)
  {
    return; // If the data is the same, do not publish //如果数据相同，不发布
  }
  Imu_Data_Pub.header.stamp = serial_received_time;
  Imu_Data_Pub.header.frame_id = gyro_frame_id;       // IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack                                                // IMU对应TF坐标，使用robot_pose_ekf功能包需要设置此项
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x; // A quaternion represents a three-axis attitude //四元数表达三轴姿态
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;

  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x; // Triaxial angular velocity //三轴角速度
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;

  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; // Triaxial acceleration //三轴线性加速度
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;
  // printf("1---- x=%.2f,y=%.2f,z=%.2f,\n",Imu_Data_Pub.linear_acceleration.x,Imu_Data_Pub.linear_acceleration.y,Imu_Data_Pub.linear_acceleration.z);

  if (montion_flag)
  {
    Imu_Data_Pub.orientation_covariance[0] = 1e6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e-6;

    Imu_Data_Pub.linear_acceleration_covariance[0] = 1e-6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    Imu_Data_Pub.linear_acceleration_covariance[4] = 1e-6;
    Imu_Data_Pub.linear_acceleration_covariance[8] = 1e6;

    Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; // Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  }
  else
  {
    Imu_Data_Pub.orientation_covariance[0] = 1e6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e6;

    Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; // Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[8] = 1e6;
    Imu_Data_Pub.linear_acceleration_covariance[0] = 1e6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    Imu_Data_Pub.linear_acceleration_covariance[4] = 1e6;
    Imu_Data_Pub.linear_acceleration_covariance[8] = 1e6;
  }

  imu_publisher.publish(Imu_Data_Pub); // Pub IMU topic //发布IMU话题

  if (!imu_initialized) // 初始化imu倾斜角
  {
    accumulate_initial_acceleration(Mpu6050.linear_acceleration);
  }
  else // imu_initialized == true
  {
    sensor_msgs::Imu Imu_Data_Corrected = Imu_Data_Pub; // 实例化修正后的IMU数据 // Instantiate corrected IMU data
    // 修正加速度
    tf2::Vector3 acc_raw(
        Mpu6050.linear_acceleration.x,
        Mpu6050.linear_acceleration.y,
        Mpu6050.linear_acceleration.z);
    tf2::Vector3 acc_corrected = tf2::quatRotate(q_correction, acc_raw);
    Imu_Data_Corrected.linear_acceleration.x = acc_corrected.x();
    Imu_Data_Corrected.linear_acceleration.y = acc_corrected.y();
    Imu_Data_Corrected.linear_acceleration.z = acc_corrected.z();

    // 修正姿态
    tf2::Quaternion q_raw;
    tf2::fromMsg(Mpu6050.orientation, q_raw);
    tf2::Quaternion q_final = q_correction * q_raw;
    q_final.normalize();
    Imu_Data_Corrected.orientation = tf2::toMsg(q_final);

    // 原始角速度不变
    Imu_Data_Corrected.angular_velocity = Mpu6050.angular_velocity;

    imu_correct_publisher.publish(Imu_Data_Corrected);
  }
  // std_msgs::UInt8 imu_msg_valid;
  // imu_msg_valid.data = 1;
  // pub_imu_msg_valid.publish(imu_msg_valid);

  // if (imu_flag == 1)
  // {

  // imu_buff.push_back(Imu_Data_Pub);
  // imu_correct = Imu_Data_Pub;
  // Average_filtering();
  // }
  // else
  // {

  // imu_state.push_back(Imu_Data_Pub);
  // Cal_state_error();
  // }
}

void turn_on_robot::accumulate_initial_acceleration(const geometry_msgs::Vector3 &acc)
{
  if (imu_init_count == 0)
    q_correction.getIdentity();
  acc_sum += tf2::Vector3(acc.x, acc.y, acc.z);
  imu_init_count++;

  if (imu_init_count >= INIT_FRAME_LIMIT)
  {
    tf2::Vector3 g_measured = acc_sum / imu_init_count;
    g_measured.normalize();

    tf2::Vector3 g_target(0, 0, 1); // 目标方向：重力朝下

    tf2::Vector3 axis = g_measured.cross(g_target);
    double angle = std::acos(g_measured.dot(g_target));
    if (axis.length() < 1e-6)
    {
      q_correction.getIdentity();
    }
    else
    {
      axis.normalize();
      q_correction.setRotation(axis, angle);
    }

    imu_initialized = true;
    ROS_INFO("通过加速度完成imu倾角补偿！");
    // 输出修正后的四元数
    ROS_INFO_STREAM("修正后的四元数: " << q_correction.x() << ", " << q_correction.y() << ", " << q_correction.z() << ", " << q_correction.w());
  }
}

double getYaw(geometry_msgs::PoseStamped pose)
{
  return tf2::getYaw(pose.pose.orientation);
}

/**************************************
Date: January 28, 2021
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/
void turn_on_robot::Publish_Odom()
{
  // Convert the Z-axis rotation Angle into a quaternion for expression
  // 把Z轴转角转换为四元数进行表达
  // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);

  if ((Robot_Pos.X == NAN) || (Robot_Pos.Y == NAN) || (Robot_Pos.Z == NAN))
  {
    printf("data no effect!\n");
    return;
  }

  nav_msgs::Odometry odom; // Instance the odometer topic data //实例化里程计话题数据
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);

  odom.header.stamp = serial_received_time;
  odom.header.frame_id = odom_frame_id;    // Odometer TF parent coordinates //里程计TF父坐标
  odom.pose.pose.position.x = Robot_Pos.X; // Position //位置
  odom.pose.pose.position.y = Robot_Pos.Y;
  odom.pose.pose.position.z = 0.2;
  odom.pose.pose.orientation = odom_quat; // Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数
  // odom.pose.pose.orientation.x = Mpu6050.orientation.x;
  // odom.pose.pose.orientation.y = Mpu6050.orientation.y;
  // odom.pose.pose.orientation.z = Mpu6050.orientation.z;
  // odom.pose.pose.orientation.w = Mpu6050.orientation.w;

  odom.child_frame_id = robot_frame_id;     // Odometer TF subcoordinates //里程计TF子坐标
  odom.twist.twist.linear.x = linear_Speed; // Speed in the X direction //X方向速度
  odom.twist.twist.linear.y = 0.0;          // Speed in the Y direction //Y方向速度
  odom.twist.twist.angular.z = ThetaSpeed;  // Angular velocity around the Z axis //绕Z轴角速度

  // There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
  // 这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
  if (Robot_Vel.Left == 0 && Robot_Vel.Right == 0)
    // If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
    // 如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
    memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
        memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
  else
    // If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
    // 如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
    memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
        memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
  // printf("odom_publisher--> x=%.2f,y=%.2f,z=%.2f\n",odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z );
  odom_publisher.publish(std::move(odom)); // Pub odometer topic //发布里程计话题
  // std_msgs::UInt8 odom_msg_valid;
  // odom_msg_valid.data = 1;
  // pub_odom_msg_valid.publish(odom_msg_valid);

  // publish the transform over tf add by jiaoyang 2022-12-27 start
  // geometry_msgs::TransformStamped odom_trans;
  // odom_trans.header.stamp = ros::Time::now();
  // odom_trans.header.frame_id = odom_frame_id;
  // odom_trans.child_frame_id = robot_frame_id;

  // odom_trans.transform.translation.x = Robot_Pos.X;
  // odom_trans.transform.translation.y = Robot_Pos.Y;
  // odom_trans.transform.translation.z = 0;
  // odom_trans.transform.rotation = odom_quat;
  // odom_trans.transform.rotation.x = Mpu6050.orientation.x;
  // odom_trans.transform.rotation.y = Mpu6050.orientation.y;
  // odom_trans.transform.rotation.z = Mpu6050.orientation.z;
  // odom_trans.transform.rotation.w = Mpu6050.orientation.w;

  // add by jiaoyang 2022-12-27 end
}

/**************************************
Date: January 28, 2021
Function: Publish voltage-related information
功能: 发布电压相关信息
***************************************/
void turn_on_robot::Publish_Voltage()
{
  std_msgs::Float32 voltage_msgs;
  voltage_msgs.data = Power_voltage;       // The power supply voltage is obtained //电源供电的电压获取
  voltage_publisher.publish(voltage_msgs); // Post the power supply voltage topic unit: V, volt //发布电源电压话题单位：V、伏特
  // printf("voltage_publisher->publish=%.5f\n",Power_voltage);
}

/**************************************
Date: February 16, 2021
Function: Publish Battery_Percentage information
功能: 发布电量百分比信息
***************************************/
double v;
bool flag = false;
int N = 10;
int Index = 0;
std::vector<double> xx(N, 0);
void turn_on_robot::Publish_Battery_Percentage()
{
  v = Power_voltage;
  // v = 13;

  float x;
  // float a, b, c, d, x, p, q, s1, s2, s3;
  // a = 0.33301984;
  // b = -1.31139425;
  // c = 1.87580217;
  // // d = 10.85200718;
  // d = 11.1;
  // d -= v;
  // // printf("Power_voltage =%.2f,d=%.2f\n",Power_voltage,d);
  // p = (3 * a * c - b * b) / (3 * a * a);
  // q = (27 * a * a * d - 9 * a * b * c + 2 * b * b * b) / (27 * a * a * a);
  // s1 = -b / (3 * a);
  // if (-q / 2 + sqrt(q * q / 4 + p * p * p / 27) > 0)
  // {
  //   s2 = pow((-q / 2 + sqrt(q * q / 4 + p * p * p / 27)), 1.0 / 3);
  // }
  // else
  // {
  //   s2 = -pow(-(-q / 2 + sqrt(q * q / 4 + p * p * p / 27)), 1.0 / 3);
  // }
  // if (-q / 2 - sqrt(q * q / 4 + p * p * p / 27) > 0)
  // {
  //   s3 = pow((-q / 2 - sqrt(q * q / 4 + p * p * p / 27)), 1.0 / 3);
  // }
  // else
  // {
  //   s3 = -pow(-(-q / 2 - sqrt(q * q / 4 + p * p * p / 27)), 1.0 / 3);
  // }
  // x = s1 + s2 + s3;
  // x = (x + 0.6468011925188) / 2.592234808167281;
  // if (x > 1.0 && x < 0.0)
  // {
  //   if (v > 11)
  //     x = 0.10;
  //   else
  //     x = 0.01;
  // }

  if (v >= Power_max)
  {
    x = 1;
  }
  else if (v > Power_min)
  {
    x = (v - Power_min) / (Power_max - Power_min);
  }
  else if (v <= Power_min)
  {
    x = 0;
  }

  xx[Index] = x;
  ++Index;
  Index %= N;

  // std::time_t nowtime = std::time(NULL) - starttime; // 获取时间戳
  // out_txt_file.open("power_voltage.txt", ios::out | ios::trunc);
  // out_txt_file << fixed;
  // out_txt_file << nowtime;
  // out_txt_file << " ";
  // out_txt_file << setprecision(4) << v;
  // out_txt_file << endl;
  // std::cout << "nowtime= " << nowtime << std::endl;
  if (count_A < 100)
    count_A++;

  float temp = 0;
  for (int i = 0; i < N; ++i)
    temp += xx[i];

  double cur_Battery_Percentage;
  cur_Battery_Percentage = temp / N; // 设置数据字段
  if (cur_Battery_Percentage > 1)
    cur_Battery_Percentage = 1;
  // printf("cur_Battery_Percentage=%.2f,--count_A=%d\n",cur_Battery_Percentage,count_A);
  // 保证电量百分比稳定下降，上升时只有连续比历史值高100次，才上升
  if (count_A > 30)
  {
    if (cur_Battery_Percentage > last_Battery_Percentage)
    {
      count_B++;
      if (count_B > 100)
      {
        last_Battery_Percentage = cur_Battery_Percentage;
        count_B = 0;
        count_C = 0;
      }
    }
    else
      count_B = 0;

    if (count_B > 200)
      count_B = 0;
    // 下降时低10次就下降
    if (cur_Battery_Percentage < last_Battery_Percentage)
    {
      count_C++;
      if (count_C > 10)
      {
        last_Battery_Percentage = cur_Battery_Percentage;
        count_B = 0;
        count_C = 0;
      }
    }
    else
      count_C = 0;
  }
  else if (last_Battery_Percentage < cur_Battery_Percentage)
  {
    last_Battery_Percentage = cur_Battery_Percentage;
  }

  Battery_Percentage_msgs.data = last_Battery_Percentage;

  Battery_Percentage_pub.publish(Battery_Percentage_msgs); // 发布电池电量百分比
  // printf("last_Battery_Percentage=%f\n",last_Battery_Percentage);

  // std::cout << "电压" << v << "==>"
  //           << "电量百分比： " << Battery_Percentage_msgs.data << std::endl;
}

/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BCC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/

unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
  unsigned char check_sum = 0, k;

  if (mode == 0) // Receive data mode //接收数据模式
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Receive_Data.rx[k]; // By bit or by bit //按位异或
    }
  }
  if (mode == 1) // Send data mode //发送数据模式
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Send_Data.tx[k]; // By bit or by bit //按位异或
    }
  }
  return check_sum; // Returns the bitwise XOR result //返回按位异或结果
}

/**************************************
Date: November 18, 2021
Function: Read and verify the data sent by the lower computer frame by frame through the serial port, and then convert the data into international units
功能: 通过串口读取并逐帧校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/
bool turn_on_robot::Get_Sensor_Data_New()
{
  serial_received_time = ros::Time::now(); // 记录接收数据的时间
  uint8_t head_Receive_Data[1] = {0};      // 临时变量，保存下位机数据
  Stm32_Serial.read(head_Receive_Data, 1); // 通过串口读取下位机发送的数据

  if (head_Receive_Data[0] != 0x7b)
    return false;

  uint8_t len_Receive_Data[1] = {0};      // 临时变量，保存下位机数据
  Stm32_Serial.read(len_Receive_Data, 1); // 通过串口读取下位机发送的数据

  if (len_Receive_Data[0] != 0x21)
    return false;

  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE - 2] = {0};        // 临时变量，保存下位机数据
  Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr)); // 通过串口读取下位机发送的数据

  Receive_Data.rx[0] = head_Receive_Data[0];
  Receive_Data.rx[1] = len_Receive_Data[0];
  for (int j = 0; j < sizeof(Receive_Data_Pr); j++)
  {
    Receive_Data.rx[j + 2] = Receive_Data_Pr[j];
  }

  // for (int kk = 0; kk < sizeof(Receive_Data_Pr); kk++)
  // {
  //   printf("[%d]=%02x,", kk, Receive_Data_Pr[kk]);
  // }
  // printf(" \n");
  // for (int kk = 0; kk < sizeof(Receive_Data.rx); kk++)
  // {
  //   printf("[%d]=%02x,", kk, Receive_Data.rx[kk]);
  // }
  // printf(" -----------------------------\n");

  // 校验和验证
  // if (Receive_Data.rx[33] != Check_Sum(33, READ_DATA_CHECK))
  // {
  //   printf("Check_Sum false  ----------\n");
  //   return false; // 校验和错误
  // }

  // 开始解析各个数据字段

  // 解析左轮和右轮速度
  Robot_Vel.Left = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]) / 1000.0;  // 左轮速度
  Robot_Vel.Right = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]) / 1000.0; // 右轮速度
  // std::cout << "ticks.Left = " << Robot_Vel.Left << ", ticks.Right = " << Robot_Vel.Right << std::endl;
  Robot_Vel.Steering_Angle = Odom_Trans(Receive_Data.rx[32], Receive_Data.rx[33]) / 100.0; // 转向角度

  // 解析电池电压
  short transition_16 = (Receive_Data.rx[6] << 8) | Receive_Data.rx[7];
  Power_voltage = transition_16 / 1000.0; // 单位转换为伏特

  // 解析IMU加速度数据
  Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);   // X轴加速度
  Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]); // Y轴加速度
  Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]); // Z轴加速度

  // 解析陀螺仪数据
  Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]); // X轴角速度
  Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]); // Y轴角速度
  Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]); // Z轴角速度

  // 加速度和角速度的单位转换
  constexpr double ACCEL_SCALE = 24.0 * 9.8 / 32768.0;
  constexpr double GYRO_SCALE = 1000.0 / (32768.0 * 57.3);
  Mpu6050.linear_acceleration.x = static_cast<double>(Mpu6050_Data.accele_x_data) * ACCEL_SCALE;
  Mpu6050.linear_acceleration.y = static_cast<double>(Mpu6050_Data.accele_y_data) * ACCEL_SCALE;
  Mpu6050.linear_acceleration.z = static_cast<double>(Mpu6050_Data.accele_z_data) * ACCEL_SCALE;

  Mpu6050.angular_velocity.x = static_cast<double>(Mpu6050_Data.gyros_x_data) * GYRO_SCALE;
  Mpu6050.angular_velocity.y = static_cast<double>(Mpu6050_Data.gyros_y_data) * GYRO_SCALE;
  Mpu6050.angular_velocity.z = static_cast<double>(Mpu6050_Data.gyros_z_data) * GYRO_SCALE;

  // 解析位置、速度和负载数据
  // 位置 1
  curYuntai_feedback_data.Position_0 = Odom_Trans(Receive_Data.rx[20], Receive_Data.rx[21]);
  // 速度 1
  curYuntai_feedback_data.Speed_0 = Odom_Trans(Receive_Data.rx[22], Receive_Data.rx[23]);
  // 负载 1
  curYuntai_feedback_data.Load_0 = Odom_Trans(Receive_Data.rx[24], Receive_Data.rx[25]);
  // printf("Position 0: %d, Speed 1: %d, Load 1: %d\n", curYuntai_feedback_data.Position_0, curYuntai_feedback_data.Speed_0, curYuntai_feedback_data.Load_0);

  // 位置 2
  curYuntai_feedback_data.Position_1 = Odom_Trans(Receive_Data.rx[26], Receive_Data.rx[27]);
  // 速度 2
  curYuntai_feedback_data.Speed_1 = Odom_Trans(Receive_Data.rx[28], Receive_Data.rx[29]);
  // 负载 2
  curYuntai_feedback_data.Load_1 = Odom_Trans(Receive_Data.rx[30], Receive_Data.rx[31]);
  // printf("Position 1: %d, Speed 2: %d, Load 2: %d\n", curYuntai_feedback_data.Position_1, curYuntai_feedback_data.Speed_1, curYuntai_feedback_data.Load_1);

  // chassis_mode_ = Receive_Data.rx[34]; // 解析小车模式
  std_msgs::UInt8 chassis_mode_msg;
  chassis_mode_msg.data = Receive_Data.rx[34]; // 将小车模式数据封装到消息中
  pub_chassis_mode.publish(chassis_mode_msg);  // 发布小车模式消息
  // std::cout << "chassis mode: " << static_cast<int>(chassis_mode_) << std::endl;
  // 处理预留位
  // Receive_Data.Flag_Reserved = Receive_Data.rx[32]; // 预留位

  // 最后返回true，表示数据解析成功
  return true;
}

void turn_on_robot::Publish_Gimbal_Feedback()
{
  // 发布云台反馈数据
  zllc_msgs::Motors gimbal_feedback_msg;
  gimbal_feedback_msg.header.stamp = serial_received_time;
  gimbal_feedback_msg.motor0.position = curYuntai_feedback_data.Position_0;
  gimbal_feedback_msg.motor0.speed = curYuntai_feedback_data.Speed_0;
  gimbal_feedback_msg.motor0.load = curYuntai_feedback_data.Load_0;

  gimbal_feedback_msg.motor1.position = curYuntai_feedback_data.Position_1;
  gimbal_feedback_msg.motor1.speed = curYuntai_feedback_data.Speed_1;
  gimbal_feedback_msg.motor1.load = curYuntai_feedback_data.Load_1;

  pub_gimbal_feedback.publish(gimbal_feedback_msg);
}

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "dzactuator");
  turn_on_robot Robot_Control;
  Robot_Control.Control();
  return 0;
}
