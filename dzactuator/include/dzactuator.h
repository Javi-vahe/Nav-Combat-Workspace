#ifndef __dzactuator_H_
#define __dzactuator_H_
#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>
#include <stdbool.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <deque>
#include <Eigen/Dense>

using namespace std;

// Macro definition
// 宏定义

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

#define SEND_DATA_CHECK 1	 // Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK 0	 // Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER 0X7B	 // Frame head //帧头
#define FRAME_TAIL 0X7D		 // Frame tail //帧尾
#define RECEIVE_DATA_SIZE 37 // The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE 11	 // The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
// #define PI 				  3.1415926f //PI //圆周率

#define CARL 0.17
#define CARW 0.10
// Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
// The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
// 与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
// 陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO 0.00026644f
// Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
// Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84
// 与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
// 加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO 1671.84f

extern sensor_msgs::Imu Mpu6050; // External variables, IMU topic data //外部变量，IMU话题数据

// Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
// 协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
										 0, 1e-3, 0, 0, 0, 0,
										 0, 0, 1e6, 0, 0, 0,
										 0, 0, 0, 1e6, 0, 0,
										 0, 0, 0, 0, 1e6, 0,
										 0, 0, 0, 0, 0, 1e3};

const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
										  0, 1e-3, 1e-9, 0, 0, 0,
										  0, 0, 1e6, 0, 0, 0,
										  0, 0, 0, 1e6, 0, 0,
										  0, 0, 0, 0, 1e6, 0,
										  0, 0, 0, 0, 0, 1e-9};

const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
										  0, 1e-3, 0, 0, 0, 0,
										  0, 0, 1e-3, 0, 0, 0,
										  0, 0, 0, 1e6, 0, 0,
										  0, 0, 0, 0, 1e6, 0,
										  0, 0, 0, 0, 0, 1e-3};

const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
										   0, 1e-3, 1e-9, 0, 0, 0,
										   0, 0, 1e6, 0, 0, 0,
										   0, 0, 0, 1e6, 0, 0,
										   0, 0, 0, 0, 1e6, 0,
										   0, 0, 0, 0, 0, 1e-9};

// Data structure for position
// 位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
} Vel_Pos_Data;

// Data structure for speed
// 速度数据结构体
typedef struct __Vel_Speed_Data_
{
	float Left;
	float Right;
	float Steering_Angle; // Steering angle 转向角度，单位：angle
} Vel_Speed_Data;

// IMU data structure
// IMU数据结构体
typedef struct __MPU6050_DATA_
{
	short accele_x_data;
	short accele_y_data;
	short accele_z_data;
	short gyros_x_data;
	short gyros_y_data;
	short gyros_z_data;

} MPU6050_DATA;

// The structure of the ROS to send data to the down machine
// ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_
{
	uint8_t tx[SEND_DATA_SIZE];
	float X_speed;
	float Y_speed;
	float Z_speed;
	unsigned char Frame_Tail;
} SEND_DATA;

// The structure in which the lower computer sends data to the ROS
// 下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_
{
	uint8_t rx[RECEIVE_DATA_SIZE];
	uint8_t Flag_Stop;
	unsigned char Frame_Header;
	float X_speed;
	float Y_speed;
	float Z_speed;
	float Power_Voltage;
	unsigned char Frame_Tail;
} RECEIVE_DATA;

typedef struct sMartcarControl
{
	double TargetAngle; // 角度量
	int SpeedDirection; // 速度符号
	int TargetSpeed;	// 速度
	int LaserGunSwitch; // 激光开关
	int Position_0;		// 云台电机0位置
	int Position_1;		// 云台电机1位置
	int Speed_0;		// 云台电机0速度
	int Speed_1;		// 云台电机1速度
	int Time_0;			// 云台电机0时间
	int Time_1;			// 云台电机1时间
	int Fun;			// 功能函数  默认：0x00 设置ID：0x01 置中0x02
	int Orin_ID;		// 使用ID
	int Set_ID_Num;		// 重置ID
	int VoiceSwitch;	// 预留位
} sMartcarControl;

typedef struct curYuntai_feedback
{
	int Position_0; // 云台电机0位置
	int Position_1; // 云台电机1位置
	int Speed_0;	// 云台电机0速度
	int Speed_1;	// 云台电机1速度
	int Load_0;		// 云台电机0时间
	int Load_1;		// 云台电机1时间
} curYuntai_feedback;

template <typename T>
T clamp(T value, T min, T max)
{
	return value < min ? min : (value > max ? max : value);
}

struct HpAndHitMsg
{
	int enemy_hp = 10;		 // 对方血量
	int self_hp = 10;		 // 自身血量
	int being_hit = 16;		 // 是否被击打 0x01左方,0x02前方，0x03右方，0x04后方,0x10未被击打
	int shooting_Count = 0; // 已射击次数
	int ammo = 10;			 // 剩余弹药数量
	int last_being_hit = 16; // 上一次被击打的装甲板
	double last_being_hit_time = 0.0; // 上一次被击打的时间
}__attribute__((packed));

// The robot chassis class uses constructors to initialize data, publish topics, etc
// 机器人底盘类，使用构造函数初始化数据和发布话题等
class turn_on_robot
{
public:
	turn_on_robot();  // Constructor 构造函数
	~turn_on_robot(); // Destructor 析构函数

	ros::NodeHandle n; // Create a ROS node handle //创建ROS节点句柄

	void Control(); // Loop control code 循环控制代码

	ros::Subscriber sub_movebase_angle;
	ros::Subscriber sub_cmd_vel;
	ros::Subscriber sub_gimbal_cruise_flag;
	ros::Subscriber sub_voice_switch;
	ros::Subscriber sub_HpAndHitmsg;

	// The speed topic subscribes to the callback function
	// 速度话题订阅回调函数
	void callback_movebase_angle(const geometry_msgs::Twist::ConstPtr &msg);
	// void callback_cmd_vel_angle(const geometry_msgs::Twist::ConstPtr &msg);
	void callback_gimbal_cruise_flag(const std_msgs::UInt8::ConstPtr &msg);
	void callback_voice_switch(const std_msgs::UInt8::ConstPtr &msg);

	ros::Subscriber sub_motor_control;
	ros::Subscriber sub_offset_center;

	void callback_motor_control(const geometry_msgs::Twist::ConstPtr &msg);
	void callback_offset_center(const std_msgs::Int32MultiArray::ConstPtr &msg);
	void callback_HpAndHitmsg(const std_msgs::UInt8MultiArray::ConstPtr &msg);

	ros::Publisher odom_publisher;
	ros::Publisher pub_odom_msg_valid;
	ros::Publisher imu_publisher, imu_correct_publisher;
	ros::Publisher pub_imu_msg_valid;
	ros::Publisher voltage_publisher;
	ros::Publisher pub_diff;
	ros::Publisher Battery_Percentage_pub;
	ros::Publisher pub_LaserShot_Command;
	ros::Publisher pub_gimbal_feedback;
	ros::Publisher pub_chassis_mode;
	ros::Publisher take_off_pub_;   // 切换物资识别模型

	void Publish_Odom();			   // Pub the speedometer topic //发布里程计话题
	void Publish_ImuSensor();		   // Pub the IMU sensor topic //发布IMU传感器话题
	void Publish_Voltage();			   // Pub the power supply voltage topic //发布电源电压话题
	void Publish_Battery_Percentage(); // Pub the Battery Percentage topic //发布电量百分比话题
	void Publish_Gimbal_Feedback();	   // Pub the gimbal feedback topic //发布云台反馈话题
	void accumulate_initial_acceleration(const geometry_msgs::Vector3& acc);

	bool Get_Sensor_Data_New();
	unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode); // BBC check function //BBC校验函数
	short IMU_Trans(uint8_t Data_High, uint8_t Data_Low);					 // IMU data conversion read //IMU数据转化读取
	float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);					 // Odometer data is converted to read //里程计数据转化读取
	void sendCarInfoKernel();

	double normalizeAngle(double angle);
	double CaremaSpeedControl0(int target_pose, int current_pose);
	double CaremaSpeedControl1(int target_pose, int current_pose);
	void CaremaMotorControl();


	ros::Time serial_received_time, last_serial_received_time; // Serial port data receiving time //串口数据接收时间

public:
	serial::Serial Stm32_Serial; // Declare a serial object 声明串口对象

	string serial_port_name, robot_frame_id, gyro_frame_id, odom_frame_id; // Define the related variables //定义相关变量
	int serial_baud_rate;												   // Serial communication baud rate //串口通信波特率
	RECEIVE_DATA Receive_Data;											   // The serial port receives the data structure //串口接收数据结构体
	SEND_DATA Send_Data;												   // The serial port sends the data structure //串口发送数据结构体

public:
	Vel_Pos_Data Robot_Pos;	   // The position of the robot //机器人的位置
	Vel_Speed_Data Robot_Vel;  // The speed of the robot //机器人的速度
	MPU6050_DATA Mpu6050_Data; // IMU data //IMU数据
	float Power_voltage;	   // Power supply voltage //电源电压

	sMartcarControl moveBaseControl;
	int calibrate_lineSpeed;
	double ticksPerMeter;
	double ticksPer2PI;
	double linear_Speed;
	double ThetaSpeed;
	double leftDistance;
	double rightDistance;
	int gimbal_cruise_msg;
	bool find_center;
	int count_return_center;
	bool return_center;

	curYuntai_feedback curYuntai_feedback_data;

	// add for pub_Battery_Percentage 20230216 whq
	bool montion_flag = false; // 小车是否在运动

	double last_Battery_Percentage;
	int count_A, count_B, count_C;

	double Power_max, Power_min;
	std_msgs::Float32 Battery_Percentage_msgs;
	sensor_msgs::Imu imu_correct;
	//	imu倾角矫正
	int imu_init_count = 0;
	const int INIT_FRAME_LIMIT = 200;
	tf2::Vector3 acc_sum{0, 0, 0};
	bool imu_initialized = true;
	tf2::Quaternion q_correction{-0.00373589, -0.0114131, 0, 0.999928};

	HpAndHitMsg hp_and_hit_msg_; // 血量和被击打信息

	uint8_t chassis_mode_ = 1; // 底盘模式 0:上位机模式 1:下位机模式
	ros::Time custom_gimbal_cruise_time_; // 自定义云台巡航时间
};

#endif
