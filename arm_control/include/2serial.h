
#include <errno.h>  // Error number definitions
#include <fcntl.h>  // File control definitions
#include <math.h>
#include <stdio.h>    // standard input / output functions
#include <string.h>   // string function definitions
#include <termios.h>  // POSIX terminal control definitionss
#include <unistd.h>   // UNIX standard function definitions

#define MSGTOMCU_SIZE 10
#define MSGFROMMCU_SIZE 34
struct RobotMsgToMCU 
{
  char a = 0;
  char b = 0;

  char c = 0;
  char d = 0;
  char e = 0;

  char f = 0;
  char g = 0;

  char h = 0;
  char i = 0;

  char j = 0;
};

struct RobotMsgFromMCU {
  short int remaining_HP = 0;
  short int attack_armorID = 0;
  short int remaining_bullet = 0;
  short int uwb_x = 0;
  short int uwb_y = 0;
  unsigned short int uwb_yaw = 0;
  short int gimbal_chassis_angle = 0;
  short int gimbal_pitch_angle = 0;
  short int imu_acceleration_x = 0;
  short int imu_acceleration_y = 0;
  short int imu_acceleration_z = 0;
  short int imu_velocity_x = 0;
  short int imu_velocity_y = 0;
  short int imu_velocity_z = 0;
  short int wheel_odom_x = 0;
  short int wheel_odom_y = 0;
  char uwb_ready_flag = 0;
  char init_flag=0;
};

class Serial {
 public:
  int fd;
  struct termios port_settings;
  Serial(const char* dev_name);
  ~Serial();

  void configurePort();
  bool SendData(struct RobotMsgToMCU msg);
  bool ReadData(struct RobotMsgFromMCU& msg);
};



