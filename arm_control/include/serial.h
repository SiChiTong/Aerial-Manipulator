
#include <errno.h>  // Error number definitions
#include <fcntl.h>  // File control definitions
#include <math.h>
#include <stdio.h>    // standard input / output functions
#include <string.h>   // string function definitions
#include <termios.h>  // POSIX terminal control definitionss
#include <unistd.h>   // UNIX standard function definitions
#include <iostream>

#define MSGTOMCU_SIZE 10
#define MSGFROMMCU_SIZE 8

struct joint
{
    float x = 0;
    float y = 0;
};

struct RobotMsgToMCU_write
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

struct RobotMsgToMCU_read
{
  char a = 0;
  char b = 0;

  char c = 0;
  char d = 0;
  char e = 0;

  char f = 0;
 
};


struct RobotMsgFromMCU 
{
  char a = 0;
  char b = 0;

  char c = 0;
  char d = 0;
  char e = 0;

  char f = 0;
  char g = 0;

  char h = 0;

};

class Serial {
 public:
  int fd;
  struct termios port_settings;
  Serial(const char* dev_name);
  ~Serial();

  void configurePort();
  bool SendData_write(struct RobotMsgToMCU_write msg);
  bool SendData_read(struct RobotMsgToMCU_read msg);

  bool ReadData( struct RobotMsgFromMCU& msg);
  bool ClearData(struct RobotMsgFromMCU& msg);
};







