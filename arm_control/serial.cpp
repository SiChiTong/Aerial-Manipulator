#include "serial.h"

Serial::Serial(const char* dev_name) {
    fd = open(dev_name, O_RDWR | O_NONBLOCK);  //| O_NONBLOCK  O_NDELAY
    if (fd == -1) {
        printf("open_port: Unable to open /dev/ttySFDFDS. \n");
    } else {
        fcntl(fd, F_SETFL, 0);
        printf("%d  port is open.\n", fd);
    }
}

Serial::~Serial() { close(fd); }
void Serial::configurePort() {  // configure the port
  // structure to store the port settings in
  cfsetispeed(&port_settings, B115200);  // set baud rates
  cfsetospeed(&port_settings, B115200);

  port_settings.c_cflag &= ~PARENB;  // set no parity, stop bits, data bits
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;
  port_settings.c_cflag |= CLOCAL | CREAD;
  tcsetattr(fd, TCSANOW, &port_settings);  // apply the settings to the port
}

bool Serial::SendData_write(struct RobotMsgToMCU_write msg) {
  unsigned char send_bytes[MSGTOMCU_SIZE] = {0x00};

  unsigned char* ptr_send_bytes = send_bytes;
  memcpy(ptr_send_bytes, &msg, sizeof(msg));

  write(fd, send_bytes, 10);
}


bool Serial::SendData_read(struct RobotMsgToMCU_read msg) {
  unsigned char send_bytes[MSGTOMCU_SIZE] = {0x00};

  unsigned char* ptr_send_bytes = send_bytes;
  memcpy(ptr_send_bytes, &msg, sizeof(msg));

  write(fd, send_bytes, 6);
}

bool Serial::ReadData(struct RobotMsgFromMCU& msg) {
  char tmp[1] = {0x00};
  char buf[8] = {0x00};
  char* ptr = buf;
  int data_ready = 0;
  int data_start = 0;

  int cnt = 0;
  while (!data_ready) {
    int ret = read(fd, tmp, 1);
    
    if (ret == 0)
    {
        continue;
        //printf("ready\n");
    }

    *ptr = tmp[0];
     ptr++;

     memcpy(&msg, buf, sizeof(msg));
 
     cnt++;
     if(cnt >= MSGFROMMCU_SIZE) data_ready = 1;

  }

  return false;
}
 
bool Serial::ClearData(struct RobotMsgFromMCU& msg) {
  char tmp[1] = {0x00};
  char buf[8] = {0x00};
  char* ptr = buf;
  int data_ready = 0;
  int data_start = 0;

  int cnt = 0;
  while (!data_ready) {
    int ret = read(fd, tmp, 1);

    if(ret != 0)
    {
        printf("clearing\n");
    }
    else
    {
        data_ready = 1;        
        printf("clear is done\n");
    }

  }

  return false;
}
 


















