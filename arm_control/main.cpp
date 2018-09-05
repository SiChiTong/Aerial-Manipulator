#include <ros/ros.h> 
#include <iostream>
#include <stdlib.h>
#include <serial.h>
#include <unistd.h>
#include <time.h>
#include <ctime>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

#define L1 30 
#define L2 60
#define L3 20

#define Pi 3.14159265
#define angle_posi_para 375 * 2 / Pi

#define vertical_posi_ID_1 50
#define vertical_posi_ID_2 555
#define vertical_posi_ID_3 700

#define y_min 3.13585

#define position_ID_1_intial 380   // 80
#define position_ID_2_intial 817
#define position_ID_3_intial 118
#define position_ID_4_intial 0
#define position_ID_8_intial 1000 //0
#define position_ID_9_intial 207 //207

#define initial_succeed_cnt 40

//speed control
#define ID_1_speed_set 1
#define ID_3_speed_set 1
#define ID_4_speed_set 1
#define ID_6_speed_set_initial 800
#define ID_8_speed_set 10000 
#define ID_9_speed_set 1 

//module funtion flag:
// #define initial_flag    1
#define loop_start_flag 1

int fabs(int a)
{
    if(a >= 0)
        return a;
    else
        return -a;
}

typedef nav_msgs::Odometry Odometry;
double odom_pitch, odom_roll, odom_yaw;
double init_odom_roll, init_odom_pitch, init_odom_yaw;
char ODOM_INIT = 1;

void callback_odom(const Odometry::ConstPtr &odom)
{
  //ROS_INFO("I heard odom");
  if(ODOM_INIT)
  {
    tf::Quaternion odom_qn(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    tf::Matrix3x3 m(odom_qn);
    m.getRPY(init_odom_roll, init_odom_pitch, init_odom_yaw);
    ODOM_INIT = 0;
  }
  else
  {
    tf::Quaternion odom_qn(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    tf::Matrix3x3 m(odom_qn);
    m.getRPY(odom_roll, odom_pitch, odom_yaw);

    odom_roll = odom_roll - init_odom_roll;
    odom_pitch = odom_pitch - init_odom_pitch;
    odom_yaw = odom_yaw - init_odom_yaw;

    if (odom_roll < -M_PI)
        odom_roll = odom_roll + 2 * M_PI;
    if (odom_roll > M_PI)
        odom_roll = odom_roll - 2 * M_PI;

    if (odom_pitch < -M_PI)
        odom_pitch = odom_pitch + 2 * M_PI;
    if (odom_pitch > M_PI)
        odom_pitch = odom_pitch - 2 * M_PI;

    if (odom_yaw < -M_PI)
        odom_yaw = odom_yaw + 2 * M_PI;
    if (odom_yaw > M_PI)
        odom_yaw = odom_yaw - 2 * M_PI;

    std::cout << "odom_roll: " << odom_roll << "  " << "odom_pitch: " << odom_pitch << endl ;//"odom_yaw: " << odom_yaw;
  }

}

int main ( int argc , char ** argv ) 
{
    ros::init(argc, argv, "arm_control");
    ros::NodeHandle nh;

    ros::Subscriber sub_odom = nh.subscribe<Odometry>("mavros/local_position/odom", 1, callback_odom);

    //initial serial:
    Serial serial("/dev/com");
    serial.configurePort();
 
    struct RobotMsgToMCU_write Ctr_ID_1 , Ctr_ID_speed_1 , Ctr_ID_3 , Ctr_ID_4 , Ctr_ID_5 , Ctr_ID_6 , Ctr_ID_8 , Ctr_ID_9;
    struct RobotMsgToMCU_read Read_ID_1 , Read_ID_2 ;
    struct RobotMsgFromMCU ID_Clear , ID_1 , ID_2;


    //rotate_angle:
    double real_theta_1 = 0 , real_theta_2 = 0;
    double tar_theta_1 = 0, tar_theta_2 = 0 , tar_theta_3 = 0;


    //rotate_position:
    int position_ID_1 = position_ID_1_intial , position_ID_2 = 0 , position_ID_3 = position_ID_3_intial , position_ID_4 = position_ID_4_intial , position_ID_8 = position_ID_8_intial , position_ID_9 = position_ID_9_intial;


    float tar_position_ID_1 = position_ID_1_intial , tar_position_ID_2  = 0 ; 


    //joint coordinate:
    joint J3;



    //shut down all motor
    //1
    int time_1 = ID_1_speed_set;
    char high_bit_1 , low_bit_1;
    char High_bit_1 , Low_bit_1;
    int Checksum_1 = 0;

    char high_bit_speed_1 , low_bit_speed_1;
    int  Checksum_speed_1 = 0;
    int  speed_ID_speed_1 = 0 ;// + shou

    //2
    char high_bit_2 , low_bit_2;
    int Checksum_2;

    //3
    int time_3 = ID_3_speed_set;
    char high_bit_3 , low_bit_3;
    char High_bit_3 , Low_bit_3;
    int Checksum_3 = 0;

    //4
    int time_4 = ID_4_speed_set;
    char high_bit_4 , low_bit_4;
    char High_bit_4 , Low_bit_4;
    int Checksum_4 = 0;

    //5
    char high_bit_5 , low_bit_5;
    int  Checksum_5 = 0;
    int  speed_ID_5 = -200 ;// + shou

    //6
    char high_bit_6 , low_bit_6;
    int  Checksum_6 = 0;
    int  speed_ID_6 = 0 ;// + shou
    int ID_6_speed_set = 0;

    //8
    int time_8 = ID_8_speed_set;
    char high_bit_8 , low_bit_8;
    char High_bit_8 , Low_bit_8;
    int Checksum_8 = 0;

    //9
    int time_9 = ID_9_speed_set;
    char high_bit_9 , low_bit_9;
    char High_bit_9 , Low_bit_9;
    int Checksum_9 = 0;

    //ID_1 control:
    int ID_1_ctrl_flag = 0;

    //ID_3 control:
    int ID_3_ctrl_flag = 0;

    //ID_4 control:
    int ID_4_ctrl_flag = 0;

    //ID_5 control:
    int ID_5_ctrl_flag = 0;

    //ID_6 control:
    int ID_6_ctrl_flag = 0 , ID_6_ctrl_flag_last = 0;

    //ID_8 control:
    int ID_8_ctrl_flag = 0;
    ///////////////////////////////////////////////////////////////////////////////////////


    high_bit_6 = (speed_ID_6 >> 8) & 0xff; //high 8-bit
    low_bit_6  = speed_ID_6 & 0xff;        //low  8-bit
    Checksum_6 = 0xFF &( ~(0x06 + 0x07 + 0x1d + 0x01 + 0x00 + low_bit_6 + high_bit_6 ) );
    Ctr_ID_6.a = 0x55;
    Ctr_ID_6.b = 0x55;
    Ctr_ID_6.c = 0x06;
    Ctr_ID_6.d = 0x07;
    Ctr_ID_6.e = 0x1d;
    Ctr_ID_6.f = 0x01;
    Ctr_ID_6.g = 0x00;
    Ctr_ID_6.h = low_bit_6;
    Ctr_ID_6.i = high_bit_6;
    Ctr_ID_6.j = Checksum_6;
    serial.SendData_write(Ctr_ID_6);
    clock_t nr1 = clock();
    while( clock() - nr1 < 50000);//1000000

  
    high_bit_5 = (speed_ID_5 >> 8) & 0xff; //high 8-bit
    low_bit_5  = speed_ID_5 & 0xff;        //low  8-bit
    Checksum_5 = 0xFF &( ~(0x05 + 0x07 + 0x1d + 0x01 + 0x00 + low_bit_5 + high_bit_5 ) );
    Ctr_ID_5.a = 0x55;
    Ctr_ID_5.b = 0x55;
    Ctr_ID_5.c = 0x05;
    Ctr_ID_5.d = 0x07;
    Ctr_ID_5.e = 0x1d;
    Ctr_ID_5.f = 0x01;
    Ctr_ID_5.g = 0x00;
    Ctr_ID_5.h = low_bit_5;
    Ctr_ID_5.i = high_bit_5;
    Ctr_ID_5.j = Checksum_5;
    serial.SendData_write(Ctr_ID_5);
    clock_t nr213e1 = clock();
    while( clock() - nr213e1 < 50000);//1000000

    // //position intial:
    // ////////////////////////////////////////////////////////////////////////////////////////
    // /////////////////////////////////////////////////////////////////////

    serial.ClearData(ID_Clear); 
    serial.ClearData(ID_Clear); 
    serial.ClearData(ID_Clear); 
    serial.ClearData(ID_Clear); 
    serial.ClearData(ID_Clear); 

    int initial_cnt = 0;

    cout << "data is correct!!!!" << endl << endl;
    cout << "data is correct!!!!" << endl << endl;
    cout << "data is correct!!!!" << endl << endl;
  
 
    // //return 0;
    // //////////////////////////////////////////////////////////////////////////////////////////
    // 1:
    high_bit_1 = (position_ID_1 >> 8) & 0xff; //high 8-bit
    low_bit_1  = position_ID_1 & 0xff;        //low  8-bit
    High_bit_1 = (1 >> 8) & 0xff; //high 8-bit
    Low_bit_1  = 1 & 0xff;        //low  8-bit
    Checksum_1 = 0xFF &( ~(0x01 + 0x07 + 0x01 + low_bit_1 + high_bit_1 + Low_bit_1 + High_bit_1) );
    Ctr_ID_1.a = 0x55;
    Ctr_ID_1.b = 0x55;
    Ctr_ID_1.c = 0x01;
    Ctr_ID_1.d = 0x07;
    Ctr_ID_1.e = 0x01;
    Ctr_ID_1.f = low_bit_1;
    Ctr_ID_1.g = high_bit_1;
    Ctr_ID_1.h = Low_bit_1;
    Ctr_ID_1.i = High_bit_1;
    Ctr_ID_1.j = Checksum_1;
    serial.SendData_write(Ctr_ID_1);
    clock_t n1 = clock();
    while( clock() - n1 < 50000);//1000000

    high_bit_speed_1 = (0 >> 8) & 0xff; //high 8-bit
    low_bit_speed_1  =  0 & 0xff;        //low  8-bit
    Checksum_speed_1 = 0xFF &( ~(0x01 + 0x07 + 0x1d + 0x01 + 0x00 + low_bit_speed_1 + high_bit_speed_1 ) );
    Ctr_ID_speed_1.a = 0x55;
    Ctr_ID_speed_1.b = 0x55;
    Ctr_ID_speed_1.c = 0x01;
    Ctr_ID_speed_1.d = 0x07;
    Ctr_ID_speed_1.e = 0x1d;
    Ctr_ID_speed_1.f = 0x01;
    Ctr_ID_speed_1.g = 0x00;
    Ctr_ID_speed_1.h = low_bit_speed_1;
    Ctr_ID_speed_1.i = high_bit_speed_1;
    Ctr_ID_speed_1.j = Checksum_speed_1;

    //serial.SendData_write(Ctr_ID_speed_1);
    clock_t nfewefw33 = clock();
    while( clock() - nfewefw33 < 50000);

    cout << " initialing !!! " << endl;
    ///////////////////////////////////////////////////////////////////////////////////////



    ////////////////////////////////////////////////////////////////////////
    // 2:
    position_ID_2 = 5000;
    high_bit_2= (position_ID_2 >> 8) & 0xff; //high 8-bit
    low_bit_2 = position_ID_2 & 0xff;        //low  8-bit
    Checksum_2 = 0xFF &( ~(0x01 + 0x03 + 0x1C) );
    cout << " initialing !!! " << endl;                     
    //////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    // 3:
    high_bit_3 = (position_ID_3 >> 8) & 0xff; //high 8-bit
    low_bit_3  = position_ID_3 & 0xff;        //low  8-bit
    High_bit_3 = (2000 >> 8) & 0xff; //high 8-bit
    Low_bit_3  = 2000 & 0xff;        //low  8-bit
    Checksum_3 = 0xFF &( ~(0x03 + 0x07 + 0x01 + low_bit_3 + high_bit_3 + Low_bit_3 + High_bit_3) );
    Ctr_ID_3.a = 0x55;
    Ctr_ID_3.b = 0x55;
    Ctr_ID_3.c = 0x03;
    Ctr_ID_3.d = 0x07;
    Ctr_ID_3.e = 0x01;
    Ctr_ID_3.f = low_bit_3;
    Ctr_ID_3.g = high_bit_3;
    Ctr_ID_3.h = Low_bit_3;
    Ctr_ID_3.i = High_bit_3;
    Ctr_ID_3.j = Checksum_3;
    serial.SendData_write(Ctr_ID_3);
    clock_t nbv1 = clock();
    while( clock() - nbv1 < 50000);//1000000
    cout << " initialing !!! " << endl;
    ////////////////////////////////////////////////////////////////////////
    

    ////////////////////////////////////////////////////////////////////////
    // 4:
    high_bit_4 = (position_ID_4 >> 8) & 0xff; //high 8-bit
    low_bit_4  = position_ID_4 & 0xff;        //low  8-bit
    High_bit_4 = (time_4 >> 8) & 0xff; //high 8-bit
    Low_bit_4  = time_4 & 0xff;        //low  8-bit
    Checksum_4 = 0xFF &( ~(0x04 + 0x07 + 0x01 + low_bit_4 + high_bit_4 + Low_bit_4 + High_bit_4) );
    Ctr_ID_4.a = 0x55;
    Ctr_ID_4.b = 0x55;
    Ctr_ID_4.c = 0x04;
    Ctr_ID_4.d = 0x07;
    Ctr_ID_4.e = 0x01;
    Ctr_ID_4.f = low_bit_4;
    Ctr_ID_4.g = high_bit_4;
    Ctr_ID_4.h = Low_bit_4;
    Ctr_ID_4.i = High_bit_4;
    Ctr_ID_4.j = Checksum_4;
    serial.SendData_write(Ctr_ID_4);
    clock_t nb21v1 = clock();
    while( clock() - nb21v1 < 50000);//1000000
    cout << " initialing !!! " << endl;
    ////////////////////////////////////////////////////////////////////////
    

    ///////////////////////////////
    //8:
    high_bit_8 = (position_ID_8 >> 8) & 0xff; //high 8-bit
    low_bit_8  = position_ID_8 & 0xff;        //low  8-bit
    High_bit_8 = (time_8 >> 8) & 0xff; //high 8-bit
    Low_bit_8  = time_8 & 0xff;        //low  8-bit
    Checksum_8 = 0xFF &( ~(0x08 + 0x07 + 0x01 + low_bit_8 + high_bit_8 + Low_bit_8 + High_bit_8) );
    Ctr_ID_8.a = 0x55;
    Ctr_ID_8.b = 0x55;
    Ctr_ID_8.c = 0x08;
    Ctr_ID_8.d = 0x07;
    Ctr_ID_8.e = 0x01;
    Ctr_ID_8.f = low_bit_8;
    Ctr_ID_8.g = high_bit_8;
    Ctr_ID_8.h = Low_bit_8;
    Ctr_ID_8.i = High_bit_8;
    Ctr_ID_8.j = Checksum_8;
    serial.SendData_write(Ctr_ID_8);
    clock_t ndwq1 = clock();
    while( clock() - ndwq1 < 1000000);//1000000

    cout << " initialing !!! " << endl;
    /////////////////////////////////////////////////// 
            


    ///////////////////////////////
    //9:
    high_bit_9 = (position_ID_9 >> 8) & 0xff; //high 8-bit
    low_bit_9  = position_ID_9 & 0xff;        //low  8-bit
    High_bit_9 = (time_9 >> 8) & 0xff; //high 8-bit
    Low_bit_9  = time_9 & 0xff;        //low  8-bit
    Checksum_9 = 0xFF &( ~(0x09 + 0x07 + 0x01 + low_bit_9 + high_bit_9 + Low_bit_9 + High_bit_9) );
    Ctr_ID_9.a = 0x55;
    Ctr_ID_9.b = 0x55;
    Ctr_ID_9.c = 0x09;
    Ctr_ID_9.d = 0x07;
    Ctr_ID_9.e = 0x01;
    Ctr_ID_9.f = low_bit_9;
    Ctr_ID_9.g = high_bit_9;
    Ctr_ID_9.h = Low_bit_9;
    Ctr_ID_9.i = High_bit_9;
    Ctr_ID_9.j = Checksum_9;
    serial.SendData_write(Ctr_ID_9);
    clock_t ndw2q1 = clock();
    while( clock() - ndw2q1 < 1000000);//1000000

    cout << " initialing !!! " << endl;
    /////////////////////////////////////////////////// 


    /////////////////
    //judge L1 initial completed
    float position1_error = 0 , ID1_pout , ID1_iout;
    int real_pos_ID1 = 0 , Real_pos_ID1 = 0 , Last_real_pos_ID1 = 0 , beyond_boder_ID1_cnt = 0;

    //judge L2 initial completed
    int real_pos_ID2 = 0 , Real_pos_ID2 = 0 , Last_real_pos_ID2 = 0 , beyond_boder_ID2_cnt = 0;

    while(1)
    {
        //////////////////////////////////////////////////////////////////////////////////////////
        //read ID1 postion
        //cout << " initialing !!! " << endl;
        Read_ID_1.a = 0x55;
        Read_ID_1.b = 0x55;
        Read_ID_1.c = 0x01;
        Read_ID_1.d = 0x03;
        Read_ID_1.e = 0x1C;
        Read_ID_1.f = 0xDF;
       // serial.SendData_read(Read_ID_1);
        //clock_t now101 = clock();
       // while( clock() - now101 < 20000);// 
       // serial.ReadData(ID_1); 
 
        Last_real_pos_ID1 = Real_pos_ID1;
        Real_pos_ID1 =  ID_1.f | (ID_1.g << 8) ;

        //////////////////////////////////////////////////////////////////////////////////////////
        if(Real_pos_ID1 >= 256)
        {
            //cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
            beyond_boder_ID1_cnt = 0;
        }
        if( ((Real_pos_ID1 >= 127-50 && Real_pos_ID1 <= 127) && Last_real_pos_ID1 >= 256)  )//|| ( (Real_pos_ID2 >= 512 && Real_pos_ID2 <= 512+50) && (Last_real_pos_ID2 >= 639-50 && Last_real_pos_ID2 <= 639) )
        {
            beyond_boder_ID1_cnt = 1;
            cout << " beyond_boder_ID1_cnt :" << beyond_boder_ID1_cnt << endl;
        }

        ///////////////////////
        if(beyond_boder_ID1_cnt == 0)
        {
            real_pos_ID1 = Real_pos_ID1;
        }
        if(beyond_boder_ID1_cnt == 1)
        {
            real_pos_ID1 = 256 - 1 - (127 - Real_pos_ID1);//639 - 512 = 128
        }   
        //cout << "real_pos_ID1 :" << real_pos_ID1 << endl;

        tar_position_ID_1 = 370;
        position1_error = tar_position_ID_1 - real_pos_ID1;
        ID1_pout =  0.2 * position1_error;
        ID1_iout +=  0.015 * position1_error;
        if(ID1_iout >= 300)
        {
            ID1_iout = 300;
        }
        if(ID1_iout <= -300)
        {
            ID1_iout = -300;
        }
        //cout << " position1_error :" << position1_error << endl;

        speed_ID_speed_1 = ID1_pout + ID1_iout;
  
        high_bit_speed_1 = (speed_ID_speed_1 >> 8) & 0xff; //high 8-bit
        low_bit_speed_1  =  speed_ID_speed_1 & 0xff;        //low  8-bit
        Checksum_speed_1 = 0xFF &( ~(0x01 + 0x07 + 0x1d + 0x01 + 0x00 + low_bit_speed_1 + high_bit_speed_1 ) );
        Ctr_ID_speed_1.a = 0x55;
        Ctr_ID_speed_1.b = 0x55;
        Ctr_ID_speed_1.c = 0x01;
        Ctr_ID_speed_1.d = 0x07;
        Ctr_ID_speed_1.e = 0x1d;
        Ctr_ID_speed_1.f = 0x01;
        Ctr_ID_speed_1.g = 0x00;
        Ctr_ID_speed_1.h = low_bit_speed_1;
        Ctr_ID_speed_1.i = high_bit_speed_1;
        Ctr_ID_speed_1.j = Checksum_speed_1;

        //if(ID_6_ctrl_flag_last != ID_6_ctrl_flag)
       // {
         //   cout << " ID_6 action !" << endl;
           //serial.SendData_write(Ctr_ID_speed_1);
         //  clock_t now2133 = clock();
          // while( clock() - now2133 < 30000);
        //}













        //read ID2 postion
        Read_ID_2.a = 0x55;
        Read_ID_2.b = 0x55;
        Read_ID_2.c = 0x02;
        Read_ID_2.d = 0x03; 
        Read_ID_2.e = 0x1C;
        Read_ID_2.f = 0xDE;
        serial.SendData_read(Read_ID_2);
        clock_t now202 = clock();
        while( clock() - now202 < 20000);// 
        serial.ReadData(ID_2);       
 
        Last_real_pos_ID2 = Real_pos_ID2;
        Real_pos_ID2 =  ID_2.f | (ID_2.g << 8) ;
 
        ///////////////////////////////////////////////////////////////////////////////////////////
        if(Real_pos_ID2 >= 768)
        {
            //cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
            beyond_boder_ID2_cnt = 0;
        }
        if( ((Real_pos_ID2 >= 639-50 && Real_pos_ID2 <= 639) && Last_real_pos_ID2 >= 768) || ( (Real_pos_ID2 >= 512 && Real_pos_ID2 <= 512+50) && (Last_real_pos_ID2 >= 639-50 && Last_real_pos_ID2 <= 639) ) )
        {
            beyond_boder_ID2_cnt = 1;
            cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
        }
        if( ( (Real_pos_ID2 >= 639-50 && Real_pos_ID2 <= 639) && (Last_real_pos_ID2 >= 512 && Last_real_pos_ID2 <= 512+50) ) || ( (Real_pos_ID2 >= 512 && Real_pos_ID2 <= 512+50) && (Last_real_pos_ID2 >= 383-50 && Last_real_pos_ID2 <= 383) ) )
        {
            beyond_boder_ID2_cnt = 2;
            cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
        }
        if( ( (Real_pos_ID2 >= 383-50 && Real_pos_ID2 <= 383) && (Last_real_pos_ID2 >= 512 && Last_real_pos_ID2 <= 512+50) ) || ( (Real_pos_ID2 >= 256 && Real_pos_ID2 <= 256+50) && (Last_real_pos_ID2 >=  383-50 && Last_real_pos_ID2 <= 383) ) )
        {
            beyond_boder_ID2_cnt = 3;
            cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
        }
        if( ( (Real_pos_ID2 >= 383-50 && Real_pos_ID2 <= 383) && (Last_real_pos_ID2 >= 256 && Last_real_pos_ID2 <= 256+50) ) || ( (Real_pos_ID2 >= 256 && Real_pos_ID2 <= 256+50) && (Last_real_pos_ID2 >= 129-50 && Last_real_pos_ID2 <= 129) ))
        {
            beyond_boder_ID2_cnt = 4;
            cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
        }
        if( ( (Real_pos_ID2 >= 129-50 && Real_pos_ID2 <= 129) && (Last_real_pos_ID2 >= 256 && Last_real_pos_ID2 <= 256+50) ) )//|| ( (Real_pos_ID2 >= 258 && Real_pos_ID2 <= 258+50) && (Last_real_pos_ID2 >= 129-50 && Last_real_pos_ID2 <= 129) )
        {
            beyond_boder_ID2_cnt = 5;
            cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
        }

        ///////////////////////
        if(beyond_boder_ID2_cnt == 0)
        {
            real_pos_ID2 = Real_pos_ID2;
        }
        if(beyond_boder_ID2_cnt == 1)
        {
            real_pos_ID2 = 768 - 1 - (639 - Real_pos_ID2);//639 - 512 = 128
        }   
        if(beyond_boder_ID2_cnt == 2)
        {
            real_pos_ID2 = 640 - 1 - (639 - Real_pos_ID2);//639 - 512 = 128
        }          
        if(beyond_boder_ID2_cnt == 3)
        {
            real_pos_ID2 = 512 - 1 - (383 - Real_pos_ID2);//383 - 256 = 127
        }          
        if(beyond_boder_ID2_cnt == 4)
        {
            real_pos_ID2 = 386 - 1 - (383 - Real_pos_ID2);//383 - 256 = 127
        }              
        if(beyond_boder_ID2_cnt == 5)
        {
            real_pos_ID2 = 258 - 1 - (129 - Real_pos_ID2);//383 - 256 = 127
        }              
                  
        cout << " real_pos_ID2 : " << real_pos_ID2 << endl;
        ////////////////////////////////////////////////////////////////////////////////////////////
    

        //theta_1 + theta_2  = 90:
        ////////////////////////////////////////////////////////////////////////////////////////////     
        real_theta_1 = (vertical_posi_ID_1 - real_pos_ID1) * 1.0 / 375 * (Pi / 2);  // 
        tar_theta_2 = Pi / 2 - real_theta_1;
        tar_position_ID_2 = position_ID_2_intial;//vertical_posi_ID_2 - 25; //tar_theta_2 * angle_posi_para;
        ////////////////////////////////////////////////////////////////////////////////////////////
        


        ////////////////////////////////////////////////////////////////////////////////////////////
        ID_6_ctrl_flag_last = ID_6_ctrl_flag;       
        if( fabs(tar_position_ID_2 - real_pos_ID2) <= 5 )
            ID_6_ctrl_flag = 0;
        if( tar_position_ID_2 - real_pos_ID2 > 5 )
            ID_6_ctrl_flag = 1; 
        if( tar_position_ID_2 - real_pos_ID2 < -5 )
            ID_6_ctrl_flag = -1;    

        if( ID_6_ctrl_flag == -1 )
            speed_ID_6 = ID_6_speed_set_initial;
        if( ID_6_ctrl_flag ==  0 )
            speed_ID_6 =  0;
        if( ID_6_ctrl_flag ==  1 )
            speed_ID_6 = -ID_6_speed_set_initial;

        //speed_ID_6 = 0;
        high_bit_6 = (speed_ID_6 >> 8) & 0xff; //high 8-bit
        low_bit_6  =  speed_ID_6 & 0xff;        //low  8-bit
        Checksum_6 = 0xFF &( ~(0x06 + 0x07 + 0x1d + 0x01 + 0x00 + low_bit_6 + high_bit_6 ) );
        Ctr_ID_6.a = 0x55;
        Ctr_ID_6.b = 0x55;
        Ctr_ID_6.c = 0x06;
        Ctr_ID_6.d = 0x07;
        Ctr_ID_6.e = 0x1d;
        Ctr_ID_6.f = 0x01;
        Ctr_ID_6.g = 0x00;
        Ctr_ID_6.h = low_bit_6;
        Ctr_ID_6.i = high_bit_6;
        Ctr_ID_6.j = Checksum_6;

        if(ID_6_ctrl_flag_last != ID_6_ctrl_flag)
        {
            cout << " ID_6 action !" << endl;
            serial.SendData_write(Ctr_ID_6);
            clock_t now3 = clock();
            while( clock() - now3 < 100000);
        }
        
        if(fabs(tar_position_ID_2 - real_pos_ID2) <= 5)
           break; 
        ////////////////////////////////////////////////////////////////////////////////////////////
    }

    speed_ID_6 = 0;
    high_bit_6 = (speed_ID_6 >> 8) & 0xff; //high 8-bit
    low_bit_6  = speed_ID_6 & 0xff;        //low  8-bit
    Checksum_6 = 0xFF &( ~(0x06 + 0x07 + 0x1d + 0x01 + 0x00 + low_bit_6 + high_bit_6 ) );
    Ctr_ID_6.a = 0x55;
    Ctr_ID_6.b = 0x55;
    Ctr_ID_6.c = 0x06;
    Ctr_ID_6.d = 0x07;
    Ctr_ID_6.e = 0x1d;
    Ctr_ID_6.f = 0x01;
    Ctr_ID_6.g = 0x00;
    Ctr_ID_6.h = low_bit_6;
    Ctr_ID_6.i = high_bit_6;
    Ctr_ID_6.j = Checksum_6;
    serial.SendData_write(Ctr_ID_6);


    cout << "initial completed !!!!!!!!!!!!!!!" << endl << endl;
    cout << "initial completed !!!!!!!!!!!!!!!" << endl << endl;
    cout << "initial completed !!!!!!!!!!!!!!!" << endl << endl;
    ///////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////








    ///////////////////////////////////////////////////////////////////////////////////////
    //input x y
    float x, y;
    cout << "please input target_point(x , y)" << endl;
    cin >> x >> y;
    if(y < y_min)
    {
        //return 0;
    }

    //float X = x , Y = y;
    //cout << X << " " << Y << endl;
    ///////////////////////////////////////////////////////////////////////////////////////




    ////////////////////////////////////////////////////////////////
    //judge validation of x y:
    float i ,j , k , h , g , a1 , b1 , a2 , b2 , a , b , delta;

    i = 2 * x - 2 * L2;
    //cout << " i :" << i << endl;

    j = (pow(x,2) + pow(y,2)) + pow(L1,2) - pow(L2,2) - pow(L3,2);
    //cout << " j :" << j << endl;

    k = 1 + pow(i,2) / (4 * pow(y,2));
    //cout << " k :" << k << endl;

    h = 2 * L2 + (2 * i * j) / (4 * pow(y,2));
    //cout << " h :" << h << endl;

    g = pow(L2,2) + pow(j,2) / (4 * pow(y,2)) - pow(L1,2);
    //cout << " g :" << g << endl;

    delta = pow(h,2) - 4 * k * g;

    //no solver
    if( delta < 0 )
    {
        cout << "There is no solver" << endl;
        //return 0;
    }
    ////////////////////////////////////////////////////////////////





    //define L: J3 - input
    double L ;

    ///////////////////////////////////////////////////////////////////////////////////////
    //algorithm flag:
    int solver_start = 1;
 
    //mode control:
    int stretchout_flag = 1;
    int grab_flag = 0;
    int drop_flag = 0;
    int pullback_flag = 0;
    
    int control_mode = 0;

    //move flag:
    int ID_1_move_flag = 0;
    int ID_3_move_flag = 0;


    while(ros::ok())
    {
        //////////////////////////////////////////////////////////////////////////////////////////
        //read ID1 postion
        Read_ID_1.a = 0x55;
        Read_ID_1.b = 0x55;
        Read_ID_1.c = 0x01;
        Read_ID_1.d = 0x03;
        Read_ID_1.e = 0x1C;
        Read_ID_1.f = 0xDF;
        serial.SendData_read(Read_ID_1);
        clock_t now101 = clock();
        while( clock() - now101 < 20000);// 
        serial.ReadData(ID_1); 
 
        Last_real_pos_ID1 = Real_pos_ID1;
        Real_pos_ID1 =  ID_1.f | (ID_1.g << 8) ;

        //////////////////////////////////////////////////////////////////////////////////////////
        if(Real_pos_ID1 >= 256)
        {
            //cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
            beyond_boder_ID1_cnt = 0;
        }
        if( ((Real_pos_ID1 >= 127-50 && Real_pos_ID1 <= 127) && Last_real_pos_ID1 >= 256)  )//|| ( (Real_pos_ID2 >= 512 && Real_pos_ID2 <= 512+50) && (Last_real_pos_ID2 >= 639-50 && Last_real_pos_ID2 <= 639) )
        {
            beyond_boder_ID1_cnt = 1;
            cout << " beyond_boder_ID1_cnt :" << beyond_boder_ID1_cnt << endl;
        }

        ///////////////////////
        if(beyond_boder_ID1_cnt == 0)
        {
            real_pos_ID1 = Real_pos_ID1;
        }
        if(beyond_boder_ID1_cnt == 1)
        {
            real_pos_ID1 = 256 - 1 - (127 - Real_pos_ID1);//639 - 512 = 128
        }   
        //cout << "real_pos_ID1 :" << real_pos_ID1 << endl;

        //tar_position_ID_1 = 300;
        //position1_error = tar_position_ID_1 - real_pos_ID1;
        //ID1_pout =  0.2 * position1_error;
        //ID1_iout +=  0.015 * position1_error;
        //if(ID1_iout >= 300)
        //{
        //    ID1_iout = 300;
        //}
        //if(ID1_iout <= -300)
        //{
        //    ID1_iout = -300;
        //}
        //cout << " position1_error :" << position1_error << endl;

        //speed_ID_speed_1 = ID1_pout + ID1_iout;
  
       // //high_bit_speed_1 = (speed_ID_speed_1 >> 8) & 0xff; //high 8-bit
        //low_bit_speed_1  =  speed_ID_speed_1 & 0xff;        //low  8-bit
        //Checksum_speed_1 = 0xFF &( ~(0x01 + 0x07 + 0x1d + 0x01 + 0x00 + low_bit_speed_1 + high_bit_speed_1 ) );
        //Ctr_ID_speed_1.a = 0x55;
        //Ctr_ID_speed_1.b = 0x55;
        //Ctr_ID_speed_1.c = 0x01;
        //Ctr_ID_speed_1.d = 0x07;
        //Ctr_ID_speed_1.e = 0x1d;
        //Ctr_ID_speed_1.f = 0x01;
        //Ctr_ID_speed_1.g = 0x00;
        //Ctr_ID_speed_1.h = low_bit_speed_1;
        //Ctr_ID_speed_1.i = high_bit_speed_1;
        //Ctr_ID_speed_1.j = Checksum_speed_1;

        //if(ID_6_ctrl_flag_last != ID_6_ctrl_flag)
       // {
         //   cout << " ID_6 action !" << endl;
         //  serial.SendData_write(Ctr_ID_speed_1);
           //clock_t now2133 = clock();
          // while( clock() - now2133 < 30000);
        //}

        //////////////////////////////////////////////////////////////////////////////////////////


        //read ID2 postion
        Read_ID_2.a = 0x55;
        Read_ID_2.b = 0x55;
        Read_ID_2.c = 0x02;
        Read_ID_2.d = 0x03;
        Read_ID_2.e = 0x1C;
        Read_ID_2.f = 0xDE;
        serial.SendData_read(Read_ID_2);
        clock_t now2 = clock();
        while( clock() - now2 < 20000);
        serial.ReadData(ID_2);      
        Last_real_pos_ID2 = Real_pos_ID2;
        Real_pos_ID2 =  ID_2.f | (ID_2.g << 8) ;
 
        ////////////////////////
        if(Real_pos_ID2 >= 768)
        {
            //cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
            beyond_boder_ID2_cnt = 0;
        }
        if( ((Real_pos_ID2 >= 639-50 && Real_pos_ID2 <= 639) && Last_real_pos_ID2 >= 768) || ( (Real_pos_ID2 >= 512 && Real_pos_ID2 <= 512+50) && (Last_real_pos_ID2 >= 639-50 && Last_real_pos_ID2 <= 639) ) )
        {
            beyond_boder_ID2_cnt = 1;
            cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
        }
        if( ( (Real_pos_ID2 >= 639-50 && Real_pos_ID2 <= 639) && (Last_real_pos_ID2 >= 512 && Last_real_pos_ID2 <= 512+50) ) || ( (Real_pos_ID2 >= 512 && Real_pos_ID2 <= 512+50) && (Last_real_pos_ID2 >= 383-50 && Last_real_pos_ID2 <= 383) ) )
        {
            beyond_boder_ID2_cnt = 2;
            cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
        }
        if( ( (Real_pos_ID2 >= 383-50 && Real_pos_ID2 <= 383) && (Last_real_pos_ID2 >= 512 && Last_real_pos_ID2 <= 512+50) ) || ( (Real_pos_ID2 >= 256 && Real_pos_ID2 <= 256+50) && (Last_real_pos_ID2 >=  383-50 && Last_real_pos_ID2 <= 383) ) )
        {
            beyond_boder_ID2_cnt = 3;
            cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
        }
        if( ( (Real_pos_ID2 >= 383-50 && Real_pos_ID2 <= 383) && (Last_real_pos_ID2 >= 256 && Last_real_pos_ID2 <= 256+50) ) || ( (Real_pos_ID2 >= 256 && Real_pos_ID2 <= 256+50) && (Last_real_pos_ID2 >= 129-50 && Last_real_pos_ID2 <= 129) ))
        {
            beyond_boder_ID2_cnt = 4;
            cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
        }
        if( ( (Real_pos_ID2 >= 129-50 && Real_pos_ID2 <= 129) && (Last_real_pos_ID2 >= 256 && Last_real_pos_ID2 <= 256+50) ) )//|| ( (Real_pos_ID2 >= 258 && Real_pos_ID2 <= 258+50) && (Last_real_pos_ID2 >= 129-50 && Last_real_pos_ID2 <= 129) )
        {
            beyond_boder_ID2_cnt = 5;
            cout << " beyond_boder_ID2_cnt :" << beyond_boder_ID2_cnt << endl;
        }

        ///////////////////////
        if(beyond_boder_ID2_cnt == 0)
        {
            real_pos_ID2 = Real_pos_ID2;
        }
        if(beyond_boder_ID2_cnt == 1)
        {
            real_pos_ID2 = 768 - 1 - (639 - Real_pos_ID2);//639 - 512 = 128
        }   
        if(beyond_boder_ID2_cnt == 2)
        {
            real_pos_ID2 = 640 - 1 - (639 - Real_pos_ID2);//639 - 512 = 128
        }          
        if(beyond_boder_ID2_cnt == 3)
        {
            real_pos_ID2 = 512 - 1 - (383 - Real_pos_ID2);//383 - 256 = 127
        }          
        if(beyond_boder_ID2_cnt == 4)
        {
            real_pos_ID2 = 386 - 1 - (383 - Real_pos_ID2);//383 - 256 = 127
        }              
        if(beyond_boder_ID2_cnt == 5)
        {
            real_pos_ID2 = 258 - 1 - (129 - Real_pos_ID2);//383 - 256 = 127
        }              
        cout << " real_pos_ID2 : " << real_pos_ID2 << endl;
        ////////////////////////////////////////////////////////////////////////////////////////////
        






        /////////////////////////////////////////////////
        // theta_1 + theta_2 = 90
        //real_theta_1 = (vertical_posi_ID_1 - real_pos_ID1) * 1.0 / 375 * (Pi / 2);  // 
        //tar_theta_2 =   Pi / 2 - real_theta_1;
        //tar_position_ID_2 = vertical_posi_ID_2 - tar_theta_2 * angle_posi_para;
        // cout << " tar_position_ID_2 :" << tar_position_ID_2 << endl;
        //cout << " theta_1 : " << theta_1  * 360 / (Pi * 2) << endl;

        //position_ID_8 = position_ID_8_intial - (real_pos_ID1 - position_ID_1_intial) * 1000.0 / 350.0;

        //J3 coordinate:
        //J3.x = -L1 * sin(real_theta_1);
        //J3.y =  L1 * cos(real_theta_1);
        //cout << " J3.x : " << J3.x << " " << " J3.y : " << J3.y << endl;
 
        //L = sqrt( pow(x - J3.x , 2) + pow(y - J3.y , 2) );
        //cout << "L : " << L << endl; 

        //////////////////////////////////////////
        //if(x == 0 && y == 0)//initial arm postion
        //{
           // tar_theta_2 = Pi / 2 - theta_1;
           // tar_position_ID_2 = vertical_posi_ID_2 - tar_theta_2 * angle_posi_para;
           // cout << " tar_position_ID_2 :" << tar_position_ID_2 << endl;
        //}
        ///////////////////////////////////////////////////////////////







        ///////////////////////////////////////////////////////////////
        //angle solver algorithm:
        //if(solver_start == 1)
        //{
        //    float i ,j , k , h , g , a1 , b1 , a2 , b2 , a , b , delta;

        //    i = 2 * x - 2 * L2;
            //cout << " i :" << i << endl0000;

         //   j = (pow(x,2) + pow(y,2)) + pow(L1,2) - pow(L2,2) - pow(L3,2);
            //cout << " j :" << j << endl;

         //   k = 1 + pow(i,2) / (4 * pow(y,2));
            //cout << " k :" << k << endl;

        //    h = 2 * L2 + (2 * i * j) / (4 * pow(y,2));
         //   //cout << " h :" << h << endl;

         //   g = pow(L2,2) + pow(j,2) / (4 * pow(y,2)) - pow(L1,2);
            //cout << " g :" << g << endl;

        //    delta = pow(h,2) - 4 * k * g;

            //no solver
         //   if( delta < 0)
          //  {
         //       cout << "There is no solver ： delta < 0" << endl;
                //return 0;
         //   }
            //two solver
         //   if( delta > 0)
         //   {
         //      a1 = ( h - sqrt(delta) ) / (2 * k);
         //       b1 = (-i / (2 * y)) * a1 + j / (2 * y);
                //cout << "a1 : " << a1 << " b1 : " << b1 << endl;

         //       a2 = ( h + sqrt(delta) ) / (2 * k);
          //      b2 = (-i / (2 * y)) * a2 + j / (2 * y); 
                //cout << "a2 : " << a2 << " b2 : " << b2 << endl;

                //judge x range to choose better point
       //         if( (a1 >= 30.1643 && a1 <= 60))
         //       {
          //          a = a1;
          //          b = b1;
           //         cout << "smaller one   a : " << a << " b : " << b << endl;  
           //     }  
           //     if( (a1 < 30.1643 || a1 > 60) && (a2 >= 30.1643 && a2 <= 60) )
          //      {
           //         a = a2;
           //         b = b2;
          //          cout << "bigger one    a : " << a << " b : " << b << endl;  
           //     }         
           //     if( (a1 < 30.1643 || a1 > 60) && (a2 < 30.1643 || a2 > 60) )
          //      {
            //        cout << "There is no solver ： delta < 0" << endl;
                    //return 0;
           //     }    
                //
           // }
            //only one solver
         //   if( delta == 0)
         //   {
         //       a  = h / (2 * k);
          //      b  = (-i / 2 * y) * a   + j / (2 * y); 
          //      cout <<  "only one solver   a : " << a << " b : " << b << endl;
          //  }

            //final solving result: position_ID_1 and position_ID_3
       //     tar_theta_1   = atan2(L2 - a , b);
        //    position_ID_1 = vertical_posi_ID_1 - tar_theta_1 * angle_posi_para;
         //   cout << "position_ID_1 :" << position_ID_1 << endl;
        //    ID_1_ctrl_flag = 1;

         //   if(y != b)
         //   {
        //        tar_theta_3       = atan2( x - a , y - b);
          //      // cout << "tar_theta_3 :" << tar_theta_3 << endl;
         //       if(x < a)
          //      {
          //          position_ID_3 = vertical_posi_ID_3 - tar_theta_3 * angle_posi_para;
          //      }
           //     else
           //     {
          //          position_ID_3 = vertical_posi_ID_3 - tar_theta_3 * angle_posi_para;
         //      }
         //   }
         //   else
          //  {

         //       position_ID_3     = vertical_posi_ID_3 - (Pi / 2)    * angle_posi_para;
        //    }
        //    if(position_ID_3 >= 1000)
         //   {
        //        position_ID_3 = 1000;
         //   }
        //    if(position_ID_3 <= 0)
        //    {
        //        position_ID_3 = 0;
        //    }
        //    cout << "position_ID_3 :" << position_ID_3 << endl;
         //   ID_3_ctrl_flag = 1;

        //    solver_start = 0;
       // }


        /////////////////////////////////////////////////////////////////////////



        //control mode:
        if(stretchout_flag == 1)
        {
            stretchout_flag = 0;


            ID_1_ctrl_flag = 1;
            time_1 = 5000;
            tar_position_ID_1 = position_ID_1_intial  ;//tar_position_ID_1 - 0.5;


            tar_position_ID_2 = 790;
            ID_6_speed_set    = 800;

            ID_3_ctrl_flag = 1;
            position_ID_3 = 900;//position_ID_3 + 15;

            ID_4_ctrl_flag = 1;
            position_ID_4  = 720;

            ID_8_ctrl_flag = 1;
            position_ID_8 = position_ID_8_intial - 500;
            time_8 = ID_8_speed_set - 6000;
        }

        if(grab_flag == 1)
        {
 
        }

        if(drop_flag == 1)
        {
 
        }

        if(pullback_flag == 1)
        {
 
        }
     
        //////////////////////////////////////////////////////////////////////////////////////////////////
        
        if(control_mode == 100) //stretchout
        {
            tar_position_ID_2 = 780;
            ID_6_speed_set    = 300;

            //if(fabs(tar_position_ID_2 - real_pos_ID2) <= 50)
            //{
            //    ID_3_move_flag = 1;
            //}

            ID_3_ctrl_flag = 1;
            position_ID_3 = 800;//position_ID_3 + 15;

            if(ID_3_move_flag == 1)
            {
                ID_3_ctrl_flag = 1;
                position_ID_3 = 800;//position_ID_3 + 15;
                if(position_ID_3 >= 800)
                {
                    position_ID_3 = 800;
                
                    //ID_4_ctrl_flag = 1;
                    //position_ID_4  = 0;
                }
                
                //ID_8_ctrl_flag = 0;
                //position_ID_8 = position_ID_8_intial - 500;
                //time_8 = ID_8_speed_set - 2000;
            }
            
            if(fabs(tar_position_ID_2 - real_pos_ID2) <= 5)
            {
               // ID_1_move_flag = 1;
            }

            if(ID_1_move_flag == 1)
            {
                speed_ID_6 = 1000;

                ID_1_ctrl_flag = 1;
                time_1 = 1;
                tar_position_ID_1 = position_ID_1_intial;//tar_position_ID_1 - 0.5;
                if(tar_position_ID_1 <= 350)
                {
                   // control_mode = 0;
                    //tar_position_ID_1 = 350;
                }
            }
        }

        if(control_mode == 5)//grab
        {
            ID_5_ctrl_flag = 1;
            speed_ID_5 = 600;
        }
        if(control_mode == 6)//drop
        {
            ID_5_ctrl_flag = 1;
            speed_ID_5 = -100;
        }


        if(control_mode == 3) //pullback
        {
            ID_1_ctrl_flag = 1;
            time_1 = 1; 
            tar_position_ID_1 = tar_position_ID_1 + 5;
            if(tar_position_ID_1 >= position_ID_1_intial)
            {
                tar_position_ID_1 = position_ID_1_intial;
            }

            tar_position_ID_2 = position_ID_2_intial;

            ID_3_ctrl_flag = 1;
            position_ID_3  = position_ID_3_intial;

            ID_4_ctrl_flag = 1;
            position_ID_4  = position_ID_4_intial;

            if(fabs(tar_position_ID_2 - real_pos_ID2) <= 5)
            {
               control_mode = 10;
            }

            ID_8_ctrl_flag = 1;
            position_ID_8 = position_ID_8_intial;
            time_8 = ID_8_speed_set - 2000;
        }

  
        if(control_mode == 1) //emergency
        {
            ID_1_ctrl_flag = 1;
            time_1 = 1;

            ID_3_ctrl_flag = 1;

            tar_position_ID_1 = position_ID_1_intial;
            tar_position_ID_2 = position_ID_2_intial - 100;
            position_ID_3 = position_ID_3_intial;

            if(fabs(tar_position_ID_2 - real_pos_ID2) <= 5)
            {
                control_mode = 10;
            }
        }


        if(control_mode == 2)//
        {
            tar_position_ID_2 = 610;

            if(fabs(tar_position_ID_2 - real_pos_ID2) <= 5)
            {
                ID_1_ctrl_flag = 1;
                time_1 = 1000; 
                tar_position_ID_1 = tar_position_ID_1 - 0.5;
                if(tar_position_ID_1 <= 200)
                {
                    control_mode = 3;
                    tar_position_ID_1 = 200;
                }

                ID_3_ctrl_flag = 1;
                position_ID_3  = 300;

            }
        }




        cout << " control_mode :" << control_mode << endl;
        //cout << "tar_position_ID_1 :" << tar_position_ID_1 << endl;
        ////////////////////////////////////////////////////////////////////////////////////////


        //control:
        /////////////////////////////////////////////////////////
        // 6 control:
        ID_6_ctrl_flag_last = ID_6_ctrl_flag;       
        if( tar_position_ID_2 - real_pos_ID2 <= 5 && tar_position_ID_2 - real_pos_ID2 >= -5 )
            ID_6_ctrl_flag = 0;
        if( tar_position_ID_2 - real_pos_ID2 > 5 )
            ID_6_ctrl_flag = 1;//500
        if( tar_position_ID_2 - real_pos_ID2 < -5 )
            ID_6_ctrl_flag = -1;    
   
        if( ID_6_ctrl_flag == -1 )
            speed_ID_6 =  ID_6_speed_set;
        if( ID_6_ctrl_flag ==  0 )
            speed_ID_6 =  0;
        if( ID_6_ctrl_flag ==  1 )
            speed_ID_6 = -ID_6_speed_set;

        //speed_ID_6 = 0;
        high_bit_6 = (speed_ID_6 >> 8) & 0xff; //high 8-bit
        low_bit_6  = speed_ID_6 & 0xff;        //low  8-bit
        Checksum_6 = 0xFF &( ~(0x06 + 0x07 + 0x1d + 0x01 + 0x00 + low_bit_6 + high_bit_6 ) );
        Ctr_ID_6.a = 0x55;
        Ctr_ID_6.b = 0x55;
        Ctr_ID_6.c = 0x06;
        Ctr_ID_6.d = 0x07;
        Ctr_ID_6.e = 0x1d;
        Ctr_ID_6.f = 0x01;
        Ctr_ID_6.g = 0x00;
        Ctr_ID_6.h = low_bit_6;
        Ctr_ID_6.i = high_bit_6;
        Ctr_ID_6.j = Checksum_6;

        if(ID_6_ctrl_flag_last != ID_6_ctrl_flag)
        {
            cout << " ID_6 action !" << endl;
            serial.SendData_write(Ctr_ID_6);
            clock_t now3 = clock();
            while( clock() - now3 < 100000);// 
        }
    

        //5   control :
        high_bit_5 = (speed_ID_5 >> 8) & 0xff; //high 8-bit
        low_bit_5  = speed_ID_5 & 0xff;        //low  8-bit
        Checksum_5 = 0xFF &( ~(0x05 + 0x07 + 0x1d + 0x01 + 0x00 + low_bit_5 + high_bit_5 ) );
        Ctr_ID_5.a = 0x55;
        Ctr_ID_5.b = 0x55;
        Ctr_ID_5.c = 0x05;
        Ctr_ID_5.d = 0x07;
        Ctr_ID_5.e = 0x1d;
        Ctr_ID_5.f = 0x01;
        Ctr_ID_5.g = 0x00;
        Ctr_ID_5.h = low_bit_5;
        Ctr_ID_5.i = high_bit_5;
        Ctr_ID_5.j = Checksum_5;

        if(ID_5_ctrl_flag == 1)
        {
            ID_5_ctrl_flag = 0;
            serial.SendData_write(Ctr_ID_5);
            clock_t nrhrhrfe1 = clock();
            while( clock() - nrhrhrfe1 < 500);//1000000
        }


        // 1 control:
        //position_ID_1 = 80;
        position_ID_1 = tar_position_ID_1 + odom_pitch / (Pi / 2) * 375;
        if(position_ID_1 <= 80)
        {
            position_ID_1 = 80;
        }
        if(position_ID_1 >= position_ID_9_intial * 2)
        {
            //position_ID_1 = position_ID_9_intial * 2;
        }
        //cout << "position_ID_1 :" << position_ID_1 << endl;

        //time_1 = 1;
        high_bit_1 = (position_ID_1 >> 8) & 0xff; //high 8-bit
        low_bit_1  = position_ID_1 & 0xff;        //low  8-bit
        High_bit_1 = (time_1 >> 8) & 0xff; //high 8-bit
        Low_bit_1  = time_1 & 0xff;        //low  8-bit
        Checksum_1 = 0xFF &( ~(0x01 + 0x07 + 0x01 + low_bit_1 + high_bit_1 + Low_bit_1 + High_bit_1) );
        Ctr_ID_1.a = 0x55;
        Ctr_ID_1.b = 0x55;
        Ctr_ID_1.c = 0x01;
        Ctr_ID_1.d = 0x07;
        Ctr_ID_1.e = 0x01;
        Ctr_ID_1.f = low_bit_1;
        Ctr_ID_1.g = high_bit_1;
        Ctr_ID_1.h = Low_bit_1;
        Ctr_ID_1.i = High_bit_1;
        Ctr_ID_1.j = Checksum_1;

        if(ID_1_ctrl_flag == 1)
        {
            ID_1_ctrl_flag = 0;
            cout << " ID_1 action !" << endl;           
            serial.SendData_write(Ctr_ID_1);
            clock_t now4 = clock();
            while( clock() - now4 < 500);//1000000
        }


        // 3 control:
        high_bit_3 = (position_ID_3 >> 8) & 0xff; //high 8-bit
        low_bit_3  = position_ID_3 & 0xff;        //low  8-bit
        High_bit_3 = (3000 >> 8) & 0xff; //high 8-bit
        Low_bit_3  = 3000 & 0xff;        //low  8-bit
        Checksum_3 = 0xFF &( ~(0x03 + 0x07 + 0x01 + low_bit_3 + high_bit_3 + Low_bit_3 + High_bit_3) );
        Ctr_ID_3.a = 0x55;
        Ctr_ID_3.b = 0x55;
        Ctr_ID_3.c = 0x03;
        Ctr_ID_3.d = 0x07;
        Ctr_ID_3.e = 0x01;
        Ctr_ID_3.f = low_bit_3;
        Ctr_ID_3.g = high_bit_3;
        Ctr_ID_3.h = Low_bit_3;
        Ctr_ID_3.i = High_bit_3;
        Ctr_ID_3.j = Checksum_3;

        if(ID_3_ctrl_flag == 1)
        {
            ID_3_ctrl_flag = 0;
            cout << " ID_3 action !" << endl;
            serial.SendData_write(Ctr_ID_3);
            clock_t nbfv1 = clock();
            while( clock() - nbfv1 < 5000);//1000000
        }

        // 4 control:
        high_bit_4 = (position_ID_4 >> 8) & 0xff; //high 8-bit
        low_bit_4  = position_ID_4 & 0xff;        //low  8-bit
        High_bit_4 = (time_4 >> 8) & 0xff; //high 8-bit
        Low_bit_4  = time_4 & 0xff;        //low  8-bit
        Checksum_4 = 0xFF &( ~(0x04 + 0x07 + 0x01 + low_bit_4 + high_bit_4 + Low_bit_4 + High_bit_4) );
        Ctr_ID_4.a = 0x55;
        Ctr_ID_4.b = 0x55;
        Ctr_ID_4.c = 0x04;
        Ctr_ID_4.d = 0x07;
        Ctr_ID_4.e = 0x01;
        Ctr_ID_4.f = low_bit_4;
        Ctr_ID_4.g = high_bit_4;
        Ctr_ID_4.h = Low_bit_4;
        Ctr_ID_4.i = High_bit_4;
        Ctr_ID_4.j = Checksum_4;

        if(ID_4_ctrl_flag == 1)
        {
            ID_4_ctrl_flag = 0;
            cout << " ID_4 action !" << endl;
            serial.SendData_write(Ctr_ID_4);
            clock_t nbf22v1 = clock();
            while( clock() - nbf22v1 < 5000);//1000000
        }

        //8 control:
        high_bit_8 = (position_ID_8 >> 8) & 0xff; //high 8-bit
        low_bit_8  = position_ID_8 & 0xff;        //low  8-bit

        High_bit_8 = (time_8 >> 8) & 0xff; //high 8-bit
        Low_bit_8  = time_8 & 0xff;        //low  8-bit

        Checksum_8 = 0xFF &( ~(0x08 + 0x07 + 0x01 + low_bit_8 + high_bit_8 + Low_bit_8 + High_bit_8) );

        Ctr_ID_8.a = 0x55;
        Ctr_ID_8.b = 0x55;
        Ctr_ID_8.c = 0x08;
        Ctr_ID_8.d = 0x07;
        Ctr_ID_8.e = 0x01;
        Ctr_ID_8.f = low_bit_8;
        Ctr_ID_8.g = high_bit_8;
        Ctr_ID_8.h = Low_bit_8;
        Ctr_ID_8.i = High_bit_8;
        Ctr_ID_8.j = Checksum_8;

        if(ID_8_ctrl_flag == 1)
        {
            ID_8_ctrl_flag = 0;

            serial.SendData_write(Ctr_ID_8);
            clock_t nowqw4 = clock();
            while( clock() - nowqw4 < 10000);//1000000
        }

        //9 control:
        position_ID_9 = position_ID_9_intial - odom_roll / (Pi / 2) * 375;
        if(position_ID_9 <= 0)
        {
            position_ID_9 = 0;
        }
        if(position_ID_9 >= position_ID_9_intial * 2)
        {
            position_ID_9 = position_ID_9_intial * 2;
        }

        //cout << "position_ID_9 :" << position_ID_9 << endl;
        
        high_bit_9 = (position_ID_9 >> 8) & 0xff; //high 8-bit
        low_bit_9  = position_ID_9 & 0xff;        //low  8-bit
        High_bit_9 = (time_9 >> 8) & 0xff; //high 8-bit
        Low_bit_9  = time_9 & 0xff;        //low  8-bit
        Checksum_9 = 0xFF &( ~(0x09 + 0x07 + 0x01 + low_bit_9 + high_bit_9 + Low_bit_9 + High_bit_9) );
        Ctr_ID_9.a = 0x55;
        Ctr_ID_9.b = 0x55;
        Ctr_ID_9.c = 0x09;
        Ctr_ID_9.d = 0x07;
        Ctr_ID_9.e = 0x01;
        Ctr_ID_9.f = low_bit_9;
        Ctr_ID_9.g = high_bit_9;
        Ctr_ID_9.h = Low_bit_9;
        Ctr_ID_9.i = High_bit_9;
        Ctr_ID_9.j = Checksum_9;
        //serial.SendData_write(Ctr_ID_9);
        clock_t ndw2q1 = clock();
        while( clock() - ndw2q1 < 500);//1000000



        ///////////////////////////////////////////////////////////////         
        

        

        ///////////////////////////////////////////////////////////
        ros::spinOnce();

    } 


    //shut down all motor
    speed_ID_6 = 0;
    high_bit_6 = (speed_ID_6 >> 8) & 0xff; //high 8-bit
    low_bit_6  = speed_ID_6 & 0xff;        //low  8-bit
    Checksum_6 = 0xFF &( ~(0x06 + 0x07 + 0x1d + 0x01 + 0x00 + low_bit_6 + high_bit_6 ) );
    Ctr_ID_6.a = 0x55;
    Ctr_ID_6.b = 0x55;
    Ctr_ID_6.c = 0x06;
    Ctr_ID_6.d = 0x07;
    Ctr_ID_6.e = 0x1d;
    Ctr_ID_6.f = 0x01;
    Ctr_ID_6.g = 0x00;
    Ctr_ID_6.h = low_bit_6;
    Ctr_ID_6.i = high_bit_6;
    Ctr_ID_6.j = Checksum_6;
    serial.SendData_write(Ctr_ID_6);


    return 0;

}


