/*
 * read_write.cpp
 *
 *  Created on: 2016. 2. 21.
 *      Author: leon
 */

//
// *********     Read and Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is designed for using a Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
// To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below "#define"d variables yourself.
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 3 (Baudrate : 1000000 [1M])
//

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "ros/ros.h"

#include "dynamixel_sdk/dynamixel_sdk.h"                                  // Uses DYNAMIXEL SDK library

#include "e2box_imu_9dofv4.h"
#include "t_serial.h"
#include "main.h"
class e2box_imu_9dofv4 m_e2box_imu;
class IMU imu;

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0            // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b
#define E2D                             360/4095

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_write");
    ros::NodeHandle n;
    ros::Publisher pub1 = n.advertise<std_msgs::Float32>("pre_pos", 1000);
    ros::Publisher pub2 = n.advertise<std_msgs::Float32>("Euler_y", 1000);
    ros::Publisher pub3 = n.advertise<std_msgs::Float32>("Ang_vel_y", 1000);
    
    //imu.publishImuData();
    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    std::string port;
    int baudrate;
    n.param<std::string>("port", port, "/dev/ttyUSB1");
    n.param("baudrate", baudrate, 115200);

     if(!m_e2box_imu.serial.Open(const_cast<char*>(port.c_str()), baudrate)){
        cout << "device is not opened! " << endl;
        return 0;
    } 
    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    double T = 2;
    double t=0, dt = 0.01;//10ms로 구동하는 듯

    double tt=0;
    int tmp3=0;
    // double T = 3;
    // double t=1, dt = 0.01;
    // double TT = 1;
    // double tt=0;
    // int tmp2=0;

    int A=0, B=2048;
    int goal_pos;
    int dxl_goal_pos_cos;

    
  // Initialize PortHandler instance
  // Set the port path11
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {1024, 2000};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }
  ros::Rate loop_rate(1000);
  //while(1)
  while(ros::ok())
  {
    std_msgs::Float32 msg1;
    std_msgs::Float32 msg2;
    std_msgs::Float32 msg3;
    imu.OnReceiveImu();
    /* 
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;*/

    /*
    printf("goal_pos=%d\n",goal_pos);
    printf("t=%f\n",t);

    goal_pos=A+ (B-A)*(t/T);

    if(t>T){
        int tmp;
        tmp = A;
        A = B;
        B = tmp;
        t=0;
        }         
    */

   //initialize begin pos
    if(t<=1){
    if(t==0){
      tmp3 = dxl_present_position;
    }//tmp3 = dxl_present_position;
    // int tmp2=0;
    // tmp2 = ;
    dxl_goal_pos_cos = (((1024-tmp3)/2)*(1-cos(3.14*t/1)))+tmp3;
    t= t+dt;
    
    
    // int tmp2=0;
    // tmp2 = dxl_present_position;
    // dxl_goal_pos_cos = (((1024-tmp2)/2)*(1-cos(3.14*t/1)))+tmp2;
    // t= t+dt;
    }
    else{
    dxl_goal_pos_cos = (((3072-1024)/2)*(1-cos(2*3.14*tt/T)))+1024;//뒤에 1024가 붙은 이유는 처음시작할 때 t=0이기 때문에 1024위치(처음위치)로 보내주기 위해서
    tt= tt+dt;
    t= t+dt;
    }
  usleep(10000);



  

  //if(T>t){
  // dxl_goal_pos_cos = (((3072-1024)/2)*(1-cos(2*3.14*t/T)))+1024;//뒤에 1024가 붙은 이유는 처음시작할 때 t=0이기 때문에 1024위치(처음위치)로 보내주기 위해서
  // t= t+dt;
  //}//if문만 주석해주면 바로 계속 돌아감 if문 안에 있으면 한번만 돔 cos안에 2pi값은 처음 위치와 나중위치 반복한다고 생각해주면 됨
  //usleep(10000); //sleep(sec) usleep(microsec) 10ms인둣 사실 슬립타임이 2초인거지 모터가 다 돌아가는게 2초는 아님(슬립타임 2초 + 모터구동시간-이건 정확히 모른다.)
  
  //if(t>T){ t=0;}
  // else
  //   break;
  

  

    // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_pos_cos , &dxl_error);//이 3번째 매개변수에다가 지정한 위치를 넣어주는 듯
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

  
      // Read present position
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }

      //printf("[ID:%03d] GoalPos:%03d  PresPos:%03d t=%f \n", DXL_ID, dxl_goal_pos_cos, dxl_present_position, t);
      std::cout<<m_e2box_imu.Euler[1]<<std::endl;
      tmp3 = dxl_present_position;

      msg1.data = dxl_present_position*E2D;
      msg2.data = -(m_e2box_imu.Euler[1])+180;
      msg3.data = m_e2box_imu.m_dAngRate[1];
      pub1.publish(msg1);
      pub2.publish(msg2);
      pub3.publish(msg3);
      ros::spinOnce();
      loop_rate.sleep();
    }

 /*
    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }*/
   


  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // Close port
  portHandler->closePort();

  return 0;
}