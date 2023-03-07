#include <stdio.h>
#include <stdlib.h> 
#include <iostream>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
using namespace std;
#define PI 3.141592

#include "e2box_imu_9dofv4.h"
#include "t_serial.h"
#include "main.h"

using namespace std;
class e2box_imu_9dofv4 m_e2box_imu;
class IMU imu;


int main(int argc, char **argv)
{   
    ros::init(argc, argv, "e2box_imu");
    ros::NodeHandle nh;

    std::string port;
    int baudrate;

    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param("baudrate", baudrate, 115200);

     if(!m_e2box_imu.serial.Open(const_cast<char*>(port.c_str()), baudrate)){
        cout << "device is not opened! " << endl;
        return 0;
    }    

    while(1)
    {
        imu.OnReceiveImu();
        //imu.publishImuData();
        std::cout<<"imu= "<<m_e2box_imu.Euler[0]<<std::endl;
    }
      
    return 0;    
}