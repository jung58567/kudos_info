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

class e2box_imu_9dofv4 m_e2box_imu;
class IMU imu;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// void OnReceiveImu(void)
// {
//     int n = m_e2box_imu.serial.GetLength();
//     unsigned char *pBuffer = m_e2box_imu.serial.GetBuffer();

//     if(n>=10){
//         for(int i=0; i<n; ++i){
//             m_e2box_imu.ExtractData(pBuffer[i]);
//             if(m_e2box_imu.data_acquisition){
//                 m_e2box_imu.serial.Reset();
//                 m_e2box_imu.HandlingDataIMU();
//                 m_e2box_imu.data_acquisition = false;
//                 break;
//             }
//         }
//     }
// }
// double gap_ang_vel_x = 0.0;
// double gap_ang_vel_y = 0.0;
// double gap_ang_vel_z = 0.0;
// double gap_acc_x = 0.0;
// double gap_acc_y = 0.0;
// double gap_acc_z = 0.0;


// void publishImuData(void)
// {

//     if(!m_e2box_imu.data_acquisition){

//         imu_data.header.seq = m_e2box_imu.m_dwordCounterChecksumPass;
//         imu_data.header.stamp = ros::Time::now();
//         imu_data.header.frame_id = "imu_link";

//         // e2box_imu_9dofV4 quaternion order : z, y, x, w
//         imu_data.orientation.x = m_e2box_imu.m_dQuaternion[2];
//         imu_data.orientation.y = m_e2box_imu.m_dQuaternion[1];
//         imu_data.orientation.z = m_e2box_imu.m_dQuaternion[0];
//         imu_data.orientation.w = m_e2box_imu.m_dQuaternion[3];

//         // imu_data_temp
//         imu_data_temp.angular_velocity.x = m_e2box_imu.m_dAngRate[0]*M_PI/180.0;
//         imu_data_temp.angular_velocity.y = m_e2box_imu.m_dAngRate[1]*M_PI/180.0;
//         imu_data_temp.angular_velocity.z = m_e2box_imu.m_dAngRate[2]*M_PI/180.0;

//         imu_data_temp.linear_acceleration.x = m_e2box_imu.m_dAccel[0]*9.80665;
//         imu_data_temp.linear_acceleration.y = m_e2box_imu.m_dAccel[1]*9.80665;
//         imu_data_temp.linear_acceleration.z = m_e2box_imu.m_dAccel[2]*9.80665;

//         // imu_data_prev
//         imu_data_prev.angular_velocity.x = imu_data_temp.angular_velocity.x;
//         imu_data_prev.angular_velocity.y = imu_data_temp.angular_velocity.y;
//         imu_data_prev.angular_velocity.z = imu_data_temp.angular_velocity.z;

//         imu_data_prev.linear_acceleration.x = imu_data_temp.linear_acceleration.x;
//         imu_data_prev.linear_acceleration.y = imu_data_temp.linear_acceleration.y;
//         imu_data_prev.linear_acceleration.z = imu_data_temp.linear_acceleration.z;

//         // imu_data
//         gap_ang_vel_x = fabs(imu_data_prev.angular_velocity.x - imu_data_temp.angular_velocity.x);
//         if(gap_ang_vel_x > 0.3){
//             imu_data_temp.angular_velocity.x = imu_data_prev.angular_velocity.x;
//         }
//         gap_ang_vel_y = fabs(imu_data_prev.angular_velocity.y - imu_data_temp.angular_velocity.y);
//         if(gap_ang_vel_y > 0.3){
//             imu_data_temp.angular_velocity.y = imu_data_prev.angular_velocity.y;
//         }
//         gap_ang_vel_z = fabs(imu_data_prev.angular_velocity.z - imu_data_temp.angular_velocity.z);
//         if(gap_ang_vel_z > 0.3){
//             imu_data_temp.angular_velocity.z = imu_data_prev.angular_velocity.z;
//         }

//         imu_data.angular_velocity.x = (imu_data_temp.angular_velocity.x + imu_data_prev.angular_velocity.x) / 2.0;
//         imu_data.angular_velocity.y = (imu_data_temp.angular_velocity.y + imu_data_prev.angular_velocity.y) / 2.0;
//         imu_data.angular_velocity.z = (imu_data_temp.angular_velocity.z + imu_data_prev.angular_velocity.z) / 2.0;

//         gap_acc_x = fabs(imu_data_prev.linear_acceleration.x - imu_data_temp.linear_acceleration.x);
//         if(gap_acc_x > 2.0){
//             imu_data_temp.linear_acceleration.x = imu_data_prev.linear_acceleration.x;
//         }
//         gap_acc_y = fabs(imu_data_prev.linear_acceleration.y - imu_data_temp.linear_acceleration.y);
//         if(gap_acc_y > 2.0){
//             imu_data_temp.linear_acceleration.y = imu_data_prev.linear_acceleration.y;
//         }
//         gap_acc_z = fabs(imu_data_prev.linear_acceleration.z - imu_data_temp.linear_acceleration.z);
//         if(gap_acc_z > 2.0){
//             imu_data_temp.linear_acceleration.z = imu_data_prev.linear_acceleration.z;
//         }

//         imu_data.linear_acceleration.x = (imu_data_temp.linear_acceleration.x + imu_data_prev.linear_acceleration.x) / 2.0;
//         imu_data.linear_acceleration.y = (imu_data_temp.linear_acceleration.y + imu_data_prev.linear_acceleration.y) / 2.0;
//         imu_data.linear_acceleration.z = (imu_data_temp.linear_acceleration.z + imu_data_prev.linear_acceleration.z) / 2.0;

//         /////
//         /*
//         imu_data.angular_velocity.x = m_e2box_imu.m_dAngRate[0]*M_PI/180.0;
//         imu_data.angular_velocity.y = m_e2box_imu.m_dAngRate[1]*M_PI/180.0;
//         imu_data.angular_velocity.z = m_e2box_imu.m_dAngRate[2]*M_PI/180.0;

//         imu_data.linear_acceleration.x = m_e2box_imu.m_dAccel[0]*9.80665;
//         imu_data.linear_acceleration.y = m_e2box_imu.m_dAccel[1]*9.80665;
//         imu_data.linear_acceleration.z = m_e2box_imu.m_dAccel[2]*9.80665;
//         */
//         /////

//         // outFile << imu_data.linear_acceleration.x << "\t" << imu_data.linear_acceleration.y << "\t" << imu_data.linear_acceleration.z << "\t" <<
//         //           imu_data.angular_velocity.x << "\t" << imu_data.angular_velocity.y << "\t" << imu_data.angular_velocity.z << endl;

//         //std::cout<< "x =: "<<m_e2box_imu.m_dQuaternion[3]<< "y=: " << m_e2box_imu.m_dAngRate[0] << "z=: " << m_e2box_imu.m_dAngRate[1] <<"\n"<< endl;
//         std::cout<< "x =: "<<m_e2box_imu.m_dQuaternion[0]<< "y=: " << m_e2box_imu.m_dQuaternion[1] << "z=: " << m_e2box_imu.m_dQuaternion[2] <<"\n"<< endl;

//         imu_pub.publish(imu_data);
//     }
// }



// void practice_L();
// void practice_R();


// VectorXd kinematics::Geometric_IK_L(VectorXd GB_cfg, VectorXd GF_cfg)
// void practice_L()
// {
//       Eigen::Vector3d ground(3), p_7(3),p_2(3),p_1(3),r_vec(3),PH(3),pf(3),fa(3),r(3); 
//    Eigen::MatrixXd R_1(3,3),R_7(3,3),R_x(3,3),R_y(3,3),rot_mat(3,3),RH(3,3);
//    Eigen::VectorXd q(6);

//    Eigen::MatrixXd GB = MatrixXd::Identity(4,4);
//    Eigen::MatrixXd GF = MatrixXd::Identity(4,4);
// //왼쪽다리 각도 구하기
//     ground << 0,0,0;
//     p_7 << 0,0.105,0.09;//0,0.105,0.09;
//     p_2 << 0,0.2,0.6;//0,0.105,0.9412

//     r_vec = p_2-p_7;
//     double C,q_5,A,B,alpa,q_7,q_6,q_2,q_3,q_4;
//     A=0.35; B=0.35;

//     C = sqrt(r_vec.dot(r_vec));
//     q_5 = -acos((A*A+B*B-C*C)/(2*A*B))+PI;//q5 = -acos((A*A + B*B -C*C)/(2*A*B))+PI;
//     alpa = asin(A*sin(PI-q_5)/C);
//     q_7 = atan2(r_vec(1),r_vec(2));
//     q_6 = -atan2(r_vec(0),signbit(r_vec(2))*(sqrt(r_vec(1)*r_vec(1)+r_vec(2)*r_vec(2)))-alpa);
    
//     R_1 << 1,0,0,\
//           0,1,0,\
//           0,0,1;\
//     R_7 << 1,0,0,\
//           0,1,0,\
//           0,0,1;\
          
//     R_x << 1,0,0,\
//           0,cos(-q_7),-sin(-q_7),\
//           0,sin(-q_7),cos(-q_7);\
//     R_y << cos(-q_5-q_6),0,sin(-q_5-q_6),\
//           0,1,0,\
//           -sin(-q_5-q_6),0,cos(-q_5-q_6);\
    
//     rot_mat = R_1.transpose()*R_7*R_x*R_y;
//     q_2 = atan2(-rot_mat(0,1),rot_mat(1,1));
//     q_3 = atan2(rot_mat(2,1),-rot_mat(0,1)*sin(q_2)+rot_mat(1,1)*cos(q_2));
//     q_4 = atan2(-rot_mat(2,0),rot_mat(2,2));

//     std::cout<< "r_vec = " <<r_vec<<std::endl;
//     std::cout<< "C = " <<C<<std::endl;
//     std::cout<< "alpa = " <<alpa<<std::endl;
//     std::cout<< "q2 = " <<q_2<<std::endl;
//     std::cout<< "q3 = " <<q_3<<std::endl;
//     std::cout<< "q4 = " <<q_4<<std::endl;
//     std::cout<< "q5 = " <<q_5<<std::endl;
//     std::cout<< "q6 = " <<q_6<<std::endl;
//     std::cout<< "q7 = " <<q_7<<std::endl;
// }

// void practice_R()
// {
//       Eigen::Vector3d ground(3), p_7(3),p_2(3),r_vec(3); 
//    Eigen::MatrixXd R_1(3,3),R_7(3,3),R_x(3,3),R_y(3,3),rot_mat(3,3);
// // 오른다리 각도 구하기
//     ground << 0,0,0;
//     p_7 << 0,-0.105,0.09;
//     p_2 << 0,-0.105,0.9412;

//     r_vec = p_2-p_7;
//     double C,q_5,A,B,alpa,q_7,q_6,q_2,q_3,q_4;
//     A=0.35; B=0.35;

//     C = sqrt(r_vec.dot(r_vec));
//     q_5 = -acos(A*A+B*B-C*C/(2*A*B))+PI;
//     alpa = asin(A*sin(PI-q_5)/C);
//     q_7 = atan2(r_vec(1),r_vec(2));
//     q_6 = -atan2(r_vec(0),signbit(r_vec(2))*(sqrt(r_vec(1)*r_vec(1)+r_vec(2)*r_vec(2)))-alpa);
    
//     R_1 << 1,0,0,\
//           0,1,0,\
//           0,0,1;\
//     R_7 << 1,0,0,\
//           0,1,0,\
//           0,0,1;\
//     R_x << 1,0,0,\
//           0,cos(-q_7),-sin(-q_7),\
//           0,sin(-q_7),cos(-q_7);\ 
//     R_y << cos(-q_5-q_6),0,sin(-q_5-q_6),\
//           0,1,0,\
//           -sin(-q_5-q_6),0,cos(-q_5-q_6);\
    
//     rot_mat = R_1.transpose()*R_7*R_x*R_y;
//     q_2 = atan2(-rot_mat(0,1),rot_mat(1,1));
//     q_3 = atan2(rot_mat(2,1),-rot_mat(0,1)*sin(q_2)+rot_mat(1,1)*cos(q_2));
//     q_4 = atan2(-rot_mat(2,0),rot_mat(2,2));

// //     std::cout<< "r_vec = \n" <<r_vec<<std::endl;
// //     std::cout<< "C = \n" <<C<<std::endl;
// //     std::cout<< "alpa = \n" <<alpa<<std::endl;
//     std::cout<< "q2 = " <<q_2<<std::endl;
//     std::cout<< "q3 = " <<q_3<<std::endl;
//     std::cout<< "q4 = " <<q_4<<std::endl;
//     std::cout<< "q5 = " <<q_5<<std::endl;
//     std::cout<< "q6 = " <<q_6<<std::endl;
//     std::cout<< "q7 = " <<q_7<<std::endl;
// }

//reft leg
int sign(double a){
    if(a>=0) return 1;
    return -1;
}

MatrixXd rotMatZ(double d)
{
    MatrixXd tmp_m(3,3);
    tmp_m << cos(d),-sin(d),0,\
             sin(d),cos(d),0,\
             0,0,1;\
    return tmp_m;
}

MatrixXd rotMatY(double d)
{
    MatrixXd tmp_m(3,3);
    tmp_m << cos(d),0,sin(d),\
             0,1,0,\
             -sin(d),0,cos(d);\
    return tmp_m;
}

MatrixXd rotMatX(double d)
{
    MatrixXd tmp_m(3,3);
    tmp_m << 1,0,0,\
             0,cos(d),-sin(d),\
             0,sin(d),cos(d);\
    return tmp_m;
}
VectorXd Geometric_IK_L(VectorXd GB_cfg, VectorXd GF_cfg) {
    double A, B, C; // thigh, calf
    double alpha, c5;
    Eigen::VectorXd PH(3) ;
    Eigen::VectorXd p1(3), p2(3), p7(3), pf(3), fa(3), r(3);
    Eigen::VectorXd q(6);
    Eigen::MatrixXd R1(3,3), R7(3,3), RH(3,3);

    //Eigen::MatrixXd rotMatZ(3,3),rotMatY(3,3),rotMatX(3,3);
    
    Eigen::MatrixXd GB = MatrixXd::Identity(4,4);
    Eigen::MatrixXd GF = MatrixXd::Identity(4,4);

    GB.block(0,0,3,3) = rotMatZ(GB_cfg(5))*rotMatY(GB_cfg(4))*rotMatX(GB_cfg(3));
    GB.block(0,3,3,1) << GB_cfg(0), GB_cfg(1), GB_cfg(2);


    GF.block(0,0,3,3) = rotMatZ(GF_cfg(5))*rotMatY(GF_cfg(4))*rotMatX(GF_cfg(3));
    GF.block(0,3,3,1) << GF_cfg(0), GF_cfg(1), GF_cfg(2);

    PH << 0, 0.05, 0;

    R1 = GB.block(0,0,3,3);
    R7 = GF.block(0,0,3,3);
    pf = GF.block(0,3,3,1);
    fa << 0, 0, 0.037 ;
    p7 = pf+fa;
   
    A = 0.133;
    B = 0.138;
   
    p1 << GB(0,3), GB(1,3), GB(2,3);
    p2 = p1 + R1 * PH;
   
    r = R7.transpose() * (p2-p7);
    C = r.norm();
    c5 = cos((A*A + B*B - C*C) / (2 * A * B));
   
    if(c5 >= 1) q(3) = 0.0;
   
    else if(c5 <= -1) q(3) = PI;
   
    else q(3)=-acos((A*A + B*B - C*C) / (2 * A * B)) + PI;
   
    alpha = asin((A*sin(PI - q(3)))/C);
   
    q(5) = atan2(r(1), r(2));
    q(4) = -atan2(r(0), sign(r(2))*sqrt(r(1)*r(1)+r(2)*r(2))) - alpha;
   
    RH = R1.transpose() * R7 * rotMatX(-q(5)) * rotMatY(-q(3)-q(4));
   
    q(0) = atan2(-RH(0,1), RH(1,1));
    q(1) = atan2(RH(2,1), -sin(q(0)) * RH(0,1) + RH(1,1) * cos(q(0)));
    q(2) = atan2(-RH(2,0), RH(2,2));
   
    return q;
}

VectorXd Geometric_IK_R(VectorXd GB_cfg, VectorXd GF_cfg) {
    double A, B, C; // thigh, calf
    double alpha, c5;
    Eigen::VectorXd PH(3) ;
    Eigen::VectorXd p1(3), p2(3), p7(3), pf(3), fa(3), r(3);
    Eigen::VectorXd q(6);
    Eigen::MatrixXd R1(3,3), R7(3,3), RH(3,3);

    //Eigen::MatrixXd rotMatZ(3,3),rotMatY(3,3),rotMatX(3,3);
    
    Eigen::MatrixXd GB = MatrixXd::Identity(4,4);
    Eigen::MatrixXd GF = MatrixXd::Identity(4,4);

    GB.block(0,0,3,3) = rotMatZ(GB_cfg(5))*rotMatY(GB_cfg(4))*rotMatX(GB_cfg(3));
    GB.block(0,3,3,1) << GB_cfg(0), GB_cfg(1), GB_cfg(2);


    GF.block(0,0,3,3) = rotMatZ(GF_cfg(5))*rotMatY(GF_cfg(4))*rotMatX(GF_cfg(3));
    GF.block(0,3,3,1) << GF_cfg(0), GF_cfg(1), GF_cfg(2);

    PH << 0, 0.05, 0;

    R1 = GB.block(0,0,3,3);
    R7 = GF.block(0,0,3,3);
    pf = GF.block(0,3,3,1);
    fa << 0, 0, 0.037 ;
    p7 = pf+fa;
   
    A = 0.133;
    B = 0.138;
   
    p1 << GB(0,3), GB(1,3), GB(2,3);
    p2 = p1 + R1 * PH;
   
    r = R7.transpose() * (p2-p7);
    C = r.norm();
    c5 = cos((A*A + B*B - C*C) / (2 * A * B));
   
    if(c5 >= 1) q(3) = 0.0;
   
    else if(c5 <= -1) q(3) = PI;
   
    else q(3)=-acos((A*A + B*B - C*C) / (2 * A * B)) + PI;
   
    alpha = asin((A*sin(PI - q(3)))/C);
   
    q(5) = atan2(r(1), r(2));
    q(4) = -atan2(r(0), sign(r(2))*sqrt(r(1)*r(1)+r(2)*r(2))) - alpha;
   
    RH = R1.transpose() * R7 * rotMatX(-q(5)) * rotMatY(-q(3)-q(4));
   
    q(0) = atan2(-RH(0,1), RH(1,1));
    q(1) = atan2(RH(2,1), -sin(q(0)) * RH(0,1) + RH(1,1) * cos(q(0)));
    q(2) = atan2(-RH(2,0), RH(2,2));
   
    return q;
}

int main(int argc, char **argv)
{   
      //내가 추가한 부분@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    // while(1)
    // {
    //     imu.OnReceiveImu();
    //     imu.publishImuData();
    // }


      //std::cout<<"imu= "<<m_e2box_imu.m_dQuaternion[0]<<std::endl;
      
      
      //마무리@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
      
      // practice_L();
      // practice_R();
      Eigen::VectorXd Base(6);
      Eigen::VectorXd foot(6);
      Eigen::VectorXd ex_q_L(6),ex_q_R(6);
      
      Base << 0,0.1,0.2,0,0,PI/4;//둘다 기준이 바닥인거 같긴한데....(골반좌표를 수직으로 내린 지점) 그럼 이 그라운드 좌표가 계속 움직인다고 생각하면 될거 같음
      foot << 0,0.1,0,0,PI/3,0;
      ex_q_L = Geometric_IK_L(Base,foot);

      Base << 0,-0.1,0.2,0,0,PI/4;
      foot << 0,-0.1,0,0,PI/3,0;
      ex_q_R = Geometric_IK_R(Base,foot);
      
      std::cout<<"ex_q_L = "<<ex_q_L<<std::endl;
      std::cout<<"ex_q_R = "<<ex_q_R<<std::endl;

    return 0;    
}