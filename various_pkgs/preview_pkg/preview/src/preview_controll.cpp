#include <stdio.h>
#include <stdlib.h> 
#include <iostream>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
//#include <Eigen/Dense>
#include <cmath>
#include <string>
#include "std_msgs/Float64.h"

//#include "e2box_imu_9dofv4.h"

using namespace std;
#define PI 3.141592

using Eigen::MatrixXd;
using Eigen::VectorXd;
Eigen::EigenSolver<MatrixXd> eZMPSolver;

MatrixXd ZMP_DARE(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, MatrixXd R) { //Kookmin.Univ Preview
    unsigned int nSize = A.rows();
    MatrixXd Z(nSize* 2, nSize* 2);

    Z.block(0, 0, nSize, nSize) = A+ B* R.inverse()* B.transpose()* (A.inverse()).transpose() * Q;
    Z.block(0, nSize, nSize, nSize) = -B* R.inverse()* B.transpose()* (A.inverse()).transpose();
    Z.block(nSize, 0, nSize, nSize) = -(A.inverse()).transpose()* Q;
    Z.block(nSize, nSize, nSize, nSize) = (A.inverse()).transpose();

    eZMPSolver.compute(Z, true);

    Eigen::MatrixXcd U(nSize* 2, nSize);
    unsigned int j=0;
    for (unsigned int i=0; i<nSize* 2; i++)
    {
        std::complex<double> eigenvalue = eZMPSolver.eigenvalues()[i];
        double dReal = eigenvalue.real();
        double dImag = eigenvalue.imag();

        if( std::sqrt((dReal* dReal) + (dImag* dImag)) < 1.0)
        {
            U.block(0, j, nSize* 2, 1) = eZMPSolver.eigenvectors().col(i);
            j++;
        }
    }
    if(j != nSize)
    {
        printf("Warning! ******* Pelvis Planning *******\n");
    }

    Eigen::MatrixXcd U1 = U.block(0, 0, nSize, nSize);
    Eigen::MatrixXcd U2 = U.block(nSize, 0, nSize, nSize);

    Eigen::MatrixXcd X = U2 * U1.inverse();

    return X.real();
}

MatrixXd A_m(double dt)
{
    MatrixXd tmp_m(3,3);
    tmp_m << 1,dt,(dt*dt)/2,\
             0,1,dt,\
             0,0,1;\
    return tmp_m;
}
VectorXd B_m(double dt)
{
    VectorXd tmp_m(3);
    tmp_m << (dt*dt*dt)/6,(dt*dt)/2,dt;
    return tmp_m;
}
MatrixXd C_m(double z_c,double g)
{
    MatrixXd tmp_m(1,3);
    tmp_m << 1,0,-z_c/g;
    return tmp_m;
}
MatrixXd Qx_m()
{
    MatrixXd tmp_m(3,3);
    tmp_m << 0,0,0,\
             0,0,0,\
             0,0,0;\
    return tmp_m;
}
MatrixXd Q_m()
{
    MatrixXd tmp_m(4,4);
    tmp_m << 1,0,0,0,\
             0,0,0,0,\
             0,0,0,0,\
             0,0,0,0;\
    return tmp_m;
}
VectorXd BB_m(MatrixXd C,VectorXd B)
{
    VectorXd tmp_m(4);
    double a=(C*B)(0,0);
    tmp_m(0)=a;
    tmp_m(1)=B(0);
    tmp_m(2)=B(1);
    tmp_m(3)=B(2);
    return tmp_m;
}
VectorXd II_m()
{
    VectorXd tmp_m(4);
    tmp_m << 1,0,0,0;
    return tmp_m;
}
MatrixXd FF_m(Eigen::MatrixXd C,Eigen::MatrixXd A)
{
    MatrixXd tmp_P(4,3);
    tmp_P.block(0,0,1,3) = C*A;
    tmp_P.block(1,0,3,3) = A;
    return tmp_P;
}
MatrixXd AA_m(Eigen::VectorXd II,Eigen::MatrixXd FF)
{
    MatrixXd tmp_m(4,4);
    tmp_m.block(0,0,4,1) = II;
    tmp_m.block(0,1,4,3) = FF;
    return tmp_m;
}


int main(int argc, char **argv)
{   
    //std::cout<<"ffffffffffffffffffffffffffffffffffffff"<<std::endl;
    ROS_INFO("ASDFASDFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF");
    printf("hellow");
    double All_time_trajectory,dt,z_c,g,Qe;
      All_time_trajectory = 15.0;
      dt = 0.005;
      int N = 300;
      //int t = [3000];
      int n = 3000;//sizeof(t)/sizeof(*t);
      z_c = 0.55;
      g = 9.81;
      Qe = 1;
      Eigen::MatrixXd R(1,1);
      R << pow(10,-6);
    
      Eigen::MatrixXd A(3,3),C(1,3),Qx(3,3),Q(4,4);
      Eigen::MatrixXd FF(4,3),AA(4,4),KK(4,4);
      Eigen::VectorXd B(3),II(4),BB(4);
    
      A = A_m(dt);
      B = B_m(dt);
      C = C_m(z_c,g);
      Qx = Qx_m();
      Q = Q_m();
      BB = BB_m(C,B);
      II = II_m();
      FF = FF_m(C,A);
      AA = AA_m(II,FF);
    ROS_INFO("ASDFASDFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF");
    // A << 1,dt,dt*dt/2,\
    //          0,1,dt,\
    //          0,0,1;\

    // B << (dt*dt*dt)/6,(dt*dt)/2,dt;
    

    // C << 1,0,-z_c/g;

    // Qx << 0,0,0,\
    //       0,0,0,\
    //       0,0,0;\

    // Q_m << 1,0,0,0,\
    //        0,0,0,0,\
    //        0,0,0,0,\
    //        0,0,0,0;\

    // BB << a,\
    //       C;\

    // II << 1,0,0,0;
 
    // FF << C*A,\
    //       A;\
    
    // AA << II,FF;
    
      KK=ZMP_DARE(AA,BB,Q,R);
      Eigen::MatrixXd Gi(1,1);

      Eigen::MatrixXd Gx(1,3),AAc(4,4),XX(4,1);
      Gi=(R+BB.transpose()*KK*BB).inverse()*BB.transpose()*KK*II;
      Gx=(R+BB.transpose()*KK*BB).inverse()*BB.transpose()*KK*FF;
      AAc= AA-BB*(R+BB.transpose()*KK*BB).inverse()*BB.transpose()*KK*AA;
      Eigen::VectorXd Gp;
      Gp=VectorXd::Zero(N);
      ROS_INFO("ASDFASDFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF");
    for(int i=0;i<N;i++)//원래 이 for문 안에 index때문에 문제가 생겼었음 ㅇㅇ(Gp를 벡터로 만들었었는데 인덱스가 벡터의 크기를 초과해서 오류가 나옴)
    {
        if(i==0)
        {
            XX=-AAc.transpose()*KK*II;
            Gp(i)=-Gi(0,0);
        }
        else
        {
            XX = AAc.transpose()*XX;
            Gp(i)=((R+BB.transpose()*KK*BB).inverse()*BB.transpose()*XX)(0.0);
            //XX = AAc.transpose()*XX;    
        }
        //history_Gp(i) = Gp(i);
    }
    ROS_INFO("ASDFASDFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF");
    double u_x=0,u_y=0,zmp_x_old=0,zmp_y_old=0,err_x =0,err_y =0,sum_e_x = 0,sum_e_y = 0,u_sum_p_x=0,u_sum_p_y = 0;
    Eigen::MatrixXd zmp_x_ref,zmp_y_ref,zmp_x,zmp_y,com_x,com_y;
    zmp_x_ref= MatrixXd::Zero(1,n);
    zmp_y_ref= MatrixXd::Zero(1,n);
    zmp_x= MatrixXd::Zero(1,n);
    zmp_y= MatrixXd::Zero(1,n);
    com_x= MatrixXd::Zero(1,n);
    com_y= MatrixXd::Zero(1,n);

    Eigen::VectorXd X,X_new,Y,Y_new;
    X= VectorXd::Zero(3);
    Y= VectorXd::Zero(3);
    X_new= VectorXd::Zero(3);
    Y_new= VectorXd::Zero(3);
    double time=0;
    //zmp ref calculrate
    for(int i=0;i<n;i++)//running time 15s 
    {
        //time = (i-1)*dt;
        // time = i*dt;
        // if (time < 3)
        // {zmp_x_ref(i) = 0;
        // zmp_y_ref(i) = 0;
        // }
        // else if (time < 4)
        // {
        // zmp_x_ref(i) = 0.1;
        // zmp_y_ref(i) = -0.1;
        // }
        // else if (time < 5)
        // {
        // zmp_x_ref(i) = 0.2;
        // zmp_y_ref(i) = 0.2;
        // }
        // else if (time < 6)
        // {
        // zmp_x_ref(i) = 0.3;
        // zmp_y_ref(i) = -0.05;
        // }
        // else if (time < 7)
        // {
        // zmp_x_ref(i) = 0.4;
        // zmp_y_ref(i) = 0.1;
        // }  
        // else
        // {zmp_x_ref(i) = 0.5;
        // zmp_y_ref(i) = 0.0;
        // }
        time = i*dt;
        if (time < 3)
        {zmp_x_ref(i) = 0;
        zmp_y_ref(i) = 0;
        }
        else if (time < 4)
        {
        zmp_x_ref(i) = 0.0;
        zmp_y_ref(i) = -0.5;
        }
        else if (time < 5)
        {
        zmp_x_ref(i) = 0.0;
        zmp_y_ref(i) = 0.5;
        }
        else if (time < 6)
        {
        zmp_x_ref(i) = 0.0;
        zmp_y_ref(i) = -0.5;
        }
        else if (time < 7)
        {
        zmp_x_ref(i) = 0.0;
        zmp_y_ref(i) = 0.5;
        }  
        else
        {zmp_x_ref(i) = 0.0;
        zmp_y_ref(i) = -0.5;
        }
    }
    
    for(int k=0;k<n-N;k++)
    {
     X = X_new;
     Y = Y_new;
      
     err_x = zmp_x_old - zmp_x_ref(k);
     err_y = zmp_y_old - zmp_y_ref(k);
     
     sum_e_x = sum_e_x + err_x; //error integral
     sum_e_y = sum_e_y + err_y; //error integral
     
     for (int j=0;j<N;j++)
     {   
        u_sum_p_x = u_sum_p_x + Gp(j) * zmp_x_ref(k+j);
        u_sum_p_y = u_sum_p_y + Gp(j) * zmp_y_ref(k+j);
     }     
     u_x = -Gi(0,0)*sum_e_x - (Gx*X)(0,0) - u_sum_p_x;
     u_y = -Gi(0,0)*sum_e_y - (Gx*Y)(0,0) - u_sum_p_y;
 //System state space Eq based New State Calculate & Update 
     X_new = A*X + B*u_x;
     Y_new = A*Y + B*u_y;
 //System state space Eq based ZMP Calculate & Update 
     zmp_x(k) = (C*X)(0,0);
     zmp_y(k) = (C*Y)(0,0);
     
     zmp_x_old = zmp_x(k);
     zmp_y_old = zmp_y(k);
     
     com_x(k) = X_new(0);
     com_y(k) = Y_new(0);
     
     u_sum_p_x = 0; //future u_sum_p_x initialize
     u_sum_p_y = 0;
    }
    ROS_INFO("ASDFASDFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF");
    int cnt = 0;
    //std::cout<<"Gp = "<<Gp<<std::endl;
    
    ros::init(argc, argv, "cubot");
    ros::NodeHandle nh;
    ros::Publisher chatter1_pub = nh.advertise<std_msgs::Float64>("ZMP_Ref_X",1000);
    ros::Publisher chatter2_pub = nh.advertise<std_msgs::Float64>("ZMP_Ref_Y",1000);
    ros::Publisher chatter3_pub = nh.advertise<std_msgs::Float64>("ZMP_X",1000);
    ros::Publisher chatter4_pub = nh.advertise<std_msgs::Float64>("ZMP_Y",1000);
    ros::Publisher chatter5_pub = nh.advertise<std_msgs::Float64>("COM_X",1000);
    ros::Publisher chatter6_pub = nh.advertise<std_msgs::Float64>("COM_Y",1000);

    ros::Rate loop_rate(200);

    while(ros::ok()){
        std_msgs::Float64 msg1;
        std_msgs::Float64 msg2;
        std_msgs::Float64 msg3;
        std_msgs::Float64 msg4;
        std_msgs::Float64 msg5;
        std_msgs::Float64 msg6;

        msg1.data = (float)(zmp_x_ref(0,cnt));
        msg2.data = (float)(zmp_y_ref(0,cnt));
        msg3.data = (float)(zmp_x(cnt));
        msg4.data = (float)(zmp_y(cnt));
        msg5.data = (float)(com_x(0,cnt));
        msg6.data = (float)(com_y(0,cnt));

        chatter1_pub.publish(msg1);
        chatter2_pub.publish(msg2);
        chatter3_pub.publish(msg3);
        chatter4_pub.publish(msg4);
        chatter5_pub.publish(msg5);
        chatter6_pub.publish(msg6);
        
        cnt++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    //cnt++;
    //ros::spinOnce();
    //loop_rate.sleep();




    return 0;    
}