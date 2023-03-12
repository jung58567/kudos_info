/*kubot22p_pkg

*/
//Header file for C++
#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>
#include <time.h>

//Header file for Gazebo and Ros
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>
#include <Eigen/Dense>
using Eigen::Matrix3d;
using Eigen::Vector3d;

#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI

//Print color 프린트f 할 때 색깔 넣어주는거 
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"

using namespace std;

namespace gazebo //gazebo api사용하기 위해 namespace선언
{
    class kubot2_plugin : public ModelPlugin //ModelPlugin 쓰는 이유는 모델만 제어하기 위해서
    {
        //*** Variables for kubot Simulation in Gazebo ***//
        //* TIME variable
        common::Time last_update_time;
        event::ConnectionPtr update_connection; //ConnectionPtr 뭔가 연결해주는 포인터같은데 api검색해봐도 잘 안나오네
        double dt;
        double time = 0;

        physics::ModelPtr model;  //physics라는 class안에서 ModelPtr라는 포인터 생성 model파일 불러오기 위해서 필요함

        physics::JointPtr L_Hip_yaw_joint;  //physics라는 class에서 JointPtr라는 포인터 생성
        physics::JointPtr L_Hip_roll_joint;
        physics::JointPtr L_Hip_pitch_joint;
        physics::JointPtr L_Knee_joint;
        physics::JointPtr L_Ankle_pitch_joint;
        physics::JointPtr L_Ankle_roll_joint;

        physics::JointPtr R_Hip_yaw_joint;
        physics::JointPtr R_Hip_roll_joint;
        physics::JointPtr R_Hip_pitch_joint;
        physics::JointPtr R_Knee_joint;
        physics::JointPtr R_Ankle_pitch_joint;
        physics::JointPtr R_Ankle_roll_joint;

        sensors::SensorPtr Sensor;//imu data를 받기위한 포인터 선언
        sensors::ImuSensorPtr IMU;
        Vector3d IMU_theta;

        enum
        {//enum열거형을 사용해 각 모터별 번호를 부여한다.
            LHY = 0, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR
        };

        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot

        typedef struct RobotJoint //Joint variable struct for joint control 
        {
            double targetDegree; //The target deg, [deg]
            double targetRadian; //The target rad, [rad]
            double targetRadian_interpolation; //The interpolated target rad, [rad]

            double targetVelocity; //The target vel, [rad/s]
            double targetTorque; //The target torque, [N·m]

            double actualDegree; //The actual deg, [deg]
            double actualRadian; //The actual rad, [rad]
            double actualVelocity; //The actual vel, [rad/s]
            double actualRPM; //The actual rpm of input stage, [rpm]
            double actualTorque; //The actual torque, [N·m]

            double Kp;
            double Ki;
            double Kd;
        } ROBO_JOINT;//RobotJoint라는 구조체를 만들어서 모터별 특정한 각도 라디안 토크등 다양한 물리값을 받아오기 위해 구조체를 선언한다.(일일이 선언해주기 귀찮으니까)
        ROBO_JOINT* joint;
    public : //kubot2_plugin이라는 class안에 public부분에는 사용할 함수들을 선언해준다.
    //*** Functions for kubot Simulation in Gazebo ***//
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
    void UpdateAlgorithm(); // Algorithm update while simulation
    void getJoints();  // Get each joint data from [physics::ModelPtr _model]
    void getjointData(); // Get encoder data of each joint
    void jointcontroller();
    void getsensor();
    void getsensorData();
    };
    GZ_REGISTER_MODEL_PLUGIN(kubot2_plugin);//뭔진 잘 모르겠지만 필요한 함수라고 함
}


void gazebo::kubot2_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    model=_model;
    getJoints(); 
    getsensor();

    nDoF = 12; // Get degrees of freedom, except position and orientation of the robot
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct
    //new라는 건 동적할당이다. 동적할당이란 새로운 저장공간을 만들어준다는 의미
    //joint라는 포인터에다 ROBO_JOINT라는 구조체를 12개 즉 모터의 개수만큼 만들어 준다는 의미인 듯

    
    last_update_time = model->GetWorld()->SimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&kubot2_plugin::UpdateAlgorithm, this));
}

void gazebo::kubot2_plugin::UpdateAlgorithm()
{
    //* UPDATE TIME : 1ms
    common::Time current_time = model->GetWorld()->SimTime();
    dt = current_time.Double() - last_update_time.Double();
    //    cout << "dt:" << dt << endl;
    time = time + dt;
    //    cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;
    //printf(C_RED "time = %f\n" C_MAGENTA,time);
    getjointData();
    jointcontroller();
    getsensorData();
}


void gazebo::kubot2_plugin::getJoints()//가제보로 연결된 모터 정보 연결
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */

    //* Joint specified in model.sdf
    
    //gazebo상에서 돌아가는 로봇의 관절정보를 
    L_Hip_yaw_joint = this->model->GetJoint("L_Hip_yaw_joint");//sdf파일에 LP라는 관절의 정보를 L_Hip_yaw_joint라는 포인터를 통해 정보를 가르킴
    L_Hip_roll_joint = this->model->GetJoint("L_Hip_roll_joint");
    L_Hip_pitch_joint = this->model->GetJoint("L_Hip_pitch_joint");
    L_Knee_joint = this->model->GetJoint("L_Knee_joint");
    L_Ankle_pitch_joint = this->model->GetJoint("L_Ankle_pitch_joint");
    L_Ankle_roll_joint = this->model->GetJoint("L_Ankle_roll_joint");

    R_Hip_yaw_joint = this->model->GetJoint("R_Hip_yaw_joint");
    R_Hip_roll_joint = this->model->GetJoint("R_Hip_roll_joint");
    R_Hip_pitch_joint = this->model->GetJoint("R_Hip_pitch_joint");
    R_Knee_joint = this->model->GetJoint("R_Knee_joint");
    R_Ankle_pitch_joint = this->model->GetJoint("R_Ankle_pitch_joint");
    R_Ankle_roll_joint = this->model->GetJoint("R_Ankle_roll_joint");
    
    //* FTsensor joint
    //LS = this->model->GetJoint("LS");
    //RS = this->model->GetJoint("RS");
}


void gazebo::kubot2_plugin::getjointData()
{
    /*
     * Get encoder and velocity data of each joint[j].targetRadian = joint_h[j];
     * encoder unit : [rad] and unit conversion to [deg]
     * velocity unit : [rad/s] and unit conversion to [rpm]
     */
   // joint[WST].actualRadian = Torso_yaw_joint->Position(0);

    joint[LHY].actualRadian = L_Hip_yaw_joint->Position(0);
    joint[LHR].actualRadian = L_Hip_roll_joint->Position(0);
    joint[LHP].actualRadian = L_Hip_pitch_joint->Position(0);
    joint[LKN].actualRadian = L_Knee_joint->Position(0);
    joint[LAP].actualRadian = L_Ankle_pitch_joint->Position(0);
    joint[LAR].actualRadian = L_Ankle_roll_joint->Position(0);

    joint[RHY].actualRadian = R_Hip_yaw_joint->Position(0);
    joint[RHR].actualRadian = R_Hip_roll_joint->Position(0);
    joint[RHP].actualRadian = R_Hip_pitch_joint->Position(0);
    joint[RKN].actualRadian = R_Knee_joint->Position(0);
    joint[RAP].actualRadian = R_Ankle_pitch_joint->Position(0);
    joint[RAR].actualRadian = R_Ankle_roll_joint->Position(0);

    for (int j = 0; j < nDoF; j++) {
        joint[j].actualDegree = joint[j].actualRadian*R2D;
        //printf(C_BLUE "joint[%d].actualDegree = %f\n" C_RESET,j, joint[j].actualDegree);
    }

    //joint[WST].actualVelocity = Torso_yaw_joint->GetVelocity(0);

    joint[LHY].actualVelocity = L_Hip_yaw_joint->GetVelocity(0);
    joint[LHR].actualVelocity = L_Hip_roll_joint->GetVelocity(0);
    joint[LHP].actualVelocity = L_Hip_pitch_joint->GetVelocity(0);
    joint[LKN].actualVelocity = L_Knee_joint->GetVelocity(0);
    joint[LAP].actualVelocity = L_Ankle_pitch_joint->GetVelocity(0);
    joint[LAR].actualVelocity = L_Ankle_roll_joint->GetVelocity(0);

    joint[RHY].actualVelocity = R_Hip_yaw_joint->GetVelocity(0);
    joint[RHR].actualVelocity = R_Hip_roll_joint->GetVelocity(0);
    joint[RHP].actualVelocity = R_Hip_pitch_joint->GetVelocity(0);
    joint[RKN].actualVelocity = R_Knee_joint->GetVelocity(0);
    joint[RAP].actualVelocity = R_Ankle_pitch_joint->GetVelocity(0);
    joint[RAR].actualVelocity = R_Ankle_roll_joint->GetVelocity(0);

    //joint[WST].actualTorque = Torso_yaw_joint->GetForce(0);

    joint[LHY].actualTorque = L_Hip_yaw_joint->GetForce(0);
    joint[LHR].actualTorque = L_Hip_roll_joint->GetForce(0);
    joint[LHP].actualTorque = L_Hip_pitch_joint->GetForce(0);
    joint[LKN].actualTorque = L_Knee_joint->GetForce(0);
    joint[LAP].actualTorque = L_Ankle_pitch_joint->GetForce(0);
    joint[LAR].actualTorque = L_Ankle_roll_joint->GetForce(0);

    joint[RHY].actualTorque = R_Hip_yaw_joint->GetForce(0);
    joint[RHR].actualTorque = R_Hip_roll_joint->GetForce(0);
    joint[RHP].actualTorque = R_Hip_pitch_joint->GetForce(0);
    joint[RKN].actualTorque = R_Knee_joint->GetForce(0);
    joint[RAP].actualTorque = R_Ankle_pitch_joint->GetForce(0);
    joint[RAR].actualTorque = R_Ankle_roll_joint->GetForce(0);

    // for (int j = 0; j < nDoF; j++) {

    //     joint[j].actualRPM = joint[j].actualVelocity * 60. / (2 * PI);
    // }

    // //* plugin -> CRobot Class, joint data update
    // for (int j = 0; j < nDoF; j++) {

    //     RoK.joint[j].currentAngle = joint[j].actualRadian;
    //     RoK.joint[j].currentVel = joint[j].actualVelocity;
    // }
}

void gazebo::kubot2_plugin::jointcontroller()
{
    double kp, kd;
    double target_radian;
    //target_radian = 90*D2R;
    //target_velocity
    
    kp=10;
    kd=0.1;
    //joint[LKN].targetTorque=kp*(target_radian-joint[LKN].actualRadian)\
    //                        +kd*(-joint[LKN].actualVelocity);
    //단위는 N(뉴턴)이다
    //L_Hip_yaw_joint->SetForce(1, joint[LHY].targetTorque);//z축이기 때문에 setforce(2)인거 같다.-> sdf파일에서 effort의 토크의 한계를 제한해서 안 돌아갔음
    //sdf파일에서 limit joint의 한계를 정해줄 수 있음
 

    target_radian=10;
    // joint[LKN].targetTorque = target_radian;
    // L_Knee_joint->SetForce(0, joint[LKN].targetTorque);

    joint[LAP].targetTorque = target_radian;
    L_Ankle_pitch_joint->SetForce(0, joint[LKN].targetTorque);
}

void gazebo::kubot2_plugin::getsensor()
{
    Sensor = sensors::get_sensor("IMU");
    IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);
}

void gazebo::kubot2_plugin::getsensorData()
{
    // IMU_theta[Roll] = IMU->Orientation().Euler()[Roll];
    // IMU_theta[Pitch] = IMU->Orientation().Euler()[Pitch];
    // IMU_theta[Yaw] = IMU->Orientation().Euler()[Yaw];

    IMU_theta[0] = IMU->Orientation().Euler()[0];
    IMU_theta[1] = IMU->Orientation().Euler()[1];
    IMU_theta[2] = IMU->Orientation().Euler()[2];
    // IMU_dtheta[Roll] = IMU->AngularVelocity(false)[Roll];
    // IMU_dtheta[Pitch] = IMU->AngularVelocity(false)[Pitch];
    // IMU_dtheta[Yaw] = IMU->AngularVelocity(false)[Yaw];
    
    // printf("roll= %f\n",IMU_theta[0]*R2D);
    // printf("pitch= %f\n",IMU_theta[1]*R2D);
    // printf("yaw= %f\n",IMU_theta[2]*R2D);

    //RoK.imu.theta = IMU_theta;
    //RoK.imu.dtheta = IMU_dtheta;
}