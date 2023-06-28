#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float32MultiArray.h>
#include "test_pub/test_pub.h"

using namespace std;
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

RT_TASK RT_task1;

//funtion
void process(int a,int b);
void dxl_initailize(void);//port, taket handler 호출
void set_dxl_goal(int dxl_1_posi, int dxl_2_posi);//모터를 원하는 위치로 보내줌
void dxl_go(void);//통신관련 함수
void read_dxl_position(void);//현재 모터의 위치 읽어옴
void dxl_add_param(void);//
void catch_signal(int sig);
void Callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

//variable
bool thread_is_running = true;
unsigned int cycle_ns = 1000000;
int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;   
bool dxl_addparam_result = false; 
uint8_t param_goal_position[4];
int32_t dxl1_present_position = 0;
int32_t dxl2_present_position = 0;

int left_x_region = 0;
int bottom_y_region = 0;
int right_x_region = 0;
int top_y_region = 0;

struct End
{
  int x=0,y=0;
};
End target;
class Region{
public:
  int i=0;
  float j=0.0;
};
Region *a= nullptr;
void process(int a,int b)
{
  // int pre_mot19,pre_mot20;
  // read_dxl_position();
  // pre_mot19=dxl1_present_position;
  // pre_mot20=dxl2_present_position;
  // cout<<pre_mot19<<" "<<pre_mot20<<endl;
  set_dxl_goal(a, b);
  dxl_go();
  groupSyncWrite.clearParam();
}
void serial_task(void* arg)
{
    
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns * 100);//0.1초에 한번씩 task 실행
    // cout<<"dddddddddddddddddddddddddddddddddddddd"<<endl;
    dxl_initailize();
    
    while (thread_is_running)
    {
      rt_task_wait_period(NULL);
      
      ROS_INFO("Object occupies regions: [%d, %d] - [%d, %d]", a[0].i, a[1].i, a[2].i, a[3].i);   
      if(213.3<a[0].j && a[0].j<426.6 && 160 <a[1].j && a[1].j <320)
      {
        ROS_INFO("Object is in the central region, do not control the neck");
      }
      else{
        ROS_INFO("Need to Neck control");
        if(a[0].j<213.3){
                read_dxl_position();
                target.x=dxl1_present_position+10;
                target.y=dxl2_present_position;
                process(target.x,target.y);
        }
        else if(a[0].j>426.6)
        {
                read_dxl_position();
                target.x=dxl1_present_position-10;
                target.y=dxl2_present_position;
                process(target.x,target.y);
        }
        if(a[1].j<160){
                read_dxl_position();
                target.x=dxl1_present_position;
                target.y=dxl2_present_position+10;
                process(target.x,target.y);
        }
        else if(a[1].j>320)
        {
                read_dxl_position();
                target.x=dxl1_present_position;
                target.y=dxl2_present_position-10;
                process(target.x,target.y);
        }
        // if(a[0].j<213.3){
            // if(a[1].j<160){
                // cout<<"Now==1"<<endl;
                // read_dxl_position();
                // target.x=dxl1_present_position+10;
                // target.y=dxl2_present_position-10;
                // process(target.x,target.y);
            // }
            // else if(a[1].j>160 && a[1].j<320)
            // {
                // cout<<"Now==4"<<endl;
                // read_dxl_position();
                // target.x=dxl1_present_position+10;
                // target.y=dxl2_present_position;
                // process(target.x,target.y);
            // }
            // else if(a[1].j>320){
                // cout<<"Now==7"<<endl;
                // read_dxl_position();
                // target.x=dxl1_present_position+10;
                // target.y=dxl2_present_position+10;
                // process(target.x,target.y);
            // }
        // }
        // else if(a[0].j>213.3 && a[0].j<426.6){
            // if(a[1].j<160){
              // cout<<"Now==2"<<endl;
              // read_dxl_position();
              // target.x=dxl1_present_position;
              // target.y=dxl2_present_position-10;
              // process(target.x,target.y);
            // }
            // else if(a[1].j>320)
            // {
              // cout<<"Now==8"<<endl;
              // read_dxl_position();
              // target.x=dxl1_present_position;
              // target.y=dxl2_present_position+10;
              // process(target.x,target.y);
            // }
        // }
        // else if(a[0].j>426.6)
        // {
            // if(a[1].j<160){
              // cout<<"Now==3"<<endl;
              // read_dxl_position();
              // target.x=dxl1_present_position-10;
              // target.y=dxl2_present_position-10;
              // process(target.x,target.y);
            // }
            // else if(a[1].j>160 && a[1].j<320)
            // {
              // cout<<"Now==6"<<endl;
              // read_dxl_position();
              // target.x=dxl1_present_position-10;
              // target.y=dxl2_present_position;
              // process(target.x,target.y);
            // }
            // else if(a[1].j>320){
              // cout<<"Now==9"<<endl;
              // read_dxl_position();
              // target.x=dxl1_present_position-10;
              // target.y=dxl2_present_position+10;
              // process(target.x,target.y);
            // }
        // }
      }
      
      // cout<<"dddddddddddddddddddddddddddddddddddddd"<<endl;
    }
    return;
}

void Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
   
   std::vector<float> bounding_box = msg->data;
   int image_width = 640;
   int image_height = 480;

  
  //영역의 개수와 크기 계산
   int num_rows = 3;
   int num_cols = 3;
   int region_width = image_width / num_cols;
   int region_height = image_height / num_rows;

// 바운딩 박스의 좌측 상단과 우측 하단 좌표
  float box_x_left = bounding_box[0];
  float box_y_bottom = bounding_box[1];
  float box_x_right = bounding_box[2];
  float box_y_top = bounding_box[3];

  float center_x = (box_x_left + box_x_right)/2.0;
  float center_y = (box_y_bottom + box_y_top)/2.0;
 // 바운딩 박스가 속한 영역 계산
  int left_x_region = box_x_left / region_width;
  int bottom_y_region = box_y_bottom / region_height;
  int right_x_region = box_x_right / region_width;
  int top_y_region = box_y_top / region_height;

  //std::lock_guard<std::mutex> lock(mtx);
  left_x_region = std::max(0, left_x_region);
  bottom_y_region = std::max(0, bottom_y_region);
  right_x_region = std::min(num_cols - 1, right_x_region);
  top_y_region = std::min(num_rows - 1, top_y_region);

  a[0].i = left_x_region;
  a[1].i = bottom_y_region;
  a[2].i = right_x_region;
  a[3].i = top_y_region;
  
  a[0].j=center_x;
  a[1].j=center_y;
  // ROS_INFO("Object occupies regions: [%d, %d] - [%d, %d]", left_x_region, bottom_y_region, right_x_region, top_y_region);   

  }
void catch_signal(int sig) {
    //signal(sig, SIG_IGN);
    printf("Program END...\n");
    cout<<"program end"<<endl;

    //close(dxl.serial_port);
    printf("Program END...\n");
    ros::shutdown();
    exit(0);
}
void dxl_initailize(void) {

  portHandler->openPort();

  portHandler->setBaudRate(BAUDRATE);

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_add_param();
}

void dxl_add_param(void){

  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
  if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
    }
  dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
  if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSync Read addparam failed", DXL2_ID);
    }
  }
//goal position 지정
void set_dxl_goal(int dxl_1_posi, int dxl_2_posi) {

  ROS_INFO("dxl_pos_1 : %d", dxl_1_posi);
  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_1_posi));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_1_posi));

  dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);

  ROS_INFO("dxl_pos_2 : %d", dxl_2_posi);
  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_2_posi));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_2_posi));

  dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
}
//통신보내주는 역할
void dxl_go(void) {
  dxl_comm_result = groupSyncWrite.txPacket();
  //ROS_INFO("%d", dxl_comm_result);
  //if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

}
void read_dxl_position(void){

 dxl_comm_result = groupSyncRead.txRxPacket();
 dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
 dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
}
int main(int argc, char **argv)
{
  a= new Region[4];

  ros::init(argc, argv, "bounding_box_pub");
  ros::NodeHandle nh;  
  ros::Subscriber sub = nh.subscribe("bounding_box_pub", 1000, Callback);

  signal(SIGINT, catch_signal);
  signal(SIGTERM,catch_signal);
  
  rt_task_create(&RT_task1, "serial_task", 0, 99, 0);
  rt_task_start(&RT_task1, &serial_task, NULL);
  
    while (ros::ok())
    {
        ros::spinOnce();
    }

  return 0;
}