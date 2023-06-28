#include <ros/ros.h>
#include <ros_tutorials_topic/MsgTutorial.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "topic_publisher");//노드명 초기화
    ros::NodeHandle nh;//노드 핸들러 선언
    ros::Publisher ros_tutorial_pub = nh.advertise<ros_tutorials_topic::MsgTutorial>("ros_tutorial_msg", 100);
    ros::Rate loop_rate(10);  // 10Hz로 발행 속도 설정
    ros_tutorials_topic::MsgTutorial msg;
    int count = 0;
   
    while (ros::ok()) {
        msg.stamp = ros::Time::now();
        msg.data = count;
        
        ROS_INFO("send msg = %d",msg.stamp.sec);
        ROS_INFO("send msg = %d",msg.stamp.nsec);
        ROS_INFO("send msg = %d",msg.data);
        
    
        ros_tutorial_pub.publish(msg);
        
        loop_rate.sleep();
        ++count;
    }

    return 0;
}