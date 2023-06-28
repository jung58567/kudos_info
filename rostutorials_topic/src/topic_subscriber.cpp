#include <ros/ros.h>
#include <ros_tutorials_topic/MsgTutorial.h>

void msgCallback(const ros_tutorials_topic::MsgTutorial::ConstPtr& msg) {
    ROS_INFO("Received message:");
    for (const auto& data : msg->data){
        ROS_INFO("data: %f", data);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("my_topic", 1000, callback);
    ros::spin();

    return 0;
}