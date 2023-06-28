#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

void callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
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