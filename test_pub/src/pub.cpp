#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "pub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("my_topic", 1000);
    ros::Rate rate(1);  // 1Hz로 발행 속도 설정
    int i=0;
    std::vector<float> numbers(1000); 
    float j=1.0; 
    while (ros::ok()) {
        std_msgs::Float32MultiArray msg;
        
        //std::vector<float> data = {1.0, 2.0, 3.0};
        //msg.data = "Hello, subscribers!";
        //numbers.push_back(i);
        //a[i]=float(i);
        numbers.push_back(j);
        msg.data=numbers;
        pub.publish(msg);
        j+=1.0;
        //i++;
        rate.sleep();
    }

    return 0;
}