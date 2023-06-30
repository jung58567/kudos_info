#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <stdlib.h>
#include <time.h>
using namespace std;

int main(int argc, char **argv)
{
    srand((unsigned int)time(NULL));
    //std_msgs::Int32 num = rand()%5;


    ros::init(argc, argv, "talker");

    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<std_msgs::Int32>("chatter", 1000);

    ros:: Rate loop_rate(1);
    std_msgs::Int32 msg;

    msg.data=rand()%5;
    //int count=0;
    while(ros::ok())
    {
//        std_msgs::Int32 msg;
//        msg.data=rand()%5;
        ROS_INFO("i speak : [%d]",msg.data);
        chatter_pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
//        ++count;
    }
    return 0;
}
