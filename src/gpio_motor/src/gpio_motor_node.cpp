//
// Created by phil on 21.03.20.
//
#include <ros/ros.h>
#include <GpioWheel/GpioWheel.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_node");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}