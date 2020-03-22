//
// Created by phil on 22.03.20.
//

#include <controller_manager/controller_manager.h>
#include "ActuatorInterface.h"
#include <ros/ros.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "actuator_interface");

    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);
    ActuatorInterface rhi(n);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}