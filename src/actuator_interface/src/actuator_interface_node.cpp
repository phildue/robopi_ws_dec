//
// Created by phil on 22.03.20.
//

#include <controller_manager/controller_manager.h>
#include "ActuatorInterface.h"
#include <ros/callback_queue.h>
#include <ros/ros.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "actuator_interface");
    ros::CallbackQueue callbackQueue;
    ros::NodeHandle nh;
    nh.setCallbackQueue(&callbackQueue);
    ActuatorInterface rhi(nh);
    //ros::AsyncSpinner spinner(0,&callbackQueue);
    //spinner.start();
    //ros::waitForShutdown();
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&callbackQueue);
    return 0;
}