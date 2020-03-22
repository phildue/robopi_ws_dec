//
// Created by phil on 22.03.20.
//

#include "RobotBase.h"
#include <controller_manager/controller_manager.h>
int main()
{
    RobotBase robot;
    controller_manager::ControllerManager cm(&robot);
    ros::Time ts = ros::Time::now();
    while (true)
    {
        ros::Duration d = ros::Time::now() - ts;
        ts = ros::Time::now();
        robot.read(ts,d);
        cm.update(ts, d);
        robot.write(ts,d);
        sleep(1);
    }
}