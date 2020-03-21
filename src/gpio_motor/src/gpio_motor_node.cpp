//
// Created by phil on 21.03.20.
//
#include <ros/ros.h>
#include <GpioWheel/GpioWheel.h>
#include <chrono>

constexpr int enA = 21;
constexpr int enB = 13;
constexpr int in1 = 20;
constexpr int in2 = 16;
constexpr int in3 = 26;
constexpr int in4 = 19;

std::shared_ptr<GpioWheel> left,right;

void leftCallBack(const std_msgs::String::ConstPtr& msg)
{
   ROS_INFO("I heard: [%s]", msg->data.c_str());

    auto start = std::chrono::system_clock::now();

    while (std::chrono::duration_cast<std::chrono::seconds>((std::chrono::system_clock::now() - start)).count() < 5.0)
    {
        left->forward();
    }
 }

void rightCallBack(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());

    auto start = std::chrono::system_clock::now();

    while (std::chrono::duration_cast<std::chrono::seconds>((std::chrono::system_clock::now() - start)).count() < 5.0)
    {
        right->forward();
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpio_motor");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);

    left = std::make_shared<GpioWheel>(in1,in2,enA,0.5);
    right = std::make_shared<GpioWheel>(in4,in3,enB,0.5);

    ros::Subscriber subLeft = n.subscribe("gpio_left", 1000, leftCallBack);
    ros::Subscriber subRight = n.subscribe("gpio_right", 1000, rightCallBack);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    left->stop();
    right->stop();
}