//
// Created by phil on 22.03.20.
//

#ifndef SRC_VACUUMPI_H
#define SRC_VACUUMPI_H

#include "RobotBase.h"
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <pi_ln298n/pi_ln298n.h>
using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

constexpr double POSITION_STEP_FACTOR = 10;
constexpr double VELOCITY_STEP_FACTOR = 10;

class VacuumPi : public RobotBase{
public:
    VacuumPi(ros::NodeHandle& nh);
    ~VacuumPi();
    void init();
    void update(const ros::TimerEvent& e);
    void read();
    void write(ros::Duration elapsed_time);

protected:
    pi_ln298n::GpioMotor _left;
    pi_ln298n::GpioMotor _right;
    ros::NodeHandle nh_;
    ros::Timer non_realtime_loop_;
    ros::Duration control_period_;
    ros::Duration elapsed_time_;
    PositionJointInterface positionJointInterface;
    PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    double p_error_, v_error_, e_error_;
};


#endif //SRC_VACUUMPI_H
