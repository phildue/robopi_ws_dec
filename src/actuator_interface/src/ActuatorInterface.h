//
// Created by phil on 22.03.20.
//

#ifndef VACUUMPI_H
#define VACUUMPI_H

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

class ActuatorInterface : public hardware_interface::RobotHW{
public:
    explicit ActuatorInterface(ros::NodeHandle& nh);
    ~ActuatorInterface() = default;
    void init();
    void update(const ros::TimerEvent& e);
    void read();
    void write(ros::Duration elapsed_time);

protected:
    std::map<std::string,pi_ln298n::GpioMotor> _wheels;
    std::vector<double> joint_effort_,_joint_position,_joint_velocity;
    std::vector<double> joint_effort_command_;

    ros::NodeHandle _nh;
    ros::Timer non_realtime_loop_;
    ros::Duration control_period_;
    ros::Duration elapsed_time_;
    JointStateInterface joint_state_interface_;
    EffortJointInterface _effortJointInterface;
    joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_interface_;
    int _num_joints;
    std::vector<std::string> _wheel_names;
    double loop_hz_;
    std::shared_ptr<controller_manager::ControllerManager> _controller_manager;
    double p_error_, v_error_, e_error_;
};


#endif //SRC_VACUUMPI_H
