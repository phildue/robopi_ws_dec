//
// Created by phil on 22.03.20.
//

#include "ActuatorInterface.h"

//TODO move this to config
constexpr int enA = 21;
constexpr int enB = 13;
constexpr int in1 = 20;
constexpr int in2 = 16;
constexpr int in3 = 26;
constexpr int in4 = 19;

ActuatorInterface::ActuatorInterface(ros::NodeHandle &nh) : _nh(nh)
                                                            {
    init();
    _controller_manager.reset(new controller_manager::ControllerManager(this, _nh));
    _nh.param("/robopi/actuator_interface/loop_hz", loop_hz_, 0.1);
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    non_realtime_loop_ = _nh.createTimer(update_freq, &ActuatorInterface::update, this);

    _nh.getParam("/robopi/gpio_wheels", _wheel_names);
    for(const auto &_wheel : _wheel_names)
    {
        int en,inF,inB;
        _nh.getParam("/robopi/gpio_wheels/en", en);
        _nh.getParam("/robopi/gpio_wheels/inF", inF);
        _nh.getParam("/robopi/gpio_wheels/inB", inB);
        _wheels.insert({_wheel,pi_ln298n::GpioMotor(inF,inB,en)});
    }
}

void ActuatorInterface::init()
{
    _num_joints = _wheel_names.size();

    joint_effort_.resize(_num_joints);
    joint_effort_command_.resize(_num_joints);

    // Initialize Controller
    for (int i = 0; i < _num_joints; ++i) {
        //TODO rework, potentially include all kind of controllers

        // Create joint state interface
        JointStateHandle jointStateHandle(_wheel_names[i], &_joint_position[i], &_joint_velocity[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        JointHandle joint_effort_handle(jointStateHandle, &joint_effort_command_[i]);
        JointLimits limits;
        getJointLimits(_wheel_names[i], _nh, limits);
        joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(joint_effort_handle, limits);
        effort_joint_saturation_interface_.registerHandle(jointLimitsHandle);
        _effortJointInterface.registerHandle(joint_effort_handle);

    }

    registerInterface(&joint_state_interface_);
    registerInterface(&_effortJointInterface);
    registerInterface(&effort_joint_saturation_interface_);
}

void ActuatorInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    _controller_manager->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ActuatorInterface::read() {
    for (int i = 0; i < _num_joints; i++) {
        joint_effort_[i] = _wheels.find(_wheel_names[i])->second.torque();
    }
}

void ActuatorInterface::write(ros::Duration elapsed_time) {
    effort_joint_saturation_interface_.enforceLimits(elapsed_time);
    for (int i = 0; i < _num_joints; i++) {
        _wheels.find(_wheel_names[i])->second.set(joint_effort_command_[i]);
    }
}