#ifndef _PID_CONT_HH_
#define _PID_CONT_HH_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <com_msgs/PDCmd.h>
#include <cont_msgs/Heading.h>
#include <cont_msgs/ContParams.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <utilities.hh>

double mass, g;
double kp_roll , kd_roll;
double kp_pitch, kd_pitch;
double kp_yaw  , kd_yaw;
double kp_x, kp_y, kp_z;
double kd_x, kd_y, kd_z;
double ki_x, ki_y, ki_z;
double d1, d2, kF;
double max_tilt, max_thrust;
double	max_dthrust, 
		max_dyaw,
		max_acc;

double refresh_rate;
bool   debug_mode;

// These are used to calculate derivatives (differences)
ros::Time curr_time;
ros::Time prev_time;
double dt;

int process_inputs(const ros::NodeHandle &n);
int setup_messaging_interface(ros::NodeHandle &n);
int loop(const ros::NodeHandle &n);
void heading_callback(const cont_msgs::Heading &msg);
void cont_params_callback(const cont_msgs::ContParams &msg);
int calculate_control_parameters();

#endif
