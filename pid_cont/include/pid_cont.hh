#ifndef _PID_CONT_HH_
#define _PID_CONT_HH_

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <utils.hh>
#include <com_msgs/PDCmd.h>
#include <cont_msgs/Heading.h>
#include <cont_msgs/ContParams.h>

class PIDController{
private:
  cont_msgs::PIDParams _params;
  ros::Time _prev_time;
	Eigen::Vector3d _int_err;
  double _prev_thrust;
public:
  PIDController(): _int_err(0, 0, 0), _prev_thrust(0){}
  com_msgs::PDCmd generate_command(const cont_msgs::Heading &heading);
  void set_params(const cont_msgs::PIDParams &params);
  const cont_msgs::PIDParams & get_params();
};

/*
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
*/

#endif
