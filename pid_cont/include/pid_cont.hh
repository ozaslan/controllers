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

#endif
