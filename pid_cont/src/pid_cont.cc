#include "pid_cont.hh"

com_msgs::PDCmd PIDController::generate_command(const cont_msgs::Heading &heading){
	// Message to return
	com_msgs::PDCmd pdcmd_msg;
	ros::Time curr_time = heading.header.stamp;

	double dt = ((double)curr_time.sec  - _prev_time.sec ) + 
		((double)curr_time.nsec - _prev_time.nsec) / 1e9;
	_prev_time = curr_time;

	// Convert current and goal orientation to Euler angles
	double psi_from = utils::trans::quat2rpy(utils::trans::quat2quat(heading.quat_from))(2);	
	double psi_to   = utils::trans::quat2rpy(utils::trans::quat2quat(heading.quat_to  ))(2);	

  //cout << "psi_from = " << psi_from << endl;
  //cout << "psi_to   = " << psi_to   << endl;

	// Constrain the maximum yaw speed.
  /*
	double dpsi = utils::fix_angle(psi_to - psi_from) / dt;
	dpsi = utils::clamp(dpsi, -_params.max_dyaw, _params.max_dyaw);
  ### On-board controller prevents fast rotations. But I still might want to enable this feature.
  */
	// Initialize intermediate variables
	Eigen::Vector3d acc_to, vel_to, pos_to, 
              		vel_from, pos_from;
	Eigen::Vector3d acc_des, euler_des;

	pos_from(0) = heading.pos_from.x;	
	pos_from(1) = heading.pos_from.y;
	pos_from(2) = heading.pos_from.z;
	vel_from(0) = heading.vel_from.x;	
	vel_from(1) = heading.vel_from.y;
	vel_from(2) = heading.vel_from.z;

	pos_to(0) = heading.pos_to.x;	
	pos_to(1) = heading.pos_to.y;	
	pos_to(2) = heading.pos_to.z;	
	vel_to(0) = heading.vel_to.x;	
	vel_to(1) = heading.vel_to.y;
	vel_to(2) = heading.vel_to.z;	
	acc_to(0) = heading.acc_to.x;	
	acc_to(1) = heading.acc_to.y;	
	acc_to(2) = heading.acc_to.z;	

	// ### I still do not use KI
	// Calculate the required acceleration and orientation
	acc_des(0) = acc_to(0) + _params.kd_x * (vel_to(0) - vel_from(0)) + _params.kp_x * (pos_to(0) - pos_from(0)) + _params.ki_x * _int_err(0);
	acc_des(1) = acc_to(1) + _params.kd_y * (vel_to(1) - vel_from(1)) + _params.kp_y * (pos_to(1) - pos_from(1)) + _params.ki_y * _int_err(1);
	acc_des(2) = acc_to(2) + _params.kd_z * (vel_to(2) - vel_from(2)) + _params.kp_z * (pos_to(2) - pos_from(2)) + _params.ki_z * _int_err(2);

	_int_err += (pos_to - pos_from) * dt;

	// Limit the accelerations and rotational speeds
	if(acc_des.norm() > _params.max_acc)
		acc_des *= _params.max_acc / (acc_des.norm() + 1e-9);

	euler_des(0) = (1/_params.g) * (acc_des(0) * sin(psi_from) - acc_des(1) * cos(psi_from));
	euler_des(1) = (1/_params.g) * (acc_des(0) * cos(psi_from) + acc_des(1) * sin(psi_from));
	euler_des(2) = psi_from + 
                  _params.kp_yaw * /*utils::fix_angle###*/(psi_to - psi_from) +
                  _params.kd_yaw * (heading.omega_to.z - heading.omega_from.z);

  

	// Saturate resultant inputs
	euler_des(0) = utils::clamp(euler_des(0), -_params.max_tilt, _params.max_tilt);
	euler_des(1) = utils::clamp(euler_des(1), -_params.max_tilt, _params.max_tilt);

	// Convert accelation to motor input (thrust := grams)
	double thrust = _params.mass * (acc_des(2) + _params.g) / _params.g * 1000;
	thrust = thrust < 1 ? 1 : thrust;

	// Prevent sudden step-ups
	double dthrust = (thrust - _prev_thrust) / dt;
	if(dthrust >= 0 && dthrust >= _params.max_dthrust)
		thrust = _prev_thrust + dt * _params.max_dthrust;
	if(thrust > _params.max_thrust)
		thrust = _params.max_thrust;
	_prev_thrust = thrust;

	// Pass the inputs to the driver side
	pdcmd_msg.thrust	= thrust;
	pdcmd_msg.roll		= euler_des(0);
	pdcmd_msg.pitch		= euler_des(1);
	pdcmd_msg.yaw		  = euler_des(2);
	pdcmd_msg.droll		= 0;
	pdcmd_msg.dpitch	= 0;
	pdcmd_msg.dyaw		= heading.omega_to.z;
	pdcmd_msg.kp_roll 	= _params.kp_roll;
	pdcmd_msg.kd_roll 	= _params.kd_roll;
	pdcmd_msg.kp_pitch	= _params.kp_pitch;
	pdcmd_msg.kd_pitch	= _params.kd_pitch;
	pdcmd_msg.kp_yaw 	  = _params.kp_yaw;
	pdcmd_msg.kd_yaw 	  = _params.kd_yaw;

	return pdcmd_msg;
}

void PIDController::set_params(const cont_msgs::PIDParams &params){
	_params = params;
}

const cont_msgs::PIDParams & PIDController::get_params(){
	return _params;
}

