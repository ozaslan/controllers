#include "pid_cont.hh"

com_msgs::PDCmd PIDController::generate_command(const cont_msgs::Heading &heading){
	// Message to return
	com_msgs::PDCmd pdcmd_msg;
	ros::Time curr_time = heading.header.stamp;

	double dt = ((double)curr_time.sec  - _prev_time.sec ) + 
				((double)curr_time.nsec - _prev_time.nsec) / 1e9;
	_prev_time = curr_time;
	
	Eigen::Vector4d quat;	// misc. variables	
	Eigen::Vector3d rpy;	// ''	 ''

	// Convert current orientation to Euler angles
	quat(0) = heading.quat_from.w;
	quat(1) = heading.quat_from.x;
	quat(2) = heading.quat_from.y;
	quat(3) = heading.quat_from.z;
	rpy = utils::trans::quat2rpy(quat);	
	double psi_from = rpy(2);
	// Conver goal orientation to Euler angles
	quat(0) = heading.quat_to.w;
	quat(1) = heading.quat_to.x;
	quat(2) = heading.quat_to.y;
	quat(3) = heading.quat_to.z;
	rpy = utils::trans::quat2rpy(quat);	
	float psi_to = rpy(2);

	// Constrain the maximum yaw speed.
	double dpsi = utils::fix_angle(psi_to - psi_from) / dt;
	//cout << "dpsi = " << dpsi << endl;
	//cout << "psi_to = " << psi_to << " psi_from = " << psi_from << endl;
	dpsi = utils::clamp(dpsi, -_params.max_dyaw, _params.max_dyaw);
	psi_to = utils::fix_angle(psi_from + dt * dpsi);

	// Initialize intermediate variables
	Eigen::Vector3d acc_to, vel_to  , pos_to, 
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

  //cout << "acc_des[0] = " << acc_des << endl;
  //cout << "max_acc = " << _params.max_acc << endl;

	_int_err += (pos_to - pos_from) * dt;

	// Limit the accelerations and rotational speeds
	if(acc_des.norm() > _params.max_acc)
		acc_des *= _params.max_acc / (acc_des.norm() + 1e-9);

	euler_des(0) = (1/_params.g) * (acc_des(0) * sin(psi_from) - acc_des(1) * cos(psi_from));
	euler_des(1) = (1/_params.g) * (acc_des(0) * cos(psi_from) + acc_des(1) * sin(psi_from));
	euler_des(2) = psi_to;
	// Saturate resultant inputs
	euler_des(0) = utils::clamp(euler_des(0), -_params.max_tilt, _params.max_tilt);
	euler_des(1) = utils::clamp(euler_des(1), -_params.max_tilt, _params.max_tilt);

	// Convert accelation to motor input (thrust := grams)
  //cout << "acc_des[1] = " << acc_des << endl;
  //cout << "_params.g = " << _params.g << endl;
	double thrust = _params.mass * (acc_des(2) + _params.g) / _params.g * 1000;
  //cout << "thrust[0] = " << thrust << endl;
	thrust = thrust < 1 ? 1 : thrust;
	// Prevent sudden step-ups
	double dthrust = (thrust - _prev_thrust) / dt;
	
	if(dthrust >= 0 && dthrust >= _params.max_dthrust)
		thrust = _prev_thrust + dt * _params.max_dthrust;
	if(thrust > _params.max_thrust)
		thrust = _params.max_thrust;
	_prev_thrust = thrust;

  //cout << "thrust[1] = " << thrust << endl;
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
	pdcmd_msg.kp_yaw 	= _params.kp_yaw;
	pdcmd_msg.kd_yaw 	= _params.kd_yaw;

  /*
	cout << "pdcmd_msg :" << endl;
	cout << "thrust = " << pdcmd_msg.thrust << endl;
	cout << "roll   = " << pdcmd_msg.roll   << endl;
	cout << "pitch  = " << pdcmd_msg.pitch << endl;
	cout << "yaw    = " << pdcmd_msg.yaw << endl;
	cout << "dyaw   = " << pdcmd_msg.dyaw << endl;
	cout << "kp_roll = " << pdcmd_msg.kp_roll << endl;
	cout << "kd_roll = " << pdcmd_msg.kd_roll << endl;
	cout << "kp_pitch = " << pdcmd_msg.kp_pitch << endl;
	cout << "kd_pitch = " << pdcmd_msg.kd_pitch << endl;
	cout << "kp_yaw = " << pdcmd_msg.kp_yaw << endl;
	cout << "kd_yaw = " << pdcmd_msg.kd_yaw << endl;
  */
	return pdcmd_msg;
}

void PIDController::set_params(const cont_msgs::PIDParams &params){
	_params = params;
}

const cont_msgs::PIDParams & PIDController::get_params(){
	return _params;
}

