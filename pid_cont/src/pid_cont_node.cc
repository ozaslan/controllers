#include "pid_cont.hh"

#ifndef PI
	#define PI 3.14159265359
#endif
#define deg2rad(x) ((x) / 180.0 * PI)
#define Dt(t2, t1) (((double)t2.sec  - t1.sec ) + \
					((double)t2.nsec - t1.nsec) / 1e9);

using namespace std;
using namespace Eigen;

cont_msgs::Heading    heading_msg;
com_msgs::PDCmd       pdcmd_msg;

ros::Subscriber       heading_subs;
ros::Subscriber       cont_params_subs;
ros::Publisher        pdcmd_publ;

// -- Process inputs
// -- Setup messaging interface
// -- Get the Heading message
// -- Prepare the PD Command
// -- Send the command
// ** Update the PID coefficients

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pid_cont_node");
  	ros::NodeHandle n("~");

	process_inputs(n);

	setup_messaging_interface(n);

	loop(n);	
	
	return 1;
}

int process_inputs(const ros::NodeHandle &n)
{
	// ### KHex accepts 'grams' as thrust whereas Asctec accepts
	// some number between [50-250]. In this node I vote for using
	// grams and let the communication nodes handle this detail.
	// I will keep the previous code commented in order not to lose
	// the conversion expression.
	g = 9.81;
	// These default values are for the Pelican robot
	// and g is for the Earth :)
	n.param("mass", mass, 1.327);
	n.param("kp_roll", kp_roll, 258.0);
	n.param("kd_roll", kd_roll, 20.0);
	n.param("kp_pitch", kp_pitch, 258.0);
	n.param("kd_pitch", kd_pitch, 20.0);
	n.param("kp_yaw", kp_yaw, 18.0);	
	n.param("kd_yaw", kd_yaw, 18.0);	
	n.param("kp_x", kp_x, 5.0);
	n.param("kp_y", kp_y, 5.0);	
	n.param("kp_z", kp_z, 25.0);
	n.param("kd_x", kd_x, 3.0);
	n.param("kd_y", kd_y, 3.0);	
	n.param("kd_z", kd_z, 10.0);		
	n.param("ki_x", ki_x, 0.0);
	n.param("ki_y", ki_y, 0.0);	
	n.param("ki_z", ki_z, 0.0);		
	n.param("d1"  , d1	, 1301.2);		
	n.param("d2"  , d2	, 39.7);		
	n.param("kF"  , kF	, 1.5468e-07);		
	n.param("max_tilt"		, max_tilt, deg2rad(10.0));
	n.param("max_thrust"	, max_thrust, 3000.0); // In grams
	n.param("max_acc"		, max_acc, 1.0);
	n.param("refresh_rate"	, refresh_rate, 100.0);
	n.param("debug_mode"	, debug_mode, true);
	n.param("max_dthrust"	, max_dthrust, 200.0); // in grams/sec
	n.param("max_dyaw"		, max_dyaw, deg2rad(20.0));
		
	ROS_INFO(" --------------- PID CONT ---------------");
	ROS_INFO("[mass] ---------------- : [%.3lf]", mass);
	ROS_INFO("[kp_roll , kd_roll ] -- : [%.3lf, %.3lf]", kp_roll , kd_roll);
	ROS_INFO("[kp_pitch, kd_pitch] -- : [%.3lf, %.3lf]", kp_pitch, kd_pitch);
	ROS_INFO("[kp_yaw  , kd_yaw] ---- : [%.3lf, %.3lf]", kp_yaw  , kd_yaw);
	ROS_INFO("[kp_x, kp_y, kp_z] ---- : [%.3lf, %.3lf, %.3lf]", kp_x, kp_y, kp_z);
	ROS_INFO("[ki_x, ki_y, ki_z] ---- : [%.3lf, %.3lf, %.3lf]", ki_x, ki_y, ki_z);
	ROS_INFO("[kd_x, kd_y, kd_z] ---- : [%.3lf, %.3lf, %.3lf]", kd_x, kd_y, kd_z);
	ROS_INFO("[d1, d2] -------------- : [%.3lf, %.3lf]", d1, d2);
	ROS_INFO("[kF] ------------------ : [%lf]", kF);
	ROS_INFO("[max_tilt, max_thrust]  : [%.3lf, %.3lf]", max_tilt, max_thrust);
	ROS_INFO("[max_acc] --------------: [%.3lf]", max_acc);
	ROS_INFO("[refresh_rate] -------- : [%.3lf]", refresh_rate);
	ROS_INFO("[debug_mode] ---------- : [%s]", debug_mode ? "TRUE" : "FALSE");
	ROS_INFO("[max_dthrust] --------- : [%.3lf]", max_dthrust);
	ROS_INFO("[max_dyaw] ------------ : [%.3lf]", max_dyaw);
	ROS_INFO(" ---------------------------------------------");
	return 0;
}

int setup_messaging_interface(ros::NodeHandle &n)
{
	heading_subs     = n.subscribe("heading"    , 10, heading_callback    , ros::TransportHints().tcpNoDelay());
	cont_params_subs = n.subscribe("cont_params", 10, cont_params_callback, ros::TransportHints().tcpNoDelay());
	pdcmd_publ       = n.advertise<com_msgs::PDCmd>("pdcmd", 10);
	
	ROS_INFO("Subscribed to [%s] for heading messages", "heading");
	ROS_INFO("Subscribed to [%s] for PID parameter messages", "cont_params");
	ROS_INFO("Publishing PD commands to [%s]", "pdmcd");

	return 0;
}

int loop(const ros::NodeHandle &n)
{
	ros::Rate r(refresh_rate);

	prev_time = ros::Time::now();
	curr_time = ros::Time::now();
	
	ros::spin();
	
	return 0;
}

void heading_callback(const cont_msgs::Heading &msg)
{	
	heading_msg = msg;
	calculate_control_parameters();
	pdcmd_publ.publish(pdcmd_msg);

	if(debug_mode){
		ROS_INFO("PID CONT : Received cont_msgs::Heading");
	}
}

// ### Update parameters here
void cont_params_callback(const cont_msgs::ContParams &msg){
	if(debug_mode){
		ROS_INFO("PID CONT : Received cont_msgs:ContParams");
		// ### Print the new set of parameters
	}
}

int calculate_control_parameters()
{
	prev_time = curr_time;
	curr_time = ros::Time::now();
	dt = Dt(prev_time, curr_time);

	if(debug_mode){
		ROS_INFO("curr time : [%d, %d]", curr_time.sec, curr_time.nsec);
		ROS_INFO("prev time : [%d, %d]", prev_time.sec, prev_time.nsec);
		ROS_INFO("dt        : [%.5lf]" , dt);
	}

	static Vector3d int_err = Vector3d::Zero(3);
	static Vector4d quat;
	static Vector3d rpy;
	// Conver current orientation to Euler angles
	quat(0) = heading_msg.quat_from.w;
	quat(1) = heading_msg.quat_from.x;
	quat(2) = heading_msg.quat_from.y;
	quat(3) = heading_msg.quat_from.z;
    rpy = quat2rpy(quat);	
	float psi_from = rpy(2);
	// Conver goal orientation to Euler angles
	quat(0) = heading_msg.quat_to.w;
	quat(1) = heading_msg.quat_to.x;
	quat(2) = heading_msg.quat_to.y;
	quat(3) = heading_msg.quat_to.z;
    rpy = quat2rpy(quat);	
	float psi_to = rpy(2);

	// Constrain the maximum yaw speed.
	double dpsi = fix_angle(psi_to - psi_from) / dt;
	dpsi = saturate(dpsi, -max_dyaw, max_dyaw);
	psi_to = fix_angle(psi_from + dt * dpsi);

	// Initialize intermediate variables
	static Vector3d acc_to, vel_to  , pos_to, 
							vel_from, pos_from;
	static Vector3d acc_des, euler_des;
	pos_from(0) = heading_msg.pos_from.x;	
	pos_from(1) = heading_msg.pos_from.y;
	pos_from(2) = heading_msg.pos_from.z;
	vel_from(0) = heading_msg.vel_from.x;	
	vel_from(1) = heading_msg.vel_from.y;
	vel_from(2) = heading_msg.vel_from.z;

	pos_to(0) = heading_msg.pos_to.x;	
	pos_to(1) = heading_msg.pos_to.y;	
	pos_to(2) = heading_msg.pos_to.z;	
	vel_to(0) = heading_msg.vel_to.x;	
	vel_to(1) = heading_msg.vel_to.y;
	vel_to(2) = heading_msg.vel_to.z;	
	acc_to(0) = heading_msg.acc_to.x;	
	acc_to(1) = heading_msg.acc_to.y;	
	acc_to(2) = heading_msg.acc_to.z;	
	
	// ### I still do not use KI
	// Calculate the required acceleration and orientation
	acc_des(0) = acc_to(0) + kd_x * (vel_to(0) - vel_from(0)) + kp_x * (pos_to(0) - pos_from(0)) + ki_x * int_err(0);
	acc_des(1) = acc_to(1) + kd_y * (vel_to(1) - vel_from(1)) + kp_y * (pos_to(1) - pos_from(1)) + ki_y * int_err(1);
	acc_des(2) = acc_to(2) + kd_z * (vel_to(2) - vel_from(2)) + kp_z * (pos_to(2) - pos_from(2)) + ki_z * int_err(2);

	int_err += (pos_to - pos_from) * dt;

	// Limit the accelerations and rotational speeds
	if(acc_des.norm() > max_acc)
		acc_des *= max_acc / acc_des.norm();

	euler_des(0) = (1/g) * (acc_des(0) * sin(psi_from) - acc_des(1) * cos(psi_from));
	euler_des(1) = (1/g) * (acc_des(0) * cos(psi_from) + acc_des(1) * sin(psi_from));
	euler_des(2) = psi_to;
	// Saturate resultant inputs
	euler_des(0) = saturate(euler_des(0), -max_tilt, max_tilt);
	euler_des(1) = saturate(euler_des(1), -max_tilt, max_tilt);
	
	// Convert accelation to motor input (thrust := grams)
	// double thrust = (sqrt((mass*(acc_des(2) + g))/(4*kF)) - d1 ) / d2 ;
	double thrust = mass * (acc_des(2) + g);
	//if(acc_des(2) < 0) // cannot check 'thrust' due to sqrt(...)
	//	thrust = 1;
	thrust = thrust < 1 ? 1 : thrust;
	// Prevent sudden step-ups
	static double prev_thrust = 0.0;
	if((thrust - prev_thrust)/dt >= max_dthrust)
		thrust = prev_thrust + dt * max_dthrust;
	if(thrust > max_thrust)
		thrust = max_thrust;
	prev_thrust = thrust;

	// Pass the inputs to the driver side
	pdcmd_msg.header.stamp	  = ros::Time::now();
	pdcmd_msg.header.frame_id = "onboard";
	pdcmd_msg.header.seq++;
	pdcmd_msg.thrust	= thrust;
	pdcmd_msg.roll		= euler_des(0);
	pdcmd_msg.pitch		= euler_des(1);
	pdcmd_msg.yaw		= euler_des(2);
	pdcmd_msg.droll		= 0;
	pdcmd_msg.dpitch	= 0;
	pdcmd_msg.dyaw		= 0;
	pdcmd_msg.kp_roll 	= kp_roll;
	pdcmd_msg.kd_roll 	= kd_roll;
	pdcmd_msg.kp_pitch	= kp_pitch;
	pdcmd_msg.kd_pitch	= kd_pitch;
	pdcmd_msg.kp_yaw 	= kp_yaw;
	pdcmd_msg.kd_yaw 	= kd_yaw;

	if(debug_mode)
	{
		ROS_INFO("------------------- PID CONT -------------------");
		ROS_INFO(" thrust       = [%.3f]", thrust);
		ROS_INFO(" euler_des    = [%.3f, %.3f, %.3f]", euler_des(2), euler_des(1), euler_des(0));
		ROS_INFO(" acc_des      = [%.3f, %.3f, %.3f]",   acc_des(0),   acc_des(1),   acc_des(2));
		ROS_INFO(" pos_from     = [%.3f, %.3f, %.3f]",  pos_from(0),  pos_from(1),  pos_from(2));
		ROS_INFO(" vel_from     = [%.3f, %.3f, %.3f]",  vel_from(0),  vel_from(1),  vel_from(2));
		ROS_INFO(" pos_to       = [%.3f, %.3f, %.3f]",    pos_to(0),    pos_to(1),    pos_to(2));
		ROS_INFO(" vel_to       = [%.3f, %.3f, %.3f]",    vel_to(0),    vel_to(1),    vel_to(2));
		ROS_INFO(" acc_to       = [%.3f, %.3f, %.3f]",    acc_to(0),    acc_to(1),    acc_to(2));
		ROS_INFO(" kp_[r, p, y] = [%.3f, %.3f, %.3f]",      kp_roll,     kp_pitch,       kp_yaw);
		ROS_INFO(" kd_[r, p, y] = [%.3f, %.3f, %.3f]",      kd_roll,     kd_pitch,       kd_yaw);
		ROS_INFO(" kp_[x, y, z] = [%.3f, %.3f, %.3f]",         kp_x,         kp_y,         kp_z);
		ROS_INFO(" ki_[x, y, z] = [%.3f, %.3f, %.3f]",         ki_x,         ki_y,         ki_z);
		ROS_INFO(" kd_[x, y, z] = [%.3f, %.3f, %.3f]",         kd_x,         kd_y,         kd_z);
	}
	return 0;
}


#undef deg2rad
#undef Dt
