#include "pid_cont.hh"

using namespace std;
using namespace Eigen;

cont_msgs::Heading    heading_msg;
com_msgs::PDCmd       pdcmd_msg;

ros::Subscriber       heading_subs;
ros::Subscriber       cont_params_subs;
ros::Publisher        pdcmd_publ;

PIDController pid_cont;

double refresh_rate;
bool   debug_mode;

int  process_inputs(const ros::NodeHandle &n);
int  setup_messaging_interface(ros::NodeHandle &n);
int  loop(const ros::NodeHandle &n);
void heading_callback(const cont_msgs::Heading &msg);
void cont_params_callback(const cont_msgs::ContParams &msg);

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
	cont_msgs::PIDParams params;
	// ### KHex accepts 'grams' as thrust whereas Asctec accepts
	// some number between [50-250]. In this node I vote for using
	// grams and let the communication nodes handle this detail.
	// I will keep the previous code commented in order not to lose
	// the conversion expression.
	params.g = 9.81;
	// These default values are for the Pelican robot
	// and g is for the Earth :)
	n.param("mass", params.mass, 1.327);
	n.param("kp_roll", params.kp_roll, 258.0);
	n.param("kd_roll", params.kd_roll, 20.0);
	n.param("kp_pitch", params.kp_pitch, 258.0);
	n.param("kd_pitch", params.kd_pitch, 20.0);
	n.param("kp_yaw", params.kp_yaw, 18.0);	
	n.param("kd_yaw", params.kd_yaw, 18.0);	
	n.param("kp_x", params.kp_x, 5.0);
	n.param("kp_y", params.kp_y, 5.0);	
	n.param("kp_z", params.kp_z, 25.0);
	n.param("kd_x", params.kd_x, 3.0);
	n.param("kd_y", params.kd_y, 3.0);	
	n.param("kd_z", params.kd_z, 10.0);		
	n.param("ki_x", params.ki_x, 0.0);
	n.param("ki_y", params.ki_y, 0.0);	
	n.param("ki_z", params.ki_z, 0.0);		
	n.param("max_tilt"		, params.max_tilt, DEG2RAD(10.0));
	n.param("max_thrust"	, params.max_thrust, 3000.0); // In grams
	n.param("max_acc"		, params.max_acc, 1.0);
	n.param("max_dthrust"	, params.max_dthrust, 200.0); // in grams/sec
	n.param("max_dyaw"		, params.max_dyaw, DEG2RAD(20.0));
	n.param("refresh_rate"	, refresh_rate, 100.0);
	n.param("debug_mode"	, debug_mode, true);


	ROS_INFO(" --------------- PID CONT ---------------");
	ROS_INFO("[mass] ---------------- : [%.3lf]", params.mass);
	ROS_INFO("[kp_roll , kd_roll ] -- : [%.3lf, %.3lf]", params.kp_roll , params.kd_roll);
	ROS_INFO("[kp_pitch, kd_pitch] -- : [%.3lf, %.3lf]", params.kp_pitch, params.kd_pitch);
	ROS_INFO("[kp_yaw  , kd_yaw] ---- : [%.3lf, %.3lf]", params.kp_yaw  , params.kd_yaw);
	ROS_INFO("[kp_x, kp_y, kp_z] ---- : [%.3lf, %.3lf, %.3lf]", params.kp_x, params.kp_y, params.kp_z);
	ROS_INFO("[ki_x, ki_y, ki_z] ---- : [%.3lf, %.3lf, %.3lf]", params.ki_x, params.ki_y, params.ki_z);
	ROS_INFO("[kd_x, kd_y, kd_z] ---- : [%.3lf, %.3lf, %.3lf]", params.kd_x, params.kd_y, params.kd_z);
	ROS_INFO("[max_tilt, max_thrust]  : [%.3lf, %.3lf]", params.max_tilt, params.max_thrust);
	ROS_INFO("[max_acc] --------------: [%.3lf]", params.max_acc);
	ROS_INFO("[refresh_rate] -------- : [%.3lf]", refresh_rate);
	ROS_INFO("[debug_mode] ---------- : [%s]", debug_mode ? "TRUE" : "FALSE");
	ROS_INFO("[max_dthrust] --------- : [%.3lf]", params.max_dthrust);
	ROS_INFO("[max_dyaw] ------------ : [%.3lf]", params.max_dyaw);
	ROS_INFO(" ---------------------------------------------");

	pid_cont.set_params(params);

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
	ros::spin();
	return 0;
}

void heading_callback(const cont_msgs::Heading &msg)
{	
	if(debug_mode){
		ROS_INFO("PID CONT : Received cont_msgs::Heading");
	}

	pdcmd_publ.publish(pid_cont.generate_command(msg));

}

// ### Update parameters here
void cont_params_callback(const cont_msgs::ContParams &msg){
	if(debug_mode){
		ROS_INFO("PID CONT : Received cont_msgs:ContParams");
	}

	pid_cont.set_params(msg.pid_params);
}

