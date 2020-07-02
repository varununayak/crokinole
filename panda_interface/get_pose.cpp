"""
Prints out the pose and joint configuration of the robot in the world frame,
used for calibration purposes
No control input is applied

@authors: Varun Nayak
"""

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>
#include <array>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;

void safetyChecks(VectorXd q,int dof);
// redis keys:
// - read:


const std::array<double, 7> joint_position_max = {2.7, 1.6, 2.7, -0.2, 2.7, 3.6, 2.7};
const std::array<double, 7> joint_position_min = {-2.7, -1.6, -2.7, -3.0, -2.7, 0.2, -2.7};
const std::array<double, 7> joint_velocity_limits = {2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5};
const std::array<double, 7> joint_torques_limits = {85, 85, 85, 85, 10, 10, 10};


// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

double ee_length = 0.253;


int main() {

	
	JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
	
	//soft safety values
Eigen::VectorXd jmax;
 jmax.resize(7);
jmax << 2.7, 1.6, 2.7, -0.2, 2.7, 3.6, 2.7;
Eigen::VectorXd jmin; jmin.resize(7);
jmin << -2.7, -1.6, -2.7, -3.0, -2.7, 0.2, -2.7;

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();	

	// pose task
	const string control_link = "link7";
	const Vector3d control_point =Vector3d(-ee_length*sin(M_PI/4.0),ee_length*cos(M_PI/4.0),0.1070+0.0635+0.0065);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	Vector3d x;//quantity to store current task space position
	Vector3d xdot;//quantiy to store current task space velocity]
	Matrix3d R; //rotation matrix
	
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(5); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		robot->updateKinematics();
		robot->position(x,control_link,control_point); //position of end effector
		robot->linearVelocity(xdot,control_link,control_point); //velocity of end effector 
		robot->rotation(R,control_link);

		cout << setprecision(4) << fixed << "\n\n---------------------\n Position of EE = \n" << x.transpose() << "\n"<< endl;
		cout << setprecision(2) << fixed << "JOINT ANGLES: \n" << jmin.transpose() << endl << robot->_q.transpose() << endl << jmax.transpose() << endl << "\n";
		safetyChecks(robot->_q, dof);
		cout << setprecision(4) << fixed << "\n Rotation Matrix = \n" << R << "\n---------------------\n" << endl;


	}

	double end_time = timer.elapsedTime();
   

	return 0;
}

void safetyChecks(VectorXd q,int dof)
{
	for(int i = 0; i < dof; i++)
	{
		if(q[i]>joint_position_max[i]) cout << "------!! VIOLATED MAX JOINT POSITION SOFT LIMIT !!------- for joint " <<i+1<< endl; 
		if(q[i]<joint_position_min[i]) cout << "------!! VIOLATED MIN JOINT POSITION SOFT LIMIT !!------- for joint" <<i+1<< endl; 

	}
}
