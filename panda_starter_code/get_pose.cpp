// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";


// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;


// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;


int main() {

	

	JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
	

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
	const Vector3d control_point = Vector3d(0,0,0.07);
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

		cout << "\n\n---------------------\n Position of EE = \n" << x.transpose() << endl;
		cout << "\n Rotation Matrix = \n" << R << "\n---------------------\n" << endl;


	}

	double end_time = timer.elapsedTime();
   

	return 0;
}
