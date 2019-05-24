// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>
#include <cmath>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1

#define WAIT_MODE 0
#define EXECUTE_MODE 1

int mode = WAIT_MODE;

int state = JOINT_CONTROLLER;

//function prototypes
bool robotReachedGoal(VectorXd x,VectorXd x_desired, VectorXd xdot, VectorXd xddot, VectorXd omega, VectorXd alpha);
Vector3d calculatePointInTrajectory(double t);
bool inRange(double t, double lower, double upper);
Matrix3d calculateRotationInTrajectory(double t);


// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

//state
// std::string MODE_CHANGE_KEY = "modechange";
std::string MODE_CHANGE_KEY = "modechange";

unsigned long long controller_counter = 0;

// const bool flag_simulation = false;
const bool flag_simulation = true;

const bool inertia_regularization = true;

int main() {

	if(flag_simulation)
	{
		JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::actuators::fgc";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::sensors::torques";
		MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::sensors::model::robot_gravity";		
	}

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
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0,0,0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

#ifdef USING_OTG
	posori_task->_use_interpolation_flag = true;
#else
	posori_task->_use_velocity_saturation_flag = true;
#endif
	
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = true;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0;
	joint_task->_kv = 15.0;

	VectorXd q_init_desired = initial_q;
	q_init_desired << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;


	//initialize position and velocity in cartesian space
	Vector3d x;//quantity to store current task space position
	Vector3d xdot;//quantiy to store current task space velocity]
	Vector3d xddot; //linear acceleration
	Vector3d omega;
	Vector3d alpha;



	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		//update cartesian position of the robot from joint angles
		robot->position(x,control_link,control_point); //position of end effector
		robot->linearVelocity(xdot,control_link,control_point); //velocity of end effector 
		robot->linearAcceleration(xddot,control_link,control_point);
		robot->angularVelocity(omega,control_link);
		robot->angularAcceleration(alpha,control_link);

		//calculate current time;
		double dt = 0.001;
		double t = controller_counter*dt;

		if(mode == WAIT_MODE)
		{	
			
			if(redis_client.get(MODE_CHANGE_KEY) == "execute")
			{	
				mode = EXECUTE_MODE;
				printf("Going into EXECUTE_MODE\n");
			}

		}
		else if(mode == EXECUTE_MODE)
		{		

			// update model
			if(flag_simulation)
			{
				robot->updateModel();
			}
			else
			{
				robot->updateKinematics();
				robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
				if(inertia_regularization)
				{
					robot->_M(4,4) += 0.07;
					robot->_M(5,5) += 0.07;
					robot->_M(6,6) += 0.07;
				}
				robot->_M_inv = robot->_M.inverse();
			}

			if(state == JOINT_CONTROLLER)
			{
				// update task model and set hierarchy
				joint_task->_desired_position = q_init_desired;
				N_prec.setIdentity();
				joint_task->updateTaskModel(N_prec);
				joint_task->_kp = 250.0;

				// compute torques
				joint_task->computeTorques(joint_task_torques);
				
				command_torques = joint_task_torques;

				if( (robot->_q - q_init_desired).norm() < 0.15 )
				{	
					cout << "Reached JOINT Goal" << endl;
					posori_task->reInitializeTask();					
					posori_task->_desired_position = calculatePointInTrajectory(t);
					//posori_task->_desired_orientation = AngleAxisd(-M_PI/2, Vector3d::UnitX()) * AngleAxisd(0,  Vector3d::UnitY()) * AngleAxisd(M_PI/2, Vector3d::UnitZ()) * posori_task->_desired_orientation;
					posori_task->_desired_orientation = calculateRotationInTrajectory(t);
					joint_task->reInitializeTask();
					joint_task->_kp = 0;

					state = POSORI_CONTROLLER;
				}
			}

			else if(state == POSORI_CONTROLLER)
			{	
				//if the robot reaches the desired position and is at rest, come out of the loop
				if(robotReachedGoal(x,calculatePointInTrajectory(100),xdot,xddot,omega,alpha))	//100 is arbitrarily large, represents last point in traj
				{	
					printf("Going into WAIT_MODE..\n");
					mode = WAIT_MODE;
					redis_client.set(MODE_CHANGE_KEY,"wait");
					state = JOINT_CONTROLLER;
					joint_task->_desired_position = q_init_desired;
				}

				// update task model and set hierarchy
				N_prec.setIdentity();
				posori_task->updateTaskModel(N_prec);
				N_prec = posori_task->_N;
				joint_task->updateTaskModel(N_prec);

				posori_task->_desired_position = calculatePointInTrajectory(t);
				posori_task->_desired_orientation = calculateRotationInTrajectory(t);

				// compute torques
				posori_task->computeTorques(posori_task_torques);
				joint_task->computeTorques(joint_task_torques);

				command_torques = posori_task_torques + joint_task_torques;
			}

			// send to redis
			redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

			controller_counter++;
		}
	}

	command_torques.setZero();

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}


bool robotReachedGoal(VectorXd x,VectorXd x_desired, VectorXd xdot, VectorXd xddot, VectorXd omega, VectorXd alpha)
{
	double epsilon = 0.001;
	double error_norm = 100*xdot.norm() + 10*(x-x_desired).norm() + 1000*xddot.norm() + 1000*omega.norm() + 1000*alpha.norm();
	if(error_norm<epsilon)
	{	
		printf("Reached Goal \n");
		return true;
	} 
	return false;
}


/*
This function returns the desired point in the operational space
that the robot needs to track. The plan is to divide it into sections 
parametrized by 't'.

From calibration and shot planner, we need (all expressed in robot frame)
1) Home position (xh, yh, zh)
2) Position of cue coin (xc,yc,zc) (also pre-determined)
3) Desired position of cue coin (xcd, ycd, zcd) (get from  shot planner over redis when mode changes)
4) Backup and Flick Trajectory expressed in the robot frame (get required params from shot planner and tranform it)
*/
Vector3d calculatePointInTrajectory(double t)
{	
	Vector3d xh; xh << 0.32,-0.35,0.65;	//calibrate this
	Vector3d xc; xc << 0.5,0.35,0.60;	//calibrate this
	Vector3d xcd; //calculate this - get from redis

	// radius of the board in mm
	double r=20.125/2; 
	double t_start = 0;
	double t_1 = 5;

	Vector3d x; 

	if(inRange(t,t_start,t_1))
	{
		//set positions according to analytical functions
		// x =  xh + (xc  -  xh )*(t-0)/(5);

		// Move cue coin from home to desired position
		double x0 = xh(1);
		double y0 = xh(2);
		double xf = xc(1);
		double yf = xc(2);

		double t0 = atan2(y0,x0);
		double tf = atan2(yf,xf);

		double old_range = t_1-t_start;
		double new_range = tf - t0;
		double new_t = (((t-0)*new_range)/old_range) + t0;

		x << r*sin(new_t), r*cos(new_t), xh(3);
	}
	else
	{	
		

	}

	return x;
}


/*
From calibration and shot planner, we need:
1) Orientation in home position (point straight and flat maybe?)
2) Angle to which to turn to once we reach the cue coin position (get from shot planner over redis)
3) Angle to which to point to for the backup and shot (get from shot planner over redis)
*/
Matrix3d calculateRotationInTrajectory(double t)
{
	Matrix3d rot;

	if(inRange(t,0,5))
	{
	 rot << 1,0,0,
	 		0,0,-1,
	 		0,1,0;
	 }
	 else  //final orientation
	 {
	 	rot << 1,0,0,
	 		0,0,-1,
	 		0,1,0;
	 }

	 return rot;

}


//return true if t lies in between lower and upper limits
bool inRange(double t, double lower, double upper)
{
	return (  (t < upper) &&  (t >= lower) );
}


/*
Generates Trajectory in Operational space for End Effector to follow. Params given by shot planner
@param   theta(rad) - angle along starting arc where cue coin starts
@param.  psi(rad) - angle to hit cue coin such that it hits opponent coin
@param   hitVelocity - speed to hit cue coin
@param.  start_time - default to 0
@param   end_time - default to 9 (should be manually calibrated)

A) Vector3d pullBack<xpb, ypb, zpb> - location to start trajectory (needs to be calibrated manually)
B) Vector3d lip_position<xl, yl, zl> - Location right above lip of board - acts as intermediate point
C) Vector3d cuecoin_pos<xc, yc, zc> - location of cue coin, derived from theta
D) Vector3d followThrough<xft, yft, zft> - location of follow through, has velocity of 0

5) Vector3d lip_velocity<vxl, vyl, vzl> - velocity to pass above the lip (needs to be calibrated manually)

Vector A - pullback position
Vector B - position right above lip

*/

Matrix3d GenerateTrajectory(double theta, double psi, Vector3d hitVelocity)

	//values need to be measured
	// radius of the arc
	double r=20.125/2; 
	double arc_length = 10.0;

	//values below need to be tuned
	double end_time = 9.0; //in seconds
	double t1 = 0.0;
	double t2 = end_time/3;
	double t3 = 2*end_time/3;
	double t4 = end_time;

	Vector3d A; A<<0.0, -r - 1.0, 3.0;
	Vector3d B; B<<0.0, -r, 0.5;
	Vector3d C; C<<0.0, -arc_length, 0.0;
	Vector3d D; D<<0.0, -4*arc_length/5, 0.0;

	Vector3d B_velocity; B_velocity<< 0.0, 1.0, -1.0;

	MatrixXd Eqns(16, 16);
	VectorXd values;
	VectorXd coeffs;


	Eqns<<1, 0, t1, 0, t1^2, 0, t1^3, 0, t1^4, 0, t1^5, 0, t1^6, 0, t1^7, 0,
      	  0, 1, 0, t1, 0, t1^2, 0, t1^3, 0, t1^4, 0, t1^5, 0, t1^6, 0, t1^7,
      	  0, 0, 1, 0, 2*t1, 0, 3*t1^2, 0, 4*t1^3, 0, 5*t1^4, 0, 6*t1^5, 0, 7*t1^6, 0.
      	  0,  0 0 1 0 2*t1 0 3*t1^2 0 4*t1^3 0 5*t1^4 0 6*t1^5 0 7*t1^6;
          1 0 t2 0 t2^2 0 t2^3 0 t2^4 0 t2^5 0 t2^6 0 t2^7 0;
          0 1 0 t2 0 t2^2 0 t2^3 0 t2^4 0 t2^5 0 t2^6 0 t2^7;
          0 0 1 0 2*t2 0 3*t2^2 0 4*t2^3 0 5*t2^4 0 6*t2^5 0 7*t2^6 0;
          0 0 0 1 0 2*t2 0 3*t2^2 0 4*t2^3 0 5*t2^4 0 6*t2^5 0 7*t2^6;
          1 0 t3 0 t3^2 0 t3^3 0 t3^4 0 t3^5 0 t3^6 0 t3^7 0;
          0 1 0 t3 0 t3^2 0 t3^3 0 t3^4 0 t3^5 0 t3^6 0 t3^7;
          0 0 1 0 2*t3 0 3*t3^2 0 4*t3^3 0 5*t3^4 0 6*t3^5 0 7*t3^6 0;
          0 0 0 1 0 2*t3 0 3*t3^2 0 4*t3^3 0 5*t3^4 0 6*t3^5 0 7*t3^6;
          1 0 t4 0 t4^2 0 t4^3 0 t4^4 0 t4^5 0 t4^6 0 t4^7 0;
          0 1 0 t4 0 t4^2 0 t4^3 0 t4^4 0 t4^5 0 t4^6 0 t4^7;
          0 0 1 0 2*t4 0 3*t4^2 0 4*t4^3 0 5*t4^4 0 6*t4^5 0 7*t4^6 0;
          0 0 0 1 0 2*t4 0 3*t4^2 0 4*t4^3 0 5*t4^4 0 6*t4^5 0 7*t4^6;

    values<<A(1), A(2), 0, 0, B(1), B(2), B_velocity(1), B_velocity(2), C(1), C(2), hitVelocity(1), hitVelocity(2), D(1), D(2), 0, 0;
    coeffs = Eqns.colPivHouseholderQr().solve(values);

    double dt = 0.001;
    double t = 0.0;
    int rows = (int) end_time/dt;
    MatrixXd trajectory = MatrixXd::Zero(rows, 3);
    for(int i = 0; t < rows; i++) {
    	double x = 0.0; double y; double z;
        y = coeffs(1) + coeffs(3)*t + coeffs(5)*(t^2) + coeffs(7)*(t^3) + coeffs(9)*(t^4) + coeffs(11)*(t^5) + coeffs(13)*(t^6) + coeffs(15)*(t^7);
		z = coeffs(2) + coeffs(4)*t + coeffs(6)*(t^2) + coeffs(8)*(t^3) + coeffs(10)*(t^4) + coeffs(12)*(t^5) + coeffs(14)*(t^6) + coeffs(16)*(t^7);
		trajectory(i, 0) = x;
		trajectory(i, 1) = y;
		trajectory(i, 2) = z;
		t += dt;
    }

    MatrixXd Transform(4, 4) << cos(- pi/2 + psi) -sin(- pi/2+ psi) 0 (L*sin(theta));
     							sin(- pi/2 + psi) cos(- pi/2 +psi) 0 (-L*cos(theta));
     							0 0 1 0;
     							0 0 0 1];












