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
#include <array>
#include <fstream>

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
VectorXd generateTrajectory(Vector3d hitVelocity);
Vector3d trajectoryPosition(VectorXd coeffs, double t, double theta, double psi);
void safetyChecks(VectorXd q,VectorXd dq,VectorXd tau, int dof);


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

//soft safety values
const std::array<double, 7> joint_position_max = {2.7, 1.6, 2.7, -0.2, 2.7, 3.6, 2.7};
const std::array<double, 7> joint_position_min = {-2.7, -1.6, -2.7, -3.0, -2.7, 0.2, -2.7};
const std::array<double, 7> joint_velocity_limits = {2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5};
const std::array<double, 7> joint_torques_limits = {85, 85, 85, 85, 10, 10, 10};


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
	const Vector3d control_point = Vector3d(0,0,0.2035);
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

		// open csv
	std::ofstream myfile;
    myfile.open ("trajectory.csv");

    //Velocity to hit the coin at. Change later to integrate with shot planner
    Vector3d hitVelocity; hitVelocity << 5.0*0.0254, 0.0, 0.0;

    //Initialize and retreive template trajectory
	VectorXd poly_trajectory_coeffs = VectorXd::Zero(16); 
	poly_trajectory_coeffs = generateTrajectory(hitVelocity);
	//cout<<poly_trajectory_coeffs(0)<<','<<poly_trajectory_coeffs(1)<<','<<poly_trajectory_coeffs(2)<<','<<poly_trajectory_coeffs(3)<<','<<poly_trajectory_coeffs(4)<<','<<poly_trajectory_coeffs(5)<<','<<poly_trajectory_coeffs(6)<<','<<poly_trajectory_coeffs(7)<<','<<poly_trajectory_coeffs(8)<<','<<poly_trajectory_coeffs(9)<<','<<poly_trajectory_coeffs(10)<<','<<poly_trajectory_coeffs(11)<<','<<poly_trajectory_coeffs(12)<<','<<poly_trajectory_coeffs(13)<<','<<poly_trajectory_coeffs(14)<<','<<poly_trajectory_coeffs(15)<<endl;
	Vector3d end_position = Vector3d::Zero(3);
	end_position = trajectoryPosition(poly_trajectory_coeffs, 9, -0.0*M_PI/180.0, 90.0*M_PI/180.0);

	//cout<<poly_trajectory_coeffs<<endl;



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
				// if(robotReachedGoal(x,calculatePointInTrajectory(100),xdot,xddot,omega,alpha))	//100 is arbitrarily large, represents last point in traj
				// {	
				// 	printf("Going into WAIT_MODE..\n");
				// 	mode = WAIT_MODE;
				// 	redis_client.set(MODE_CHANGE_KEY,"wait");
				// 	state = JOINT_CONTROLLER;
				// 	joint_task->_desired_position = q_init_desired;
				// }



				if(robotReachedGoal(x,end_position,xdot,xddot,omega,alpha))	//100 is arbitrarily large, represents last point in traj
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

				//posori_task->_desired_position = calculatePointInTrajectory(t);

				//Keep orientation
				posori_task->_desired_orientation = calculateRotationInTrajectory(t);
				//printf("%f, %f, %f\n",posori_task->_desired_position(0),posori_task->_desired_position(1),posori_task->_desired_position(2));

				// myfile<<controller_counter<<endl;
				//myfile <<controller_counter<<","<<poly_trajectory(controller_counter, 0)<<","<<poly_trajectory(controller_counter, 1)<<","<<poly_trajectory(controller_counter, 2)<<endl;
				// Vector3d position = Vector3d::Zero(3);
				// position = trajectoryPosition(poly_trajectory_coeffs, t, 45, M_PI/2);
				// myfile<<t<<","<<x(0)<<","<<x(1)<<","<<x(2)<<endl;
				// cout<<"x = "<<x(0)<<" y = "<<x(1)<<" z = "<<x(2)<<endl;
				//cout<<position(0)<<","<<position(1)<<","<<position(2)<<endl;

				//Calculate positions based on trajecotry, inputs starting position angle (theta) and hitting angle (psi)
				posori_task->_desired_position = trajectoryPosition(poly_trajectory_coeffs, t, 0*M_PI/180.0, 90.0*M_PI/180.0);

				// compute torques
				posori_task->computeTorques(posori_task_torques);
				joint_task->computeTorques(joint_task_torques);

				command_torques = posori_task_torques + joint_task_torques;
			}

			// send to redis

			safetyChecks(robot->_q,robot->_dq,command_torques, dof);

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
    myfile.close();

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
	// diameter of board is 20.125 in, convert to m:
	double r=20.125/2*0.0254; 
	double t_start = 0;
	double t_1 = 5;
	double t_2 = 10;

	double x_offset = 0.7; //need to calibrate
	double y_offset = 0; //need to calibrate
	Vector3d xh; xh << 0.32,-0.35,0.5;	//calibrate this
	// Vector3d xc; xc << 0.5,0.35,0.5;
	Vector3d xc; xc << r*sin(-M_PI/4)+x_offset, r*cos(-M_PI/4)+y_offset, 0.5;	//calibrate this
	Vector3d xcd; xcd << r*sin(-3*M_PI/4)+x_offset, r*cos(-3*M_PI/4)+y_offset, 0.5; //calculate this - get from redis

	Vector3d x; 

	if(inRange(t,t_start,t_1))
	{
		//set positions according to analytical functions
		x =  xh + (xc  -  xh )*(t-0)/(5);
	}
	else if (inRange(t,t_1,t_2))
	{

		// x = xc;
		// Move cue coin from home to desired position
		double x0 = xc(0);
		double y0 = xc(1);
		double xf = xcd(0);
		double yf = xcd(1);

		double t0 = atan2(x0-x_offset,y0-y_offset);
		double tf = atan2(xf-x_offset,yf-y_offset);

		double old_range = t_2-t_1;
		double new_range = tf - t0;
		double new_t = (((t-t_1)*(new_range))/old_range) + t0;

		x << r*sin(new_t)+x_offset, r*cos(new_t)+y_offset, xc(2);
	}
	else
	{	
		x = xcd;

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
	Matrix3d home_orientation;

	home_orientation << 1,0,0,
	 					0,1,0,
	 					0,0,-1;

	if(inRange(t,0,5))
	{
	 rot =home_orientation;
	 }
	 else if(inRange(t,5,10))  //final orientation
	 {
	 	rot = home_orientation;
		// rot = AngleAxisd(M_PI/4, Vector3d::UnitY())*home_orientation;
	 }
	 else{
		rot = home_orientation;
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
Vector C - position at the arc (0, 0) before transfomations
Vector D - ending position

*/

VectorXd generateTrajectory(Vector3d hitVelocity){

	//values need to be measured
	// radius of the arc
	double r=13.1475/2*0.0254; 
	double arc_length = 10.096*0.0254;

	//values below need to be tuned
	double end_time = 9.0; //in seconds
	double t1 = 0.0;
	double t2 = 3.0;
	double t3 = 6.0;
	double t4 = end_time;

	Vector3d A; A<<-5.5*0.0254, 5.0*0.0254, 0.0;
	Vector3d B; B<<-3.0515*0.0254, 3.0*0.0254, 0.0;
	Vector3d C; C<< 0*0.0254, 0.0, 0.0;
	Vector3d D; D<< 3.0*0.0254, 3.0*0.0254, 0.0;

	//velocity right above the lip
	Vector3d B_velocity; B_velocity<< 4.0*0.0254, -1.0*0.0254, 0.0;


	MatrixXd Eqns = MatrixXd::Zero(16, 16);
	VectorXd values = VectorXd::Zero(1, 16);
	VectorXd coeffs;


	Eqns<<1, 0, t1, 0, pow(t1, 2), 0, pow(t1, 3), 0, pow(t1, 4), 0, pow(t1, 5), 0, pow(t1, 6), 0, pow(t1, 7), 0,
      	  0, 1, 0, t1, 0, pow(t1, 2), 0, pow(t1, 3), 0, pow(t1, 4), 0, pow(t1, 5), 0, pow(t1, 6), 0, pow(t1, 7),
      	  0, 0, 1, 0, 2*t1, 0, 3*pow(t1, 2), 0, 4*pow(t1, 3), 0, 5*pow(t1, 4), 0, 6*pow(t1, 5), 0, 7*pow(t1, 6), 0,
      	  0,  0, 0, 1, 0, 2*t1, 0, 3*pow(t1, 2), 0, 4*pow(t1, 3), 0, 5*pow(t1, 4), 0, 6*pow(t1, 5), 0, 7*pow(t1, 6),
          1, 0, t2, 0, pow(t2, 2), 0, pow(t2, 3), 0, pow(t2, 4), 0, pow(t2, 5), 0, pow(t2, 6), 0, pow(t2,7), 0,
          0, 1, 0, t2, 0, pow(t2, 2), 0, pow(t2, 3), 0, pow(t2, 4), 0, pow(t2, 5), 0, pow(t2, 6), 0, pow(t2,7),
          0, 0, 1, 0, 2*t2, 0, 3*pow(t2, 2), 0, 4*pow(t2, 3), 0, 5*pow(t2, 4), 0, 6*pow(t2, 5), 0, 7*pow(t2, 6), 0,
          0, 0, 0, 1, 0, 2*t2, 0, 3*pow(t2, 2), 0, 4*pow(t2, 3), 0, 5*pow(t2, 4), 0, 6*pow(t2, 5), 0, 7*pow(t2, 6),
          1, 0, t3, 0, pow(t3, 2), 0, pow(t3, 3), 0, pow(t3, 4), 0, pow(t3, 5), 0, pow(t3, 6), 0, pow(t3, 7), 0,
          0, 1, 0, t3, 0, pow(t3, 2), 0, pow(t3, 3), 0, pow(t3, 4), 0, pow(t3, 5), 0, pow(t3, 6), 0, pow(t3, 7),
          0, 0, 1, 0, 2*t3, 0, 3*pow(t3, 2), 0, 4*pow(t3, 3), 0, 5*pow(t3, 4), 0, 6*pow(t3, 5), 0, 7*pow(t3, 6), 0,
          0, 0, 0, 1, 0, 2*t3, 0, 3*pow(t3, 2), 0, 4*pow(t3, 3), 0, 5*pow(t3, 4), 0, 6*pow(t3, 5), 0, 7*pow(t3, 6),
          1, 0, t4, 0, pow(t4, 2), 0, pow(t4, 3), 0, pow(t4, 4), 0, pow(t4, 5), 0, pow(t4, 6), 0, pow(t4, 7), 0,
          0, 1, 0, t4, 0, pow(t4, 2), 0, pow(t4, 3), 0, pow(t4, 4), 0, pow(t4, 5), 0, pow(t4, 6), 0, pow(t4, 7),
          0, 0, 1, 0, 2*t4, 0, 3*pow(t4, 2), 0, 4*pow(t4, 3), 0, 5*pow(t4, 4), 0, 6*pow(t4, 5), 0, 7*pow(t4, 6), 0,
          0, 0, 0, 1, 0, 2*t4, 0, 3*pow(t4, 2), 0, 4*pow(t4, 3), 0, 5*pow(t4, 4), 0, 6*pow(t4, 5), 0, 7*pow(t4, 6);

    values << A(0), A(1), 0, 0, B(0), B(1), B_velocity(0), B_velocity(1), C(0), C(1), hitVelocity(0), hitVelocity(1), D(0), D(1), 0, 0;

    //values<<A(0), A(1), 0;
    //values = values.transpose();
    //coeffs = Eqns.fullPivLu().solve(values);

    //Different methods of solving result in different coeffs values. Current method is the closest to matlab values.
    coeffs = Eqns.inverse()*values;
    cout<<coeffs(0)<<','<<coeffs(1)<<','<<coeffs(2)<<','<<coeffs(3)<<','<<coeffs(4)<<','<<coeffs(5)<<','<<coeffs(6)<<','<<coeffs(7)<<','<<coeffs(8)<<','<<coeffs(9)<<','<<coeffs(10)<<','<<coeffs(11)<<','<<coeffs(12)<<','<<coeffs(13)<<','<<coeffs(14)<<','<<coeffs(15)<<endl;

    return coeffs;
}

/*Takes template trajectory, and calculation positions of needed trajectory with information from shot planner. 
theta is the angle along the arc we must hit the cue coin. Psi is the direction we hit the cue coin. t is time step*/
Vector3d trajectoryPosition(VectorXd coeffs, double t, double theta, double psi)
{
	double r=13.1475/2*0.0254; 
	double arc_length = 10.096*0.0254;
	double x = 0.0; double y; double z;
    double dt = 0.001;
    double endtime = 9.0;
    if (t>= 0 and t < endtime){
    	y = coeffs(0) + coeffs(2)*t + coeffs(4)*(pow(t, 2)) + coeffs(6)*(pow(t, 3)) + coeffs(8)*(pow(t, 4)) + coeffs(10)*(pow(t, 5)) + coeffs(12)*(pow(t, 6)) + coeffs(14)*(pow(t, 7));
		z = coeffs(1) + coeffs(3)*t + coeffs(5)*(pow(t, 2)) + coeffs(7)*(pow(t, 3)) + coeffs(9)*(pow(t, 4)) + coeffs(11)*(pow(t, 5)) + coeffs(13)*(pow(t, 6)) + coeffs(15)*(pow(t, 7))+0.012192;
	}else{
		y = 0.0;
		z = 0.012192;
	}

	//transformation matrix to create desired trajectory.
    MatrixXd transformation = MatrixXd::Zero(4, 4); 
    transformation << cos(- M_PI/2 + psi), -sin(- M_PI/2+ psi), 0, (arc_length*sin(theta)),
     			 sin(- M_PI/2 + psi), cos(- M_PI/2 +psi), 0, (-arc_length*cos(theta)),
     			 0, 0, 1, 0,
     			 0, 0, 0, 1;

    //turns board frame to robot frame
    MatrixXd transformation_b_to_r = MatrixXd::Zero(4, 4);
    transformation_b_to_r <<  0, 1, 0, .6,
    						 -1, 0, 0,  0,
    						  0, 0, 1, .3,
    						  0, 0, 0,  1;
    cout<<transformation_b_to_r<<endl;

    //cout<<"x="<<x<<" "<<"y="<<y<<" "<<"z="<<z<<endl;
    Vector3d trajectory = Vector3d::Zero(3);
    VectorXd trajectory_augmented = VectorXd::Zero(4); 
    trajectory_augmented<< x, y, z, 1;
    //trajectory_augmented<< 0, 13.0*0.0254, 0.35*0.0254+0.3, 1;
    //trajectory_augmented<< (arc_length*sin(theta)), (-arc_length*cos(theta)), 0, 1;
    cout<<"before transformation: "<<trajectory_augmented(0)<<", "<<trajectory_augmented(1)<<", "<<trajectory_augmented(2)<<endl;
    VectorXd trajectory_augmented_transformed = VectorXd::Zero(4);
    trajectory_augmented_transformed = transformation_b_to_r * (transformation * trajectory_augmented);
    //trajectory_augmented_transformed = transformation_b_to_r * trajectory_augmented;

    // cout<<"xta="<<trajectory_augmented(0)<<" "<<"yta="<<trajectory_augmented(1)<<" "<<"zta="<<trajectory_augmented(2)<<endl;
    trajectory = trajectory_augmented_transformed.leftCols(3);
    cout<<"xt="<<trajectory(0)<<" "<<"yt="<<trajectory(1)<<" "<<"zt="<<trajectory(2)<<endl;
    cout<<"t="<<t<<endl;

    return trajectory;
}








//soft limit safetycheck as per the driver
void safetyChecks(VectorXd q,VectorXd dq,VectorXd tau, int dof)
{
	for(int i = 0; i < dof; i++)
	{
		if(q[i]>joint_position_max[i]) cout << "------!! VIOLATED MAX JOINT POSITION SOFT LIMIT !!-------" << endl; 
		if(q[i]<joint_position_min[i]) cout << "------!! VIOLATED MIN JOINT POSITION SOFT LIMIT !!-------" << endl; 
		if(abs(dq[i])>joint_velocity_limits[i]) cout << "------!! VIOLATED MAX JOINT VELOCITY SOFT LIMIT !!-------" << endl; 
		if(abs(tau[i])>joint_torques_limits[i]) cout << "------!! VIOLATED MAX JOINT TORQUE SOFT LIMIT !!-------" << endl; 

	}
}

