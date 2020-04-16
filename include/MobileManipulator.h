#include "Eigen/Dense"						// Required for matrix inversion
#include "geometry_msgs/Accel.h"
#include "mobile_manipulator/Control.h"				// Custom control message for both manipulator and vehicle
#include "mobile_manipulator/Vehicle.h"				// Custom vehicle message
#include "SerialLink.h"						// Custom serial-link class

class MobileManipulator
{
	public:
		// Properties
		geometry_msgs::Pose base_transform;		// Transform from vehicle origin to manipulator base

		mobile_manipulator::Vehicle vehicle;		// Vehicle object

		SerialLink arm;					// Serial-Link object

		// Constructor(s)
		MobileManipulator();

		// Get Functions
		mobile_manipulator::Control rmrc(const geometry_msgs::Pose &pos, 
					     	 geometry_msgs::Twist &vel);


		// Set Functions
		void updateState(const geometry_msgs::PoseStamped &input,
				 const geometry_msgs::Twist &vel,
				 const geometry_msgs::Accel &accel,
				 const sensor_msgs::JointState &jointState);

	private:

		// Properties
		Eigen::MatrixXd A;				// B*T
		Eigen::Matrix<double,6,6> B;			// Base velocity mapping to end-effector
		Eigen::MatrixXd J;				// Manipulator Jacobian
		Eigen::MatrixXd T;				// Thruster conversion matrix
		Eigen::MatrixXd M1, M3;				// For kinematics calcs
		Eigen::Matrix<double,6,6> M2;			// For kinematics calcs

		Eigen::MatrixXd Wq;				// Joint weighting matrix 
		Eigen::MatrixXd Wu;				// Thruster weighting matrix

		int m;						// No. of thrusters
		
		mobile_manipulator::Control control_msg;	// Control message to be returned

		// Get Functions

		// Set Functions

};								// Class definitions must end with a semicolon


MobileManipulator::MobileManipulator()
{
	// Step 1: Create the serial_link object

	// Step 2: Resize any dynamic memory arrays

	this->m = this->vehicle.thruster.size();		// No. of thrusters in this model

	this->control_msg.jointControl.velocity.resize(this->arm.n);
	this->control_msg.thrusterControl.resize(this->m);

	this->A.resize(6,this->m);				// A = B*T
	this->B.setIdentity();					// Maps vehicle velocity to end-effector velocity
	this->T.resize(6,this->m);				// Maps thruster control to body velocity

	//this->M1.resize(this->m + this->arm.n,6);		// For kinematics calcs
	this->M3.resize(this->m + this->arm.n,6);		// M3 = M1*M2

	this->Wq = 10*Eigen::MatrixXd::Identity(this->arm.n, this->arm.n); // Joint weighting matrix
	this->Wu = Eigen::MatrixXd::Identity(this->m, this->m);		   // Thruster weighting matrix

	// Step 3: Compute thruster transform matrix
	for(int i = 0; i < this->m; i++)
	{
		T(0,i) = vehicle.thruster[i].axis.x;
		T(1,i) = vehicle.thruster[i].axis.y;
		T(2,i) = vehicle.thruster[i].axis.z;
		
		T(3,i) = vehicle.thruster[i].axis.y*vehicle.thruster[i].position.z 
		       - vehicle.thruster[i].axis.z*vehicle.thruster[i].position.y;

		T(4,i) = vehicle.thruster[i].axis.z*vehicle.thruster[i].position.x
		       - vehicle.thruster[i].axis.x*vehicle.thruster[i].position.z;

		T(5,i) = vehicle.thruster[i].axis.x*vehicle.thruster[i].position.y
		       - vehicle.thruster[i].axis.y*vehicle.thruster[i].position.x;
	}
}

void MobileManipulator::updateState(const geometry_msgs::PoseStamped &input,
				    const geometry_msgs::Twist &vel,
				    const geometry_msgs::Accel &accel,
				    const sensor_msgs::JointState &jointState)
{
	this->vehicle.tf = input;							// Assign pose
	this->vehicle.vel = vel;							// Assign velocity
	this->vehicle.accel = accel;							// Assign acceleration
	this->arm.updateState(jointState,multiplyPose(input.pose,this->base_transform));	

	// Update skew-symmetric component of base velocity mapping
	double x = this->arm.FK.poses[this->arm.n].position.x - this->vehicle.tf.pose.position.x;
	double y = this->arm.FK.poses[this->arm.n].position.y - this->vehicle.tf.pose.position.y;
	double z = this->arm.FK.poses[this->arm.n].position.z - this->vehicle.tf.pose.position.z;
	this->B(0,4) = z;
	this->B(0,5) = -y;
	this->B(1,3) = -z;
	this->B(1,5) = x;
	this->B(2,3) = y;
	this->B(2,4) = -x;

	this->A = this->B*this->T;		// Converts thruster control to end-effector velocity
	
}

mobile_manipulator::Control MobileManipulator::rmrc(const geometry_msgs::Pose &pos,
						    geometry_msgs::Twist &vel)
{
	// Update the joint weight based on proximity to joint limits
	for(int i = 0; i < this->arm.n; i++)
	{
		this->Wq(i,i) = getJointWeight(this->arm.joint_state.position[i],
					       this->arm.joint_state.velocity[i],
					       this->arm.serial.joint[i]);
	}

	// Hopefully these assignments make calcs a little faster
	Eigen::MatrixXd invWqJt = Wq.inverse()*this->arm.J.transpose();
	Eigen::MatrixXd invWuAt = Wu.inverse()*this->A.transpose();	

	// First component of weighted pseudoinverse
	this->M1.block(0,0,this->arm.n,6) = invWqJt;
	this->M1.block(this->arm.n,0,this->m,6) = invWuAt;

	// Second component of weighted pseudoinverse
	double lambda = this->arm.getDamping(this->arm.J);		// Apply DLS if arm is near-singular	
	this->M2 = this->arm.J*invWqJt + this->A*invWuAt + lambda*Eigen::MatrixXd::Identity(6,6);
				

	this->M3 = M1*M2.inverse();					// This is the total pseudoinverse


	// Compute pose error and add feedback to velocity control
	geometry_msgs::Pose error = getPoseError(pos, this->arm.FK.poses[this->arm.n]);
	vel.linear.x += 0.9*error.position.x;
	vel.linear.y += 0.9*error.position.y;
	vel.linear.z += 0.9*error.position.z;
	vel.angular.x += 0.9*error.orientation.x;
	vel.angular.y += 0.9*error.orientation.y;
	vel.angular.z += 0.9*error.orientation.z;

	// Required joint control
	for(int i = 0; i < this->arm.n; i++)
	{
		control_msg.jointControl.velocity[i] = M3(i,0)*vel.linear.x
						     + M3(i,1)*vel.linear.y
						     + M3(i,2)*vel.linear.z
						     + M3(i,3)*vel.angular.x
						     + M3(i,4)*vel.angular.y
						     + M3(i,5)*vel.angular.z;
	}

	// Required thruster control
	for(int i = this->arm.n; i < this->m + this->arm.n; i++)
	{
		control_msg.thrusterControl[i] 	= M3(i,0)*vel.linear.x
						+ M3(i,1)*vel.linear.y
						+ M3(i,2)*vel.linear.z
						+ M3(i,3)*vel.angular.x
						+ M3(i,4)*vel.angular.y
						+ M3(i,5)*vel.angular.z;
	}

	return control_msg;
}




