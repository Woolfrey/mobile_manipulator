#include "Eigen/Dense"						// Required for matrix inversion
#include "geometry_msgs/Accel.h"
#include "mobile_manipulator/Vehicle.h"
#include "SerialLink.h"

class MobileManipulator
{
	public:
		// Properties
		geometry_msgs::Pose baseTF;			// Transform from vehicle origin to manipulator base
		mobile_manipulator::Vehicle vehicle;		// Vehicle object
		SerialLink arm;					// Serial-Link object

		// Constructor(s)
		MobileManipulator();
		MobileManipulator(int Hz);

		// Get Functions
		sensor_msgs::JointState rmrc(const geometry_msgs::Pose &pos, 
					     const geometry_msgs::Twist &vel);


		// Set Functions
		void updateState(const geometry_msgs::PoseStamped &input,
				 const geometry_msgs::Twist &vel,
				 const geometry_msgs::Accel &accel,
				 const sensor_msgs::JointState &jointState);

	private:

		// Properties
		Eigen::MatrixXd A;				// B*T
		Eigen::Matrix<double,6,6> B;			// Base velocity mapping to end-effector
		Eigen::MatrixXd T;				// Thruster conversion matrix

		

		// Get Functions

		// Set Functions

};								// Class definitions must end with a semicolon


MobileManipulator::MobileManipulator()
{
	ROS_INFO_STREAM("Default control frequency is 100Hz. Is this OK? (Y/n):");
	char input;
	std::cin >> input;
	if(input == 'Y') MobileManipulator(100);
}

MobileManipulator::MobileManipulator(int Hz)
{
	const int n = this->vehicle.thruster.size();		// No. of thrusters in this model


	this->A.resize(6,n);
	this->B.setIdentity();
	this->T.resize(6,n);
	

	// Compute thruster transform matrix
	for(int i = 0; i < n; i++)
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
	this->vehicle.tf = input;
	this->vehicle.vel = vel;
	this->vehicle.accel = accel;
	this->arm.updateState(jointState,multiplyPose(input.pose,this->baseTF));

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
	
}

sensor_msgs::JointState MobileManipulator::rmrc(const geometry_msgs::Pose &pos,
						const geometry_msgs::Twist &vel)
{

	
/*geometry_msgs::Twist relative;					// Relative velocity to be passed to the manipulator

	// End-effector position - vehicle position
	r.x = this->arm.FK[this->arm.n].position.x - this->vehicle.tf.pose.position.x;
	r.y = this->arm.FK[this->arm.n].position.y - this->vehicle.tf.pose.position.y;
	r.z = this->arm.FK[this->arm.n].position.z - this->vehicle.tf.pose.position.z;


	// Relative linear velocity
	relative.linear.x = vel.linear.x - this->vehicle.vel.linear.x
					 + this->vehicle.vel.angluar.z*r.y
					 - this->vehicle.vel.angular.y*r.x;

	relative.linear.y = vel.linear.y - this->vehicle.vel.linear.y
					 + this->vehicle.vel.angular.x*r.z
					 - this->vehicle.vel.angular.z*r.x;

	relative.linear.z = vel.linear.z - this->vehicle.vel.linear.z
					 + this->vehicle.vel.angular.y*r.x
					 - this->vehicle.vel.angular.x*r.y;

	// Relative angular velocity
	relative.angular.x = vel.angular.x - this->vehicle.vel.angular.x;
	relative.angular.y = vel.angular.y - this->vehicle.vel.angular.y;
	relative.angular.z = vel.angular.z - this->vehicle.vel.angular.z;
*/
}




