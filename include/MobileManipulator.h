#include "geometry_msgs/Accel.h"
#include "SerialLink.h"

class MobileManipulator
{
	public:
		// Properties
		geometry_msgs::Pose baseTF;			// Transform from vehicle origin to manipulator base
		//mobile_manipulator::Vehicle vehicle;		// Vehicle object
		//serial_link::Serial arm;			// Serial-Link object

		// Constructor(s)
		MobileManipulator();
		MobileManipulator(int Hz);

		// Get Functions
		sensor_msgs::JointState rmrc(const geometry_msgs::Pose &pos, 
					     const geometry_msgs::Twist &vel);


		// Set Functions
		void updateState(const geometry_msgs::Pose &pose,
				 const geometry_msgs::Twist &vel,
				 const geometry_msgs::Accel &accel,
				 const sensor_msgs::JointState &jointState);

	private:

		// Properties

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

}

void MobileManipulator::updateState(const geometry_msgs::Pose &pose,
				    const geometry_msgs::Twist &vel,
				    const geometry_msgs::Accel &accel,
				    const sensor_msgs::JointState &jointState)
{
	//this->vehicle.pose = pose;
	//this->vehicle.velocity = vel;
	//this->vehicle.acceleration = accel;
	//this->arm.updateState(jointState,multiplyPose(pose.pose,this->baseTF);
}

sensor_msgs::JointState MobileManipulator::rmrc(const geometry_msgs::Pose &pos, const geometry_msgs::Twist &vel)
{

}




