#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser_OOP/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

class DriveBot{

	private:
		
	ros::ServiceServer service; // Service for receiving robot velocity commands
	ros::Publisher motor_command_publisher; // Publisher for motor commands

	public:

	// Constructor	
	DriveBot(ros::NodeHandle *nh){

		// Define a service server to drive the robot
		service = nh->advertiseService("/ball_chaser_OOP/command_robot", &DriveBot::handle_drive_request, this);

		// Define publisher of message type geometry_msgs::Twist on the robot actuation topic
    		motor_command_publisher = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	}

	// Callback function that executes whenever a drive_bot service is requested
	bool handle_drive_request(ball_chaser_OOP::DriveToTarget::Request& req,
	    ball_chaser_OOP::DriveToTarget::Response& res)
	{
		ROS_INFO("Drive to Target request received - linear x:%1.2f, angular z:%1.2f", (float)req.linear_x, (float)req.angular_z);

		// Create a motor_command object of type geometry_msgs::Twist
		geometry_msgs::Twist motor_command;

		// Set wheel velocities
		motor_command.linear.x = req.linear_x;
		motor_command.angular.z = req.angular_z;

		// Publish velocities to drive the robot
		motor_command_publisher.publish(motor_command);

	    	// Wait 3 seconds for arm to settle
	    	//ros::Duration(3).sleep();


	    	// Return a response message
	    	res.msg_feedback = "Linear velocity set: " + std::to_string(req.linear_x) + " , Angular Velocity set: " + std::to_string(req.angular_z);
	    	ROS_INFO_STREAM(res.msg_feedback);

	    	return true;

	} // handle_drive_request

}; // DriveBot class


int main(int argc, char** argv)
{
    // Initialize a ROS node and create ROS NodeHandle
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Create DriveBot object
    DriveBot db = DriveBot(&n);

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}
