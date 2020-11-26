#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser_OOP/DriveToTarget.h"

class DriveBot{

	private:
		
	ros::NodeHandle* nh_; // public node handle
	ros::NodeHandle* pnh_; // private node handle (for accessing parameter values, not currently used)
	ros::ServiceServer service; // Service for receiving robot velocity commands
	ros::Publisher motor_command_publisher; // Publisher for motor commands

	public:

	// Constructor	
	//DriveBot(ros::NodeHandle *nh){
	DriveBot(ros::NodeHandle *nh, ros::NodeHandle *pnh) : nh_(nh), pnh_(pnh)
	{

		// Define a service server to drive the robot
		service = nh_->advertiseService("/ball_chaser_OOP/command_robot", &DriveBot::handle_drive_request, this);

		// Define publisher of message type geometry_msgs::Twist on the robot actuation topic
    		motor_command_publisher = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	}

	//Destructor
	~DriveBot(){ 

		ros::shutdown();
		ROS_INFO("Destructor called");
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
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Create DriveBot object
    //DriveBot db = DriveBot(&nh, &pnh);
    DriveBot db(&nh, &pnh);

    //Handle ROS communication events
    ros::spin();

    return 0;
}
