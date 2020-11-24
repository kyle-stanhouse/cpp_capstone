#include "ros/ros.h"
#include "ball_chaser_OOP/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include "geometry_msgs/Twist.h"

class BallChaser{

	private:

	ros::ServiceClient client; // Define a service client for commanding robot velocity
	ros::Subscriber sub; // Subscriber for image topic
	ros::Publisher motor_command_publisher; // Publisher for motor commands
    	ros::ServiceServer service; // Service for receiving robot velocity commands

	public:
	
	// Constructor	
	BallChaser(ros::NodeHandle *nh, ros::NodeHandle *nh2){

		// Define a service server to drive the robot
		service = nh2->advertiseService("/ball_chaser_OOP/command_robot", &BallChaser::handle_drive_request, this);

		// Define publisher of message type geometry_msgs::Twist on the robot actuation topic
    		motor_command_publisher = nh2->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    		// Define a client service for requesting services from command_robot
    		client = nh->serviceClient<ball_chaser_OOP::DriveToTarget>("/ball_chaser_OOP/command_robot");

    		// Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    		sub = nh->subscribe("/camera/rgb/image_raw", 10, &BallChaser::process_image_callback, this);
	}

	// This function calls the command_robot service to drive the robot in the specified direction
	void drive_robot(float lin_x, float ang_z)
	{

    		ROS_INFO_STREAM("Calling command_robot service");
    
    		ball_chaser_OOP::DriveToTarget srv;
    		srv.request.linear_x = lin_x;
    		srv.request.angular_z = ang_z;

    		// Check for failed service
    		if (!client.call(srv))
        		ROS_ERROR("Failed to call service DriveToTarget");
	}

	// This callback function continuously executes and reads the image data
	void process_image_callback(const sensor_msgs::Image img)
	{

	    int white_pixel = 255;
	    int white_pixel_idx;
	    bool found_white_ball = false;

	    // Loop through each pixel in the image and check if there's a bright white one
	    // Then, identify if this pixel falls in the left, mid, or right side of the image
	    // Depending on the white ball position, call the drive_bot function and pass velocities to it
	    // Request a stop when there's no white ball seen by the camera

	    for (int i = 0; i < img.height * img.step; i+=3) {

		// find white pixel
		if ( (img.data[i] == white_pixel) && ( img.data[i+1] == white_pixel ) && ( img.data[i+2] == white_pixel ) ) {

		    ROS_INFO_STREAM("Found white ball!");
		    white_pixel_idx = i;
		    found_white_ball = true;
		}

		if (found_white_ball == true){break;}

	    } //end for

	    
	    if (found_white_ball == true){

		// Decide to drive robot left, forward, right, or stop
		if ( (white_pixel_idx % img.step) < fabs(img.step/3) ){

			ROS_INFO_STREAM("Driving left");
			//drive_robot(0.025*multx,0.1*multx); //drive left
			drive_robot(0.0,0.08); //drive left
		} 
		else if( ( (white_pixel_idx % img.step) >= fabs(img.step/3) ) && ( white_pixel_idx % img.step < fabs(img.step * 2/3) ) ){

			ROS_INFO_STREAM("Driving forward");	    	
			drive_robot(0.1,0.0); //drive forward
		} 
		else if( (white_pixel_idx % img.step) >= fabs(img.step*2/3) ){

			ROS_INFO_STREAM("Driving right");		
			drive_robot(0.0,-0.08); //drive right
		} 
		else{

		     ROS_ERROR("Couldn't find drive direction");
		     drive_robot(0.0,0.0); //stop robot 
		}

		// reset flag
		found_white_ball = false;

	    } //end if

	    else if (found_white_ball == false){

		ROS_INFO_STREAM("Couldn't find white ball!");
		drive_robot(0.0,0.0); //stop robot
		} 	

	  } // end process_image_callback

	// Callback for command_robot service
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


}; // BallChaser class end


int main(int argc, char** argv)
{

    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle nh_db;

    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle nh_pi;

    // Create object
    BallChaser bc = BallChaser(&nh_pi,&nh_db);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
