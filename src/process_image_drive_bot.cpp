#include "ros/ros.h"
#include "ball_chaser_OOP/DriveToTarget.h"
#include <sensor_msgs/Image.h>

#include "geometry_msgs/Twist.h"

// Define a global client that can request services
ros::ServiceClient client;

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
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
}

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Commanding robot");
    
    ball_chaser::DriveToTarget srv;
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
int multx = 20;


    // TODO: Loop through each pixel in the image and check if there's a bright white one
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
		drive_robot(0.0,0.2); //drive left
	} 
	else if( ( (white_pixel_idx % img.step) >= fabs(img.step/3) ) && ( white_pixel_idx % img.step < fabs(img.step * 2/3) ) ){

		ROS_INFO_STREAM("Driving forward");	    	
		//drive_robot(0.025*multx,0.0); //drive forward
		drive_robot(6.0,0.0); //drive forward
	} 
	else if( (white_pixel_idx % img.step) >= fabs(img.step*2/3) ){

		ROS_INFO_STREAM("Driving right");		
		//drive_robot(0.025,-0.1); //drive right
		//drive_robot(0.025*multx,-.1*multx); //drive right
		drive_robot(0.0,-0.2); //drive right
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

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    // Create a ROS NodeHandle object
    ros::NodeHandle n1;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n1.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
