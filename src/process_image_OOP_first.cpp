#include "ros/ros.h"
#include "ball_chaser_OOP/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage{

	private:

	ros::ServiceClient client; // Define a global client that can request services
	ros::Subscriber sub; // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function

	public:
	
	// Constructor	
	ProcessImage(ros::NodeHandle *nh){

    		// Define a client service capable of requesting services from command_robot
    		client = nh->serviceClient<ball_chaser_OOP::DriveToTarget>("/ball_chaser_OOP/command_robot");

    		// Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    		sub = nh->subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback, this);
	}

	// This function calls the command_robot service to drive the robot in the specified direction
	void drive_robot(float lin_x, float ang_z)
	{

    		ROS_INFO_STREAM("Commanding robot");
    
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

}; // ProcessImage class


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle nh;

    // Create object
    ProcessImage pi = ProcessImage(&nh);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
