/**
* \file user_interface.cpp
* \brief client for getting the choice from the user and sending it to the server
* \author Parisi Daniele Martino
* \version 1.0
* \date 27/02/2022
*
* \details
*
* services:<BR>
* /user
*
* Description:
*
* The code implements the logic for getting the modality chosen by the user and sending it to the server
**/


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "final_assignment/User.h" 

//Initialize the client that send the request to the server
ros::ServiceClient client; ///< Global client for sending the modality chosen by the user

//Initialize a global variable for storing the choice pressed by the user
int choice; ///< Global variable for storing the modality chosen by the user
bool resp; ///< Global variable for storing the feedback from the server

/**
* \brief main function
* \param argc
* \param argv
*
* Function for getting from the user the chosen modality above the possible 3 and receiving the feedback
* 
**/

int main (int argc, char **argv)
{	
	//Initialize the node, setup the NodeHandle for handling the communication with the ROS system  
	ros::init(argc, argv, "user_interface");  
	ros::NodeHandle nh;
	
	//Declare the client subscribed to the 'user' service
	client = nh.serviceClient<final_assignment::User>("/user");
	
	//Declare my name of the User service
	final_assignment::User usr_srv;
	
	while(1)
	{	
		//Menu to tell the user which are the possible choices
		std::cout << "Menu: you can type: \n 1) for choosing a target position (x,y) which the robot has to reach\n 2)  driving the robot around the environment with the keyboard\n 3) for driving the robot around environment with the keyboard, but with a assistance for avoiding the obstacles" << std::endl;
		
		//Get the command pressed from the stdin and store it in a variable
		std::cin >> choice;
		
		//Check if the command pressed by the user is correct
		if(choice == 1 || choice == 2 || choice == 3)
		{
			std::cout << "the modality chosen is: " << choice << std::endl;

			usr_srv.request.command = choice;
			client.waitForExistence();
			
			//Call a service
			client.call(usr_srv);
			
			//Put into 'resp' variable the response by the server. It tells me if the request is satisfied or not
			resp = usr_srv.response.feedback;
			
			//Print the feedback: it returns '1' if the request is satisfied, otherwise '0'
			std::cout << "feedback: " << resp << std::endl;
		}
		//If the user choice is invalid, it is not sent to the server
		else
		{
			std::cout << "Invalid command, retype" << std::endl; //Advise the user to retype beacuse the choice is invalid
		}
		//ros::spin();		
		ros::spinOnce();	
	}	
	return 0;

}
