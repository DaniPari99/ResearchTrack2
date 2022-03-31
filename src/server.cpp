/**
* \file server.cpp
* \brief server for the execution of the three modalities
* \author Parisi Daniele Martino
* \version 1.0
* \date 27/02/2022
*
* \details
*
* Subscribes to:<BR>
* /new_cmd_vel it contains the current velocity of the robot
* /scan it contains the current distances from the obstacles
* /move_base/feedback it contains the current position of the robot
*
* Publishes to:<BR>
* /move_base/goal
* /cmd_vel
* /move_base/cancel
*
* services:<BR>
* /user
*
* Description:
*
* The code implements the logic to execute one among 3 different modality chosen by the user for driving the robot among a virtual environment
**/

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "final_assignment/User.h"
#include "std_srvs/Empty.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"


#define DTH 1 ///< Macro-definition for the threshold distance
#define GOAL_TH 0.5 ///< Macro-definition for the goal threshold


//Initialize a publishers
ros::Publisher pub_goal; ///< Global publisher for the goal
ros::Publisher pub_vel; ///< Global publisher for the velocity
ros::Publisher pub_cancel; ///< Global publisher for cancelling the goal

bool manDrive = false; ///< Global bool variable for enabling 2nd modality
bool collisionAvoidence = false; ///< Global bool variable for enabling 3rd modality
bool goal = false; ///< Global bool variable to store the status of the goal

float target_x; ///< global variable for storing the x coordinate of goal chosen by the user
float target_y; ///< global variable for storing the y coordinate of goal chosen by the user

std::string id = ""; ///< Global variable for the id of the goal

geometry_msgs::Twist my_vel; ///< Velocity to be checked by the collision avoidence before being published

actionlib_msgs::GoalID canc_goal; ///< message for cancelling a goal when it is reached

/**
* \brief function to cancell a goal if it is reached
* \return true if the goal is cancelled, otherwise false
*
* The function uses the status of the global variable 'goal' for cancelling the goal
**/
bool cancelGoal()
{
	if(!goal)
		return false;
	canc_goal.id = id;
	pub_cancel.publish(canc_goal);
	goal = false;
	return true;
	
}

/**
* \brief function to compute the distance from the goal
* \param msg robot's coordinates and goal's status
*
* The function calculate the distance of the robot from 
* the goal and if the distance is less than or equal to 
* a threshold the current coal is cancelled
**/
void currentPositionCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{

    // Update the goal ID if there is a new goal
    if (id != msg->status.goal_id.id) 
    {
        id = msg->status.goal_id.id;
    }
    
    if(goal)
    {
    	//take the current robot position
        float dist_x;
	float dist_y;
	float robot_x = msg->feedback.base_position.pose.position.x;
	float robot_y = msg->feedback.base_position.pose.position.y;
    
	//calculate the error from the actual position and the goal position
	dist_x = robot_x - target_x;
	dist_y = robot_y - target_y;

	//if the robot is on the goal position
	if (abs(dist_x) <= GOAL_TH && abs(dist_y) <= GOAL_TH)
	{
	     printf("Goal reached\n");
	     cancelGoal();
	}

     }
}

/**
* \brief function to do different things according to the chosen modality
* \param msg linear and angular velocities of the robot
*
* Callback function called whenever the teleop twist keyboard want to change the robot velocity
If the robot isn't neither in manual driving nor in collision avoidence modality the message is ignored
If the robot is in manual driving modality the message is sent to the cmd_vel topic
If the robot is in collision avoidence modality the message is saved in a global variable (my_vel)
and it will be corrected by the collision avoidence function before being published
**/
void getVelCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
    if(!manDrive && !collisionAvoidence)
    	return;
    if(manDrive && !collisionAvoidence)
    {
    	pub_vel.publish(msg);
    	return;
    }
    if(!manDrive && collisionAvoidence)
    {
    	my_vel.linear.x = msg->linear.x;
    	my_vel.angular.z = msg->angular.z;
    }
}

/**
* \brief function to compute the minimum element of an array
* \param scan[] the array 
* \param size the dimension of the array
*
* Function for computing the min element of a given array and its size
**/
double computeMinDist(double scan[], int size)
{
	double min_dist = 100;
	for(int i=0; i < size; i++)
	{
		if(scan[i] < min_dist)
		{
			min_dist = scan[i];
		}
	}
	return min_dist;
}


/**
* \brief function to check the obstacles' distances and in case of possible collisions it corrects the trajectory
* \param msg laser scanner's ranges
*
* This function is capable of getting the laser scanner's data, dividing them into different parts,
* that correspond to the distances from left, front-left, front, front right and right.
* According to the distances, if we can have possible collisions the velocities are modified.
**/
void collisionAvoidenceCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	if(!collisionAvoidence)
		return;
	//get array ranges by laserscan
	float scan[msg->ranges.size()];
	
	for(int j = 0; j < msg->ranges.size(); j++)
	{
		scan[j] = msg->ranges[j];
	}
	
	//divide the array ranges values in 5 sectors
	int sector_size = (msg->ranges.size())/5;
	double left[sector_size];
	double front_left[sector_size];
	double front[sector_size];
	double front_right[sector_size];
	double right[sector_size];
	
	//fill sector arrays
	for(int i = 0; i < sector_size; i++)
	{
		right[i] = scan[i];
	}
	
	for(int i = 0; i < sector_size; i++)
	{
		front_right[i] = scan[i + sector_size];
	}
	
	for(int i = 0; i < sector_size; i++)
	{
		front[i] = scan[i + 2*sector_size];
	}
	
	for(int i = 0; i < sector_size; i++)
	{
		front_left[i] = scan[i + 3*sector_size];
	}
	
	for(int i = 0; i < sector_size; i++)
	{
		left[i] = scan[i + 4*sector_size];
	}
	
		
	//check if the robot is going to crash to obstacles and in case stop it
	if(computeMinDist(front, sector_size) < DTH) 
	{
		if(my_vel.linear.x > 0 && my_vel.angular.z == 0) 
		{
			my_vel.linear.x = 0;
			printf("Attenction: there is an obstacle in front\n"); 
		}
	}
		
	if(computeMinDist(front_left, sector_size) < DTH)
	{
		if(my_vel.linear.x > 0 && my_vel.angular.z > 0)
		{
			my_vel.linear.x = 0;
			my_vel.angular.z = 0;
			printf("Attenction: there is an obstacle on the front-left\n");
		}
	}
	
	if(computeMinDist(front_right, sector_size) < DTH)
	{
		if(my_vel.linear.x > 0 && my_vel.angular.z < 0)
		{
			my_vel.linear.x = 0;
			my_vel.angular.z = 0;
			printf("Attenction: there is an obstacle on the front-right\n");
		}
	}
		
	if(computeMinDist(left, sector_size) < DTH)
	{
		if(my_vel.linear.x == 0 && my_vel.angular.z > 0)
		{
			my_vel.angular.z = 0;
			printf("Attenction: there is an obstacle on the left\n");
		}
	}
		
	if(computeMinDist(right, sector_size) < DTH)
	{
		if(my_vel.linear.x == 0 && my_vel.angular.z < 0)
		{
			my_vel.angular.z = 0;
			printf("Attenction: there is an obstacle on the right\n");
		}
	}

    	pub_vel.publish(my_vel); //publish the corrected velocity for avoiding collisions 
		
}



/**
* \brief Function to direct the code to the modality chosen
* \param req
* \param res
*
* This function is capable of direct the code through the modality chosen by the user.
* I do this with a list of 'if' statements and in each body of the 'if' I set the value of
* the boolean global variables according to the chosen modality
**/
bool serviceCallback(final_assignment::User::Request &req, final_assignment::User::Response &res)
{	
	//Initialize the node handler
	ros::NodeHandle nh;
	
	//if 1st modality is chosen
	if(req.command == 1)
	{	
		std::cout <<"1st modality chosen" << std::endl;
		manDrive = false;
		collisionAvoidence = false;
		
		
		move_base_msgs::MoveBaseActionGoal goal_msg;
		
		std::cout <<"What target do you want to achieve?\n" << std::endl;
		
		//get the user's position goal chosen
		std::cout <<"Type a x" << std::endl;
		std::cin >> target_x;		
		std::cout <<"Type a y" << std::endl;
		std::cin >> target_y;
		
		//set the goal status at true
		goal = true;
		
		goal_msg.goal.target_pose.header.frame_id = "map";
		goal_msg.goal.target_pose.pose.orientation.w = 1;
		
		//fill the goal position fields
		goal_msg.goal.target_pose.pose.position.x = target_x;
		goal_msg.goal.target_pose.pose.position.y = target_y;
		
		//publish the goal chosen 
		pub_goal.publish(goal_msg);
		
		
		res.feedback = true;		
	}
	//if 2nd modality is chosen
	else if(req.command == 2)
	{
		std::cout << "2nd modality chosen" << std::endl;
		cancelGoal();
		
		manDrive = true;
		collisionAvoidence = false;

		res.feedback = true;
	}
	//if 3rd modality is chosen
	else if(req.command == 3)
	{
		std::cout << "3rd modality chosen" << std::endl;
		cancelGoal();
		
		collisionAvoidence = true;
		manDrive = false;

		res.feedback = true;
	}
	//If something is wrong the feedback must be negative 
	else
	{
		res.feedback = false;
	}
	
	return res.feedback;
	
}

/**
* \brief main function
* \param argc
* \param argv
*
* Function for publishing, subscribing to some topics or to define the service for implement the chosen modality
* 
**/

int main (int argc, char **argv)
{
	//Initialize the node, setup the NodeHandle for handling the communication with the ROS system   
	ros::init(argc, argv, "server"); 
	ros::NodeHandle nh;
	
	//publish to some topics
	pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1);
	pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	pub_cancel = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1); // Cancel actual goal
	
	//subsrcribe to some topics
	ros::Subscriber sub_keyBoard = nh.subscribe("/new_cmd_vel", 1, getVelCallback); // Get velocity from teleop twist keyboard
	ros::Subscriber sub_laser = nh.subscribe("/scan", 1, collisionAvoidenceCallback); // Laser scanner
	ros::Subscriber sub_position = nh.subscribe("/move_base/feedback", 1, currentPositionCallback); // Current position
	
	//Define a service 
	ros::ServiceServer service = nh.advertiseService("/user", serviceCallback);
	
	ros::spin();
	return 0;
}
