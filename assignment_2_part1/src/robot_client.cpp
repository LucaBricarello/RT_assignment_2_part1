/**
* \file robot_client.cpp
* \brief Client node for interacting with the robot's action server.
* \author Luca Bricarello
* \version 1.1
* \date 09/03/2025
*
* \details
*
* **Subscribes to**: <BR>
* - /odom
*
* **Publishes to**: <BR>
* - /robot_state
* - /targ_coords
*
* **ActionServers**: <BR>
* - /reaching_goal
*
* **Description**: <BR>
* This node allows a user to send goals to the action server, cancel them, and monitor the robot state.
* It subscribes to the /odom topic to track the robot's position and velocity and publishes
* the robot's state and target coordinates.
**/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2024/PlanningAction.h>

#include <nav_msgs/Odometry.h>
#include <assignment_2_part1/RobotState.h> // Custom message
#include "geometry_msgs/Point.h"
#include <thread>  // Include for std::thread


// Global variables ---------------------------------------

/** 
* \brief X coordinate of the current goal.
*/
float currentGoalX;

/** 
* \brief Y coordinate of the current goal.
*/
float currentGoalY;

/** 
* \brief Counter used to ensure a single success message.
*/
int cnt = 0;

/** 
* \brief Publisher for the robot's state.
*/
ros::Publisher state_pub; // Publisher for the robot's state


// feedbackClbk ---------------------------------------

/**
* \brief Callback function for action feedback.
* 
* This function checks if the robot has reached its goal within a threshold.
* If the robot is close enough to the goal, a success message is printed.
* 
* \param feedback Feedback message containing the robot's actual position.
*/
void feedbackClbk(const assignment_2_2024::PlanningFeedbackConstPtr &feedback)
{
    float threshold = 0.5;
    if ((abs(feedback->actual_pose.position.x - currentGoalX) < threshold) && (abs(feedback->actual_pose.position.y - currentGoalY) < threshold) && (cnt < 1))
    {
    	ROS_INFO("Target successfully reached!");    	
    	cnt++;
    }
}


// odomClbk ---------------------------------------

/**
* \brief Callback function for the /odom topic.
* 
* Extracts the robot's position and velocity from the Odometry message
* and publishes it as a custom RobotState message.
* 
* \param msg Odometry message containing position and velocity data.
*/
void odomClbk(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Extract position
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // Extract velocity
    double vel_x = msg->twist.twist.linear.x;
    double vel_z = msg->twist.twist.angular.z;

    // creating custom message
    assignment_2_part1::RobotState state_msg;
    state_msg.x = x;
    state_msg.y = y;
    state_msg.vel_x = vel_x;
    state_msg.vel_z = vel_z;

    // publishing custom message
    state_pub.publish(state_msg);
}


// main ---------------------------------------

/**
* \brief Main function of the client node.
* 
* Initializes the ROS node, sets up publishers and subscribers, implements a graphical interface (UI) 
* to interact with the user and let him send goals to the robot using an action client, it also let's him
* choose an option to cancel the current goal or another option to close this node.
* This node performs some controls on the values inserted by the user, and it also creates a new thread
* that runs the ros::spin() command, this lets the node control for incoming msgs on the subscribed topics
* and eventually run the associated callback function while independently running the UI.
* 
* \param argc Number of input arguments (if any).
* \param argv Pointer to array of arguments (if any).
* 
* \return 0 on successful execution.
*/
int main (int argc, char **argv)
{
  ros::init(argc, argv, "client_to_robotac");
  ros::NodeHandle nh;
  
  // Publisher for the robot's state
  state_pub = nh.advertise<assignment_2_part1::RobotState>("/robot_state", 10);
  
  // Publisher for the robot target coords
  ros::Publisher coord_pub = nh.advertise<geometry_msgs::Point>("/targ_coords", 10);

  // Subscriber to the /odom topic
  ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomClbk);

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> ac("/reaching_goal", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, asking the user what to do.\n");
  
  // Create a thread for ros::spin()
  std::thread spinThread([]() {
  	ros::spin();
  });

  while (ros::ok())
  {
  	ROS_INFO("Enter 1 to send a new goal, 2 to cancel the current goal, 0 to close the node:\n");
  	
  	int choice; 
  	std::cin >> choice;
  	if (std::cin.fail()) 
        {
            	std::cin.clear();
            	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		choice = 3;
        }
  
  	if (choice == 1)
  	{
  		assignment_2_2024::PlanningGoal goal;
  		cnt = 0;
  		
  		while(true)
  		{
  			ROS_INFO("Enter desired coordinate x:\n");
  			std::cin >> goal.target_pose.pose.position.x;
  			if (std::cin.fail()) 
        		{
            			std::cin.clear();
            			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            			ROS_INFO("Invalid input, enter a number.\n");
			
        		}
        		else
        		{
        			break;
        		}
        	}	
  		currentGoalX = goal.target_pose.pose.position.x;
  		
  		while(true)
  		{
  			ROS_INFO("Enter desired coordinate y:\n");
  			std::cin >> goal.target_pose.pose.position.y;
  			if (std::cin.fail()) 
        		{
            			std::cin.clear();
            			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            			ROS_INFO("Invalid input, enter a number.\n");
			
        		}
        		else
        		{
        			break;
        		}
  		}
  		currentGoalY = goal.target_pose.pose.position.y;
  		
  		geometry_msgs::Point coord_msg;
  		coord_msg.x = currentGoalX;
  		coord_msg.y = currentGoalY;
  		
  		coord_pub.publish(coord_msg);

  		ac.sendGoal(goal,
  				actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>::SimpleDoneCallback(),
                        	actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>::SimpleActiveCallback(),
                        	feedbackClbk);
  	}
  	else if (choice == 2)
  	{
  		ROS_INFO("Canceling the current goal.\n");
  		ac.cancelGoal();
  	}
  	else if (choice == 0)
        {
        	ROS_INFO("Exiting...");
        	break;
        }
  	else
  	{
  		ROS_INFO("Wrong input.\n");
  	}
  }

  //exit
  return 0;
}
