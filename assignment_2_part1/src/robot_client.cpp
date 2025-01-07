#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2024/PlanningAction.h>

#include <nav_msgs/Odometry.h>
#include <assignment_2_part1/RobotState.h> // Custom message
#include "geometry_msgs/Point.h"
#include <thread>  // Include for std::thread


// Global variables ---------------------------------------


float currentGoalX;
float currentGoalY;
int cnt = 0;

ros::Publisher state_pub; // Publisher for the robot's state


// feedbackClbk ---------------------------------------


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


// Callback for the /odom topic
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
