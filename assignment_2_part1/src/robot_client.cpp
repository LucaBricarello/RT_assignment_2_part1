#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2024/PlanningAction.h>



// main ---------------------------------------


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_planning");
  ros::NodeHandle nh;

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> ac("/reaching_goal", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, asking the user what to do.\n");
  
  while (ros::ok())
  {
  	ROS_INFO("Enter 1 to send a new goal, 2 to cancel the current goal, 0 to close the node:\n");
  	
  	int choice; 
  	std::cin >> choice;
  
  	if (choice == 1)
  	{
  		assignment_2_2024::PlanningGoal goal;
  	
  		ROS_INFO("Enter desired coordinate x:\n");
  		std::cin >> goal.target_pose.pose.position.x;

  		ROS_INFO("Enter desired coordinate y:\n");
  		std::cin >> goal.target_pose.pose.position.y;

  		ac.sendGoal(goal);
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
