/**
* \file target_coords_srv.cpp
* \brief Service node for retrieving the last target coordinates.
* \author Luca Bricarello
* \version 1.1
* \date 09/03/2025
*
* \details
*
* **Subscribes to**: <BR>
* - /targ_coords
*
* **Services**: <BR>
* - /get_last_target
*
* **Description**: <BR>
* This node provides a service that returns the last target coordinates received.
* It subscribes to the /targ_coords topic to update the last known target position
* and responds to service calls with the last recorded coordinates.
*/
#include "ros/ros.h"
#include "assignment_2_part1/LastTarget.h"
#include "geometry_msgs/Point.h"

/**
* \brief Stores the last received target X coordinate.
*/
float last_target_coord_x = 0;

/**
* \brief Stores the last received target Y coordinate.
*/
float last_target_coord_y = 0;

/**
* \brief Service callback to return the last recorded target coordinates.
*
* This function is triggered when a client requests the "get_last_target" service.
* It responds with the last known target X and Y coordinates.
*
* \param req The service request (unused in this case).
* \param res The service response containing the last known target coordinates.
* \return true if the service executed successfully.
*/
bool srv_curr_targ_clbk(assignment_2_part1::LastTarget::Request &req, assignment_2_part1::LastTarget::Response &res)
{
	res.x = last_target_coord_x;
	res.y = last_target_coord_y;
	ROS_INFO("Service called: Returning last target (x=%.2f, y=%.2f)", res.x, res.y);
	
	return true;
}

/**
* \brief Callback function for updating the last target coordinates.
*
* This function updates the stored target coordinates whenever a new message
* is received on the /targ_coords topic.
*
* \param msg The received message containing the latest target coordinates.
*/
void update_targ_coords_clbk(const geometry_msgs::Point::ConstPtr& msg)
{
	last_target_coord_x = msg->x;
	last_target_coord_y = msg->y;
}

/**
* \brief Main function of the last target service node.
*
* Initializes the ROS node, subscribes to the /targ_coords topic, and advertises
* the "get_last_target" service. The node continuously listens for incoming
* messages and service requests using ros::spin().
*
* \param argc Number of input arguments (if any).
* \param argv Pointer to array of arguments (if any).
* \return 0 on successful execution.
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "service_node_last_target");
	ros::NodeHandle nh;
	
	// Subscriber to the /targ_coords topic
	ros::Subscriber targ_coords_sub = nh.subscribe("/targ_coords", 10, update_targ_coords_clbk);
	
	ros::ServiceServer service = nh.advertiseService("get_last_target", srv_curr_targ_clbk);
	
	ros::spin();
	return 0;
}
