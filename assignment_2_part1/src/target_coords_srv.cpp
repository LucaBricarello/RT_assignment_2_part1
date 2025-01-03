#include "ros/ros.h"
#include "assignment_2_part1/LastTarget.h"
#include "geometry_msgs/Point.h"


float last_target_coord_x = 0;
float last_target_coord_y = 0;

bool srv_curr_targ_clbk(assignment_2_part1::LastTarget::Request &req, assignment_2_part1::LastTarget::Response &res)
{
	res.x = last_target_coord_x;
	res.y = last_target_coord_y;
	ROS_INFO("Service called: Returning last target (x=%.2f, y=%.2f)", res.x, res.y);
	
	return true;
}

void update_targ_coords_clbk(const geometry_msgs::Point::ConstPtr& msg)
{
	last_target_coord_x = msg->x;
	last_target_coord_y = msg->y;
}

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
