#include <ros/ros.h>
#include <cerlab_uav/deplanner.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "planner_node");
	ros::NodeHandle nh;
	deplanner p = deplanner(nh);
	return 0;
}