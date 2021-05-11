#ifndef DEPLANNER_H
#define DEPLANNER_H
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <cerlab_uav/prm.h>
#include <nav_msgs/Odometry.h>


using std::cout; using std::endl;

class deplanner{
private:
	ros::NodeHandle nh;
	ros::Subscriber odom_sub;
	ros::Subscriber octomap_sub;
	Node current_node; // used as current position for planning
	AbstractOcTree* abtree;
	OcTree* tree_ptr; // current map for planning
	bool odom_received;
	bool octomap_received;

public:
	deplanner(ros::NodeHandle _nh);
	~deplanner();
	void odom_cb(const nav_msgs::OdometryConstPtr& odom);
	void octomap_cb(const octomap_msgs::Octomap::ConstPtr& bmap);
	void planning();




};


#endif
