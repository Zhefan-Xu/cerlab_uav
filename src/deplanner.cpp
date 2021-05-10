#include <ros/ros.h>
#include <cerlab_uav/deplanner.h>

deplanner::deplanner(ros::NodeHandle _nh){
	nh = _nh;
	odom_sub = nh.subscribe("/mavros/local_position/odom", 10, &deplanner::odom_cb, this);
	octomap_sub = nh.subscribe("octomap_binary", 10, &deplanner::octomap_cb, this);
	ros::spin();

}



void deplanner::odom_cb(const nav_msgs::OdometryConstPtr& odom){
	cout << "odom received" << endl;
}

void deplanner::octomap_cb(const octomap_msgs::Octomap::ConstPtr& bmap){
	cout << "map received" << endl;
}

void deplanner::planning(){

}

deplanner::~deplanner(){}