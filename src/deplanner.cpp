#include <ros/ros.h>
#include <cerlab_uav/deplanner.h>


deplanner::deplanner(ros::NodeHandle _nh):nh(_nh){
	odom_received = false;
	octomap_received = false;
	odom_sub = nh.subscribe("/mavros/local_position/odom", 10, &deplanner::odom_cb, this);
	octomap_sub = nh.subscribe("/octomap_binary", 10, &deplanner::octomap_cb, this);

	ros::Rate rate(10.0);
	while (ros::ok() and (not octomap_received or not odom_received)){
		ros::spinOnce();
        rate.sleep();
	}
	ROS_INFO("Map and Odom Message Received!");
	ros::spin();

}


void deplanner::odom_cb(const nav_msgs::OdometryConstPtr& odom){
	double x = odom->pose.pose.position.x;
	double y = odom->pose.pose.position.y;
	double z = odom->pose.pose.position.z;
	current_node.p.x() = x;
	current_node.p.y() = y;
	current_node.p.z() = z;
	geometry_msgs::Quaternion quat = odom->pose.pose.orientation;
	tf2::Quaternion tf_quat;
	double current_roll, current_pitch, current_yaw;
	tf2::Matrix3x3(tf_quat).getRPY(current_roll, current_pitch, current_yaw);
	if (not odom_received){
		odom_received = true;
		ROS_INFO("Odom Received!");	
	}
}

void deplanner::octomap_cb(const octomap_msgs::Octomap::ConstPtr& bmap){
	abtree = octomap_msgs::binaryMsgToMap(*bmap);
	tree_ptr = dynamic_cast<OcTree*>(abtree);
	tree_ptr->setResolution(RES);
	delete abtree;
	if (not octomap_received){
		octomap_received = true;
		ROS_INFO("Octomap Received!");
	}
}

void deplanner::planning(){

}

deplanner::~deplanner(){}