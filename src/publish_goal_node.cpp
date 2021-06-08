#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


geometry_msgs::PoseStamped goal_msg;

void goal_cb(const geometry_msgs::PoseStampedConstPtr& goal){
	goal_msg.header = goal->header;
	goal_msg.pose = goal->pose;
	// std::cout << goal->pose.position.x << std::endl;
}



int main(int argc, char **argv){
	ros::init(argc, argv, "publish_goal_node");
	ros::NodeHandle nh;
	ros::Subscriber goal_sub =  nh.subscribe<geometry_msgs::PoseStamped>("cerlab_uav/goal", 10, goal_cb);
	ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::Rate rate(10.0);
	while(ros::ok()){
		goal_pub.publish(goal_msg);
		ros::spinOnce();
		rate.sleep();
	}


	return 0;
}