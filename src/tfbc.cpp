#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>



void callback(const nav_msgs::OdometryConstPtr& odom){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y
		, odom->pose.pose.position.z));
	tf::Quaternion q (odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, 
		odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

}









int main(int argc, char** argv){
	ros::init(argc, argv, "tf_broadcaster");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/mavros/local_position/odom", 10, &callback);
	ros::spin();
	return 0;
}