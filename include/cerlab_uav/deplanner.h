#ifndef DEPLANNER_H
#define DEPLANNER_H
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <cerlab_uav/prm.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <thread>
#include <mutex>


using std::cout; using std::endl;

class deplanner{
private:
	ros::NodeHandle nh;
	ros::Subscriber odom_sub;
	ros::Subscriber octomap_sub;
	ros::Subscriber state_sub;
	ros::Publisher goal_pub;
	ros::ServiceClient arming_client;
	ros::ServiceClient set_mode_client;
	Node current_node; // used as current position for planning
	AbstractOcTree* abtree;
	OcTree* tree_ptr; // current map for planning
	mavros_msgs::State current_state;
	bool odom_received;
	bool octomap_received;
	bool state_received;
	geometry_msgs::PoseStamped goal_pose;



	std::unique_ptr<std::mutex> status_mutex_;


public:
	std::thread worker_;

	deplanner(const ros::NodeHandle& _nh);
	~deplanner();
	void odom_cb(const nav_msgs::OdometryConstPtr& odom);
	void octomap_cb(const octomap_msgs::Octomap::ConstPtr& bmap);
	void state_cb(const mavros_msgs::State::ConstPtr& mavros_state);
	void planning();
	void publishGoal();
	bool isReach();




};


#endif
