#include <ros/ros.h>
#include <cerlab_uav/deplanner.h>
#include <mutex>



deplanner::deplanner(const ros::NodeHandle& _nh):nh(_nh){
	odom_received = false;
	octomap_received = false;
	state_received = false;
	odom_sub = nh.subscribe("/mavros/local_position/odom", 10, &deplanner::odom_cb, this);
	octomap_sub = nh.subscribe("/octomap_binary", 10, &deplanner::octomap_cb, this);
	state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &deplanner::state_cb, this);

	goal_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

	ros::Rate rate(10.0);
	while (ros::ok() and (not octomap_received or not odom_received or not state_received)){
		ros::spinOnce();
        rate.sleep();
	}
	ROS_INFO("Map and Odom Message Received!");

	goal_pose.header.frame_id = "map";
	goal_pose.pose.position.x = current_node.p.x();
	goal_pose.pose.position.y = current_node.p.y();
	goal_pose.pose.position.z = current_node.p.z();
	goal_pose.pose.orientation = quaternion_from_rpy(0, 0, current_node.yaw);

	status_mutex_.reset(new std::mutex);
    worker_ = std::thread(&deplanner::publishGoal, this);
    this->planning();

}

deplanner::~deplanner(){}

void deplanner::odom_cb(const nav_msgs::OdometryConstPtr& odom){
	double x = odom->pose.pose.position.x;
	double y = odom->pose.pose.position.y;
	double z = odom->pose.pose.position.z;
	current_node.p.x() = x;
	current_node.p.y() = y;
	current_node.p.z() = z;
	geometry_msgs::Quaternion quat = odom->pose.pose.orientation;
	double current_yaw = rpy_from_quaternion(quat);
	current_node.yaw = current_yaw;

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

void deplanner::state_cb(const mavros_msgs::State::ConstPtr& mavros_state){
	current_state = *mavros_state;
	if (not state_received){
		state_received = true;
		ROS_INFO("MAVROS State Received!");
	}
}

void deplanner::planning(){
	bool new_plan = true;
	bool takeoff = false;
	bool scan_around = false;
	int scan_count = 0;
	while (ros::ok()){
		// ROS_INFO("planning");
		if (new_plan){
			ROS_INFO("NEW PLAN");
			new_plan = false;
			if (not takeoff){
				goal_pose.pose.position.z = goal_pose.pose.position.z + 0.5;
				takeoff = true;
				ROS_INFO("Set Takeoff attitude: 0.5m.");
			}
			// Initial Scan
			else if(takeoff and not scan_around){
				// Rotate three times for 120 degrees
				goal_pose.pose.orientation = quaternion_from_rpy(0, 0, current_node.yaw+PI_const*2/10);
				++scan_count;
				if (scan_count == 11){
					scan_around = true;
				} 
			}

		}
		if (new_plan == false){
			new_plan = isReach();
		}
		ros::spinOnce();

	}
}

void deplanner::publishGoal(){
	ros::Rate rate(20.0);
	for(int i = 100; ros::ok() && i > 0; --i){
        goal_pose.header.stamp = ros::Time::now();
        goal_pub.publish(goal_pose);
        ros::spinOnce();
        rate.sleep();
    }

	ros::Time last_request = ros::Time::now();
	mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
	while (ros::ok()){	
		if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
		goal_pose.header.stamp = ros::Time::now();
		goal_pub.publish(goal_pose);
		ros::spinOnce();
		rate.sleep();
	}
}

bool deplanner::isReach(){
	double goal_x = goal_pose.pose.position.x;
	double goal_y = goal_pose.pose.position.y;
	double goal_z = goal_pose.pose.position.z;
	double dx = std::abs(current_node.p.x() - goal_x);
	double dy = std::abs(current_node.p.y() - goal_y);
	double dz = std::abs(current_node.p.z() - goal_z);

	geometry_msgs::Quaternion quat = goal_pose.pose.orientation;
	double goal_yaw = rpy_from_quaternion(quat);


	double dyaw = std::abs(goal_yaw - current_node.yaw);
	// cout << goal_yaw << " " << current_node.yaw << endl;
	// cout << dx << " " << dy << " " << " " << dz << " "<< dyaw << endl; 
	if (dx < 0.15 and dy < 0.15 and dz < 0.15 and dyaw < 0.1){
		return true;
	}
	else{
		return false;
	}
}