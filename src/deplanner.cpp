#include <ros/ros.h>
#include <cerlab_uav/deplanner.h>
#include <mutex>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



deplanner::deplanner(const ros::NodeHandle& _nh):nh(_nh){
	odom_received = false;
	octomap_received = false;
	state_received = false;
	map_ready = false;
	new_path = false;
	new_plan = true;


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

	goal_mutex_.reset(new std::mutex);
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
	if (new_path and new_plan){
		AbstractOcTree* abtree = octomap_msgs::binaryMsgToMap(*bmap);
		tree_ptr = dynamic_cast<OcTree*>(abtree);
		tree_ptr->setResolution(RES);
		map_ready = true;
		// while (ros::ok() and new_path){
		// 	// ROS_INFO("Wait for planning");
		// 	rate.sleep();
		// }
		// delete abtree;
		// map_ready = false;
	}
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
	bool takeoff = false;
	bool scan_around = false;
	int scan_count = 0;
	bool first_time = true;
	bool replan = false;
	int path_idx = 0;
	Node* next_goal_target;
	Node* last_goal;
	std::vector<visualization_msgs::Marker> map_vis_array;
	bool rotate = true;
	ros::Rate rate(10.0);




	while (ros::ok()){
		// ROS_INFO("planning");
		if (new_plan){
			// ROS_INFO("NEW PLAN");
			if (not takeoff){
				goal_pose.pose.position.z = goal_pose.pose.position.z + 0.8;
				takeoff = true;
				ROS_INFO("Set Takeoff attitude: 0.8m.");
			}
			// Initial Scan
			else if(takeoff and not scan_around){
				// Rotate three times for 120 degrees
				if (scan_count < 9){
					goal_pose.pose.orientation = quaternion_from_rpy(0, 0, current_node.yaw+PI_const*2/10);
				}
				// else{
				// goal_pose.pose.position.x = goal_pose.pose.position.x + 0.3;
				// }
				++scan_count;
				if (scan_count == 10){
					scan_around = true;
					new_path = true;
				} 
			}
			else{
				if (new_path){
					while (ros::ok() and not map_ready){
						// ROS_INFO("wait for map");
						rate.sleep();
					}
					Node* start;
					if (first_time){
						roadmap = new PRM ();
						roadmap = buildRoadMap(*tree_ptr, roadmap, path,  NULL, map_vis_array);
						start = findStartNode(roadmap, &current_node, *tree_ptr);
						first_time = false;
					}
					else{
						start = last_goal;
						roadmap = buildRoadMap(*tree_ptr, roadmap, path,  start, map_vis_array);
					}
					std::vector<Node*> goal_candidates = getGoalCandidates(roadmap);
					path = findBestPath(roadmap, start, goal_candidates, *tree_ptr, replan);
					last_goal = *(path.end() - 1);
					new_path = false;
					map_ready = false;
					// ++path_idx;
					path_idx = 0;
				}
				

				next_goal_target = path[path_idx];
				// Generate Goal for Publishing
				if (rotate){
					goal_pose.pose.orientation = quaternion_from_rpy(0, 0, next_goal_target->yaw); 
					rotate = false;
				}
				else{
					goal_pose.pose.position.x = next_goal_target->p.x();
					goal_pose.pose.position.y = next_goal_target->p.y();
					goal_pose.pose.position.z = next_goal_target->p.z();
					rotate = true;
				}

				++path_idx;
				if (path_idx >= path.size()){
					new_path = true;
				} 
			}
			new_plan = false;
		}


		if (new_plan == false){
			new_plan = isReach();
		}
		// cout << "new plan: " << new_plan << "new path: " << new_path << endl;
		// rate.sleep();
		ros::spinOnce();
	}
}

void deplanner::publishGoal(){
	ros::Rate rate(10.0);
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
        std::lock_guard<std::mutex> goal_guard(*(goal_mutex_));

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
	if (dx < 0.1 and dy < 0.1 and dz < 0.1 and dyaw < 0.1){
		return true;
	}
	else{
		return false;
	}
}

std::vector<Node*> deplanner::getGoalCandidates(PRM* roadmap){
	std::vector<Node*> goal_candidates;
	double thresh = 0.1;
	double min_number = 10;
	double max_num_voxels = 0;
	// Go over node which has number of voxels larger than 0.5 maximum
	bool first_node = true;
	std::priority_queue<Node*, std::vector<Node*>, GainCompareNode> goal_nodes = roadmap->getGoalNodes();
	while (true){
		Node* n = goal_nodes.top();
		goal_nodes.pop();

		if (n->num_voxels < max_num_voxels * thresh){
			break;
		}

		if (first_node){
			max_num_voxels = n->num_voxels;
			first_node = false;
		}
		goal_candidates.push_back(n);
	}

	if (goal_candidates.size() < min_number){
		Node* n = goal_nodes.top();
		goal_nodes.pop();
		goal_candidates.push_back(n);
	}

	return goal_candidates;
}

Node* deplanner::findStartNode(PRM* map, Node* current_node, OcTree& tree){
	std::vector<Node*> knn = map->kNearestNeighbor(current_node, 15);
	for (Node* n: knn){
		bool has_collision = checkCollision(tree, n, current_node);
		if (not has_collision){
			return n;
		}
	}
	ROS_INFO("No Valid Node for Start");
	return NULL;
}

std::vector<Node*> deplanner::findBestPath(PRM* roadmap,
                                Node* start,
                                std::vector<Node*> goal_candidates,
                                OcTree& tree,
                                bool replan=false){
	double linear_velocity = 0.3;
	double angular_velocity = 0.8;
	double current_yaw = start->yaw;
	double score_thresh = 0.5;

	std::vector<Node*> final_path;
	// 1. Generate all the path
	std::vector<std::vector<Node*>> path_vector;
	std::vector<double> path_score; // Score = num_voxels/time
	std::vector<Node*> candidate_path;
	int count_path_id = 0;

	for (Node* goal: goal_candidates){
		candidate_path = AStar(roadmap, start, goal, tree, replan);	

		
		// find the appropriate yaw for nodes:
	  		// 1. sensor range from node in this yaw can see next node
		double score = 0;
		double total_num_voxels = 0;
		double total_unknwon;
		for (int i=0; i<candidate_path.size(); ++i){
			// last one
			if (i == candidate_path.size()-1){
				double max_yaw = 0;
				double max_voxels = 0;
				total_unknwon = calculateUnknown(tree, candidate_path[i], dmax);
				for (double yaw: yaws){
					if (candidate_path[i]->yaw_num_voxels[yaw] > max_voxels){
						max_voxels = candidate_path[i]->yaw_num_voxels[yaw];
						max_yaw = yaw;
					}
				}
				candidate_path[i]->yaw = max_yaw;
				total_num_voxels += max_voxels;
			}
			else{

				Node* this_node = candidate_path[i];
				Node* next_node = candidate_path[i+1];
				point3d direction = next_node->p - this_node->p;
				double node_yaw = atan2(direction.y(), direction.x());
				if (node_yaw < 0){
					node_yaw = 2*PI_const - (-node_yaw);
				}
				total_num_voxels += interpolateNumVoxels(this_node, node_yaw);
			}
		}

		if (candidate_path.size() != 0){
			for (int i=0; i<candidate_path.size()-1; ++i){
				Node* this_node = candidate_path[i];
				Node* next_node = candidate_path[i+1];
				point3d direction = next_node->p - this_node->p;
				double node_yaw = atan2(direction.y(), direction.x());
				if (node_yaw < 0){
					node_yaw = 2*PI_const - (-node_yaw);
				}
				candidate_path[i]->yaw = node_yaw;
			}
			double total_path_length = calculatePathLength(candidate_path);
			double total_path_rotation = calculatePathRotation(candidate_path, current_yaw);
			double total_time = total_path_length/linear_velocity + total_path_rotation/angular_velocity;
			score = total_num_voxels/total_time;
		}
		path_score.push_back(score);
		path_vector.push_back(candidate_path);
		++count_path_id;
	}

	// Find best score
	double best_score = 0;
	double best_idx = 0;
	for (int i=0; i<path_vector.size(); ++i){
		if (path_score[i] > best_score){
			best_score = path_score[i];
			best_idx = i;
		}
	}

	final_path = path_vector[best_idx];
	for (int i=0; i<final_path.size()-1; ++i){
		Node* this_node = final_path[i];
		Node* next_node = final_path[i+1];
		point3d direction = next_node->p - this_node->p;
		double node_yaw = atan2(direction.y(), direction.x());
		if (node_yaw < 0){
			node_yaw = 2*PI_const - (-node_yaw);
		}
		final_path[i]->yaw = node_yaw;
	}

	return final_path;
}

double deplanner::calculatePathLength(std::vector<Node*> path){
	int idx1 = 0;
	double length = 0;
	for (int idx2=1; idx2<=path.size()-1; ++idx2){
		length += path[idx2]->p.distance(path[idx1]->p);
		++idx1;
	}
	return length;
}

double deplanner::interpolateNumVoxels(Node* n, double yaw){
	// Find the interval then interpolate
	double min_yaw, max_yaw;
	for (int i=0; i<yaws.size()-1; ++i){
		if (yaw > yaws[i] and yaw < yaws[i+1]){
			min_yaw = yaws[i];
			max_yaw = yaws[i+1];
			break;
		}
	}

	// Interpolate
	std::map<double, int> yaw_num_voxels = n->yaw_num_voxels;
	double num_voxels = yaw_num_voxels[min_yaw] + (yaw - min_yaw) * (yaw_num_voxels[max_yaw]-yaw_num_voxels[min_yaw])/(max_yaw - min_yaw);
	return num_voxels;
}

double deplanner::calculatePathRotation(std::vector<Node*> path, double current_yaw){
	int idx1 = 0;
	double rotation = 0;
	double start_yaw = path[idx1]->yaw;
	double dyaw_start = start_yaw - current_yaw;
	if (std::abs(dyaw_start) < PI_const){
		rotation += std::abs(dyaw_start);
	}
	else{
		rotation += 2*PI_const - std::abs(dyaw_start);
	}

	for (int idx2=1; idx2<=path.size()-1; ++idx2){
		double this_yaw = path[idx1]->yaw;
		double next_yaw = path[idx2]->yaw;
		double dyaw = next_yaw - this_yaw;
		if (std::abs(dyaw) < PI_const){
			rotation += std::abs(dyaw);
		}
		else{
			rotation += 2*PI_const - std::abs(dyaw);
		}
		++idx1;
	}
	return rotation;
}