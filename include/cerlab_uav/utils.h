#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <time.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
// #include <rh_nbv/kdtree.h>
#define PI_const 3.1415926
using std::cout; using std::endl; using std::vector;
using std::set; using std::pair; using std::map;
using namespace octomap;
typedef pair<point3d, float> sample;

void print_point3d(point3d p){
	cout << std::fixed << std::setprecision(2)<< " Point: " << "(" << p.x() << ", " <<
	p.y() << ", " << p.z() << ")    " << endl;	
}

void print_point3d_vector(vector<point3d> p){
	int count = 0;

	for (vector<point3d>::iterator itr = p.begin(); 
			itr != p.end(); ++itr){
		print_point3d(*itr);

		/*
			cout << "Point Num: " << count+1 << endl; 
			cout 
			<< "x: " << itr->x() << endl 
			<< "y: " << itr->y() << endl 
			<< "z: " << itr->z() << endl;
		*/

	}
}

void print_node(Node n){
	cout << "+-----------Node Info----------+" << endl;
	print_point3d(n.p);
	cout << " yaw: " << n.yaw << "                    "<<  endl;
	cout << " voxels: " << n.num_voxels << "             " << endl;
	cout << "+------------------------------+" << endl;
}


void print_node_vector(std::vector<Node*> path){
	cout << "=========>Path<========" << endl;
	for (std::vector<Node*>::iterator itr=path.begin();
			itr != path.end(); ++itr){
		print_node(*(*itr));
	}

	cout << "=========>END<==========" << endl;
}

void print_path(std::vector<Node> path){
	cout << "=========>Path<========" << endl;
	for (std::vector<Node>::iterator itr=path.begin();
			itr != path.end(); ++itr){
		print_node((*itr));
	}

	cout << "=========>END<==========" << endl;
}

geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
	if (yaw > PI_const){
		yaw = yaw - 2*PI_const;
	}
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    return quaternion;
}

double rpy_from_quaternion(geometry_msgs::Quaternion quat){
	// return is [0, 2pi]
	tf2::Quaternion tf_quat;
	tf2::convert(quat, tf_quat);
	double roll, pitch, yaw;
	tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
	// in the planner, the angle is [0, 2pi] instead of [-pi, pi]
	if (yaw < 0){
		yaw = 2 * PI_const - (-yaw);
	}
	return yaw;
}