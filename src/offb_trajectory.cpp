#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <trajectory_optimization/readfile.h>
#include <trajectory_optimization/mpcPlanner.h>
#include <trajectory_optimization/staticPlanner.h>
#include <trajectory_optimization/vis_utils.h>
#include <nav_msgs/Odometry.h>
#include <chrono> 

using std::cout; using std::endl;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


nav_msgs::Odometry current_odom;
void odom_cb(const nav_msgs::OdometryConstPtr& msg){
    current_odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/local_position/odom", 10, odom_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("cerlab_uav/goal", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    cout << "Connection OK!" << endl;

    // mavros_msgs::PositionTarget pose_target;

    geometry_msgs::PoseStamped pose_target;
    pose_target.header.frame_id = "map";
    pose_target.pose.position.x = 0;
    pose_target.pose.position.y = 0;
    pose_target.pose.position.z = 0.5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pose_target.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose_target);
        ros::spinOnce();
        rate.sleep();
    }




    std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt";
    std::vector<std::vector<pose>> paths = read_waypoint_file(filename);
    std::vector<pose> path = paths[30];


    // parameters
    double res = 0.1; // map resolution
    double xsize = 0.2; double ysize = 0.2; double zsize = 0.1; // Robot collision box size
    int degree = 7; // polynomial degree
    double velocityd = 1; // desired average velocity
    int diff_degree = 4; // Minimum snap (4), minimum jerk (3)
    double perturb = 1; // Regularization term and also make PSD -> PD
    bool shortcut = true; // shortcut waypoints
    double delT = 0.1; // resolution of the final trajectory
    std::vector<pose> loadPath;

    // solve trajectory:
    std::vector<pose> trajectory = staticPlanner(nh, res, xsize, ysize, zsize, degree, velocityd, diff_degree, perturb, path, shortcut, delT, loadPath);

    // MPC
    int horizon = 20; // MPC horizon

    double mass = 1.0; double k_roll = 1.0; double tau_roll = 1.0; double k_pitch = 1.0; double tau_pitch = 1.0; 
    double T_max = 3 * 9.8; double roll_max = PI_const/6; double pitch_max = PI_const/6;
    mpcPlanner mp (horizon);
    mp.loadParameters(mass, k_roll, tau_roll, k_pitch, tau_pitch);
    mp.loadControlLimits(T_max, roll_max, pitch_max);
    mp.loadRefTrajectory(trajectory, delT);


    visualization_msgs::MarkerArray path_msg = wrapVisMsg(loadPath, 0, 0, 0);
    visualization_msgs::MarkerArray trajectory_msg = wrapVisMsg(trajectory, 0, 1, 0);

    ros::Publisher path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_path", 0);
    ros::Publisher trajectory_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 0);
    ros::Publisher mpc_trajectory_vis_pub = nh.advertise<nav_msgs::Path>("mpc_trajectory", 0);

    ros::Rate loop_rate(1/delT);




    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    bool takeoff = false;
    bool reachStart = false;
    pose poseStart = trajectory[0];
    int traj_idx = 1;

    DVector nextStates;
    DVector currentStates (8); 
    VariablesGrid xd;
    currentStates(0) = trajectory[0].x; currentStates(1) = trajectory[0].y; currentStates(2) = trajectory[0].z;

    while(ros::ok()){
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


        double current_x = current_odom.pose.pose.position.x;
        double current_y = current_odom.pose.pose.position.y;
        double current_z = current_odom.pose.pose.position.z;
        double current_vx = current_odom.twist.twist.linear.x;
        double current_vy = current_odom.twist.twist.linear.y;
        double current_vz = current_odom.twist.twist.linear.z;
        geometry_msgs::Quaternion quat = current_odom.pose.pose.orientation;
        double current_roll; double current_pitch; double current_yaw; 
        rpy_from_quaternion(quat, current_roll, current_pitch, current_yaw);


        std::vector<pose> mpc_trajectory;


        if (takeoff == false and reachStart == false){
            // takeoff
            double delta = 0.1;
            // pose_target.pose.position.x = current_x;
            // pose_target.pose.position.y = current_y;
            double dx = std::abs(pose_target.pose.position.x - current_x);
            double dy = std::abs(pose_target.pose.position.y - current_y);
            double dz = std::abs(pose_target.pose.position.z - current_z);
            if (dx < delta and dy < delta and dz < delta){
                takeoff = true;
                pose_target.pose.position.x = poseStart.x;
                pose_target.pose.position.y = poseStart.y;
                pose_target.pose.position.z=  poseStart.z;
                geometry_msgs::Quaternion target_quat = quaternion_from_rpy(0, 0, poseStart.yaw);
                pose_target.pose.orientation = target_quat;
            }
        }

        if (takeoff == true and reachStart == false){
            // go to the start position
            double delta = 0.1;
            double dx = std::abs(pose_target.pose.position.x - current_x);
            double dy = std::abs(pose_target.pose.position.y - current_y);
            double dz = std::abs(pose_target.pose.position.z - current_z);
            if (dx < delta and dy < delta and dz < delta){
                reachStart = true;
            }
        }

        if (takeoff == true and reachStart == true){
            // follow trajectory
            mp.optimize(currentStates, nextStates, mpc_trajectory, xd);
            currentStates(0) = current_x; currentStates(1) = current_y; currentStates(2) = current_z;
            // currentStates(0) = nextStates(0); currentStates(1) = nextStates(1); currentStates(2) = nextStates(2);
            currentStates(3) = current_vx; currentStates(4) = current_vy; currentStates(5) = current_vz;
            // currentStates(3) = nextStates(3); currentStates(4) = nextStates(4); currentStates(5) = nextStates(5); 
            currentStates(6) = nextStates(6); currentStates(7) = nextStates(7); 
            // currentStates = nextStates;
            cout << currentStates << endl;
            cout << nextStates << endl;

            if (traj_idx <= trajectory.size() - 1){
                // pose_target.pose.position.x = trajectory[traj_idx].x;
                // pose_target.pose.position.y = trajectory[traj_idx].y;
                // pose_target.pose.position.z = trajectory[traj_idx].z;
                // geometry_msgs::Quaternion target_quat = quaternion_from_rpy(0, 0, trajectory[traj_idx].yaw);
                int forward_idx = 1;
                pose_target.pose.position.x = mpc_trajectory[forward_idx].x;
                pose_target.pose.position.y = mpc_trajectory[forward_idx].y;
                pose_target.pose.position.z = mpc_trajectory[forward_idx].z;
                geometry_msgs::Quaternion target_quat = quaternion_from_rpy(0, 0, mpc_trajectory[forward_idx].yaw);
                pose_target.pose.orientation = target_quat;
                // ++traj_idx;
            }
            
        }



        pose_target.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose_target);
        path_vis_pub.publish(path_msg);
        trajectory_vis_pub.publish(trajectory_msg);
        nav_msgs::Path mpc_trajectory_msg = wrapPathMsg(mpc_trajectory);
        mpc_trajectory_vis_pub.publish(mpc_trajectory_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
