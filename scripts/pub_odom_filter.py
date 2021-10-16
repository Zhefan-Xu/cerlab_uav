#! /usr/bin/env python
import rospy
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from collections import deque
import math

node_to_kill = "offb_node"

bbx_xmin = -4
bbx_xmax = 4
bbx_ymin = -4
bbx_ymax = 4
bbx_zmin = -0.5
bbx_zmax = 2.0
dist_thres = 0.4
vel_thres = 1.5
time_diff = 0.5


class OdomFilter:
    def __init__(self):
        self.odom_t265 = None
        self.pose_t265 = None
        self.queue = deque([])
        self.pub_mavros = rospy.Publisher("/mavros/odometry/out", Odometry, queue_size=10)
        self.sub_t265 = rospy.Subscriber("/t265/odom/sample", Odometry, self.callback_t265,queue_size=10)
        self.sub_mavros = rospy.Subscriber("/mavros/local_position/pose",PoseStamped, self.callback_mavros)
        self.first_time = True
        self.camera_time = rospy.get_time()
        self.initial_time = rospy.get_time()

        

    def callback_t265(self, odom):
        if (self.first_time):
            self.current_time = rospy.get_time()  
            self.first_time = False
        self.camera_time = rospy.get_time()
        self.odom_t265 = odom
        self.odom_t265.header.frame_id = "odom"
        self.odom_t265.child_frame_id = "base_link"
        self.pose_t265 = self.odom_t265.pose.pose.position
        self.vel_t265 = self.odom_t265.twist.twist.linear
        

    def callback_mavros(self, pose):
        self.pose_mavros = pose.pose.position
        if self.filterPassed():
            self.pub_mavros.publish(self.odom_t265)
            if len(self.queue) >= 2:
                self.queue.popleft()
            self.queue.append(self.vel_t265)
        else:
            self.switch2LandMode()

    def filterPassed(self):
        self.current_time = rospy.get_time()
        if self.current_time - self.camera_time > time_diff:
            rospy.logwarn("Not receiving T265 value")
            return False

        # filter 1: bbx x <[a,b]
        if self.pose_t265.x < bbx_xmin or  self.pose_t265.x > bbx_xmax or self.pose_t265.y < bbx_ymin  or  self.pose_t265.y > bbx_ymax or self.pose_t265.z < bbx_zmin  or  self.pose_t265.z > bbx_zmax:
            rospy.logerr("T265 returned invalid value -- exceeds bounding box limitation")
            return False

        # filter 2: t265 != mavros
        # rospy.logerr(self.dist(self.pose_t265, self.pose_mavros))
        if (self.initial_time - self.current_time > 5.0):
            if not self.pose_mavros or self.dist(self.pose_t265, self.pose_mavros) > dist_thres:
                rospy.logerr("T265 returned invalid value -- does not match with /mavros/local_position/pose")
                return False
            
        # filter 3: velocity > thresh
        # rospy.logerr(self.norm(self.vel_t265))
        if self.norm(self.vel_t265) > vel_thres:
            rospy.logerr("T265 returned invalid value -- Velocity too large!!!")
            return False
        
        return True

    def switch2LandMode(self):
        rospy.wait_for_service("mavros/set_mode")
        try:
            flightModeService = rospy.ServiceProxy("mavros/set_mode", SetMode)
            os.system("rosnode kill " + node_to_kill)
            flightModeService(custom_mode="AUTO.LAND")
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Autoland Mode could not be set." % e)
    
    def norm(self, vel):
        vx = vel.x
        vy = vel.y
        vz = vel.z
        return (vx**2 + vy**2 + vz**2)**0.5

    def dist(self, pos1, pos2):
        x1 = pos1.x
        y1 = pos1.y
        z1 = pos1.z

        x2 = pos2.x
        y2 = pos2.y
        z2 = pos2.z
        return ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**0.5
        
	
if __name__ == "__main__":
    rospy.init_node('t265_vio_filter', anonymous=False)
    OdomFilter()
    rospy.spin()