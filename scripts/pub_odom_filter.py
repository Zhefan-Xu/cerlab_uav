import rospy
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from collections import deque
import math

node_to_kill = offboard_node

bbx_xmin = 0
bbx_xmax = 100
bbx_ymin = 0
bbx_ymax = 100
bbx_zmin = 0
bbx_zmax = 100
dist_thres = 20
drift_thres = 20


class OdomFilter:
    def __init__(self):
        self.odom_t265 = None
        self.pose_t265 = None
        self.queue = deque([])
        self.pub_mavros = rospy.Publisher("/mavros/odometry/out", Odometry, queue_size=10)
        self.sub_t265 = rospy.Subscriber("/t265/odom/sample", Odometry, self.callback_t265,queue_size=10)
        self.sub_mavros = rospy.Subscriber("/mavros/local_position/pose",PoseStamped, self.callback_mavros)

        

    def callback_t265(self, odom):  
        self.odom_t265 = odom
        self.odom_t265.header.frame_id = "odom"
        self.odom_t265.child_frame_id = "base_link"
        self.pose_t265 = self.odom_t265.pose.pose.position
        

    def callback_mavros(self, pose):
        self.pose_mavros = pose.pose.position
        if self.filterPassed():
            self.pub_mavros.publish(self.odom_t265)
            if len(self.queue > 2):
                self.queue.popleft()
            self.queue.append(self.pose_t265)
        else:
            self.switch2LandMode()


    def filterPassed(self):
        if not self.pose_t265:
            rospy.logwarn("Not receiving T265 value")
            return False

        # filter 1: bbx x <[a,b]
        if self.pose_t265.x < bbx_xmin or  self.pose_t265.x > bbx_xmax or self.pose_t265.y < bbx_ymin  or  self.pose_t265.y > bbx_ymax or self.pose_t265.z < bbx_zmin  or  self.pose_t265.z > bbx_zmax:
            rospy.logerr("T265 returned invalid value -- exceeds bounding box limitation")
            return False

        # filter 2: t265 != mavros
        if not self.pose_mavros or math.dist(self.pose_t265 - self.pose_mavros) > dist_thres:
            rospy.logerr("T265 returned invalid value -- does not match with /mavros/local_position/pose")
            return False
            
        # filter 3: x_t+1 - x_t
        if not self.queue.empty() and math.dist(self.queue[0] - self.queue[-1]) > drift_thes:
            rospy.logerr("T265 returned invalid value -- drifts too much from previous position")
            return False
        
        return True

    def switch2LandMode():
        rospy.wait_for_service("mavros/set_mode")
        try:
            flightModeService = rospy.ServiceProxy("mavros/set_mode", SetMode)
            os.system("rosnode kill " + node_to_kill)
            flightModeService(custom_mode="AUTO.LAND")
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Autoland Mode could not be set." % e)

        
	
if __name__ == "__main__":
    rospy.init_node('t265_vio_filter', anonymous=False)
    OdomFilter()
    rospy.spin()