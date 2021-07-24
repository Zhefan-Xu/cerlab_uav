#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

path = Path()
total_distance = 0

def distance(p1, p2):
	return ((p1.position.x-p2.position.x)**2 + (p1.position.y-p2.position.y)**2 + (p1.position.z-p2.position.z)**2)**0.5

def odom_cb(data):
    global path
    global total_distance
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    distance_to_last = 0
    if (len(path.poses) != 0):
    	last_pose= path.poses[-1]
    	distance_to_last = distance(last_pose.pose, pose.pose)
    if (distance_to_last > 0.2 or len(path.poses) == 0):
    	path.poses.append(pose)
    	total_distance += distance_to_last

    # path.poses.append(data.pose.pose)
    del pose

    
def main():
	rospy.init_node('path_node')

	# odom_sub = rospy.Subscriber('/t265/odom/sample', Odometry, odom_cb)
	odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, odom_cb)
	path_pub = rospy.Publisher('/path_sofar', Path, queue_size=0)
	rate = rospy.Rate(0.1)
	while not rospy.is_shutdown():
		path_pub.publish(path)
		# rospy.loginfo(len(path.poses))
		# print("Total distance traveled is {.2f}m".format(total_distance))
	rospy.spin()
if __name__ == '__main__':
    main()