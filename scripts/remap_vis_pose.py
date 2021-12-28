#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped



pose_vis_pub = rospy.Publisher("pose_visualization", PoseStamped, queue_size=10)
def callback(pose):
	pose_send = pose
	pose_vis_pub.publish(pose_send)

def main():
	rospy.init_node('pose_vis_node', anonymous=False)
	pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback)
	rospy.spin()

if __name__ == "__main__":
	main()