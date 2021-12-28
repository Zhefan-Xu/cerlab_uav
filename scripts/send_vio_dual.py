#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf


odom_pub = rospy.Publisher("mavros/odometry/out", Odometry, queue_size=10)
def callback(odom):
	odom_send = odom
	odom_send.header.frame_id = "odom"
	odom_send.child_frame_id = "base_link"
	odom_pub.publish(odom_send)

	#br = tf.TransformBroadcaster()
	#position = odom.pose.pose.position

	#orientation = odom.pose.pose.orientation
	#br.sendTransform((position.x, position.y,position.z), (orientation.x, orientation.y, orientation.z, orientation.w), rospy.Time.now(), "base_link", "odom")

def main():
	rospy.init_node('t265_vio_sender', anonymous=False)

	odom_sub = rospy.Subscriber("/t265/odom/sample", Odometry, callback)
	rospy.spin()

if __name__ == "__main__":
	main()
