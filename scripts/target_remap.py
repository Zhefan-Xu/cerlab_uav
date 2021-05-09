#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

# This node is to make sure the target is published at a normal rate (reduce the latency problem)

target_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
target_pose = PoseStamped()

def callback(target_pose_msg):
	target_pose.pose = target_pose_msg.pose
	target_pose.header.stamp = rospy.Time.now()
	target_pose.header.frame_id = "map"
	# print("here")


	# br = tf.TransformBroadcaster()
	# position = odom.pose.pose.position

	# orientation = odom.pose.pose.orientation
	# br.sendTransform((position.x, position.y,position.z), (orientation.x, orientation.y, orientation.z, orientation.w), rospy.Time.now(), "base_link", "odom")

def main():
	rospy.init_node("target_remap", anonymous=False)

	target_sub = rospy.Subscriber("target", PoseStamped, callback) 
	rate = rospy.Rate(20)
	while (not rospy.is_shutdown()):
		target_pub.publish(target_pose)

		rate.sleep()

if __name__ == "__main__":
	main()