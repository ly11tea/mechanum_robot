#!/usr/bin/env python
import rospy, tf, tf2_ros, geometry_msgs.msg, nav_msgs.msg

def callback(data, args):
	bc = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()

	t.header.stamp = rospy.Time.now()
	t.header.frame_id = args[0]
	t.child_frame_id = args[1]
	t.transform.translation.x = data.pose.position.x
	t.transform.translation.y = data.pose.position.y
	t.transform.translation.z = data.pose.position.z
	t.transform.rotation = data.pose.orientation
	
	bc.sendTransform(t)

if __name__ == "__main__":
	rospy.init_node("odomtransformer")
	odomPoseInput = rospy.get_param("~odom_pose_input")
	odomInput = rospy.get_param("~odom_frame")
	tfOutput  = rospy.get_param("~tf_frame")
	rospy.Subscriber(odomPoseInput, geometry_msgs.msg.PoseStamped, callback, [odomInput, tfOutput])
	rospy.spin()
