#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

def main():
    rospy.init_node('publish_ground_truth')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub = rospy.Publisher('/ground_truth_pose', PoseWithCovarianceStamped, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        try:
            # Lookup the transformation from 'map' to 'mocap_laser_link'
            trans = tfBuffer.lookup_transform('map', 'mocap_laser_link', rospy.Time(0))

            # Create a PoseWithCovarianceStamped message
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = trans.header.stamp
            pose_msg.header.frame_id = 'map'

            # Set position
            pose_msg.pose.pose.position = trans.transform.translation

            pose_msg.pose.pose.orientation = trans.transform.rotation

            pose_msg.pose.covariance = [
                0, 0, 0, 0, 0, 0,   # x
                0, 0, 0, 0, 0, 0,   # y
                0, 0, 0, 0, 0, 0,   # z
                0, 0, 0, 0, 0, 0,   # roll
                0, 0, 0, 0, 0, 0,   # pitch
                0, 0, 0, 0, 0, 0    # yaw
            ]

            # Publish the pose
            pub.publish(pose_msg)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(5, "Transformation from 'map' to 'mocap_laser_link' not available.")
            pass
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
