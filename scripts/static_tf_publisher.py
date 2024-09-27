#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def main():
    rospy.init_node('tf2_mocap_to_odom')

    br = tf2_ros.TransformBroadcaster()

    # Publisher to publish corrected odometry
    odom_pub = rospy.Publisher('/odom_corrected', Odometry, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "mocap"
        t.child_frame_id = "map"

        # Set translation
        t.transform.translation.x = 0.935
        t.transform.translation.y = 1.34
        t.transform.translation.z = -0.023

        # Set rotation (quaternion)
        t.transform.rotation.x = 0.001
        t.transform.rotation.y = -0.003
        t.transform.rotation.z = 0.737
        t.transform.rotation.w = 0.676

        br.sendTransform(t)

        # Create a new odometry message to "induce" ground-truth data into odometry
        odom_msg = Odometry()

        # Update odometry
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"

        # Set the position using ground-truth data (mocap)
        odom_msg.pose.pose.position.x = t.transform.translation.x
        odom_msg.pose.pose.position.y = t.transform.translation.y
        odom_msg.pose.pose.position.z = t.transform.translation.z

        # Set the orientation (from mocap data)
        odom_msg.pose.pose.orientation = t.transform.rotation

        # Reduce the covariance over time (simulate correction)
        # The covariance matrix reduces as we get more confident about the position
        odom_msg.pose.covariance = [
            1e-9, 0, 0, 0, 0, 0,
            0, 1e-9, 0, 0, 0, 0,
            0, 0, 1e-9, 0, 0, 0,
            0, 0, 0, 1e-3, 0, 0,
            0, 0, 0, 0, 1e-3, 0,
            0, 0, 0, 0, 0, 1e-9  # Confidence in yaw as well
        ]

        # Publish the corrected odometry
        odom_pub.publish(odom_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        exit()
