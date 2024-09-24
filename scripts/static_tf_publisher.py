#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg

def main():
    rospy.init_node('tf2_mocap_to_odom')

    br = tf2_ros.TransformBroadcaster()
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
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        exit()
