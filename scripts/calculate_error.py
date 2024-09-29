#!/usr/bin/env python3

import rospy
import sys
import argparse
import numpy
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class TransformHandler():

    def __init__(self, gt_frame, est_frame, max_time_between=0.01):
        self.gt_frame = gt_frame
        self.est_frame = est_frame
        self.frames = [gt_frame, est_frame]

        self.tf_buffer = Buffer(cache_time=rospy.Duration(max_time_between))
        self.__tf_listener = TransformListener(self.tf_buffer)

    def get_transform(self, fixed_frame, target_frame):
        # caller should handle the exceptions
        return self.tf_buffer.lookup_transform(target_frame, fixed_frame, rospy.Time(0))


def get_errors(transform):
    tr = transform.transform.translation
    return numpy.linalg.norm([tr.x, tr.y])

def odometry_callback(msg):
    # Extract position covariance from the Odometry message (3x3 diagonal values)
    cov_x = msg.pose.covariance[0]   # Covariance for X
    cov_y = msg.pose.covariance[7]   # Covariance for Y
    cov_z = msg.pose.covariance[14]  # Covariance for Z

    # Combine covariances (you can modify this if necessary)
    combined_covariance = numpy.sqrt(cov_x + cov_y)  # Standard deviation from covariance sum

    # Append to covariance list
    covariance_list.append(combined_covariance)


parser = argparse.ArgumentParser()
parser.add_argument('--gt_frame', help='The child frame of the GT transform', default='mocap_laser_link')
parser.add_argument('--est_frame', help='The child frame of the estimation transform', default='base_scan')

args, unknown = parser.parse_known_args()

gt_frame = args.gt_frame
est_frame = args.est_frame

rospy.init_node('evaluation_node')

# Subscribe to the Odometry topic
covariance_list = []
rospy.Subscriber('/odometry/filtered', Odometry, odometry_callback)

if rospy.rostime.is_wallclock():
    rospy.logfatal('You should be using simulated time: rosparam set use_sim_time true')
    sys.exit(1)

rospy.loginfo('Waiting for clock')
rospy.sleep(0.00001)

handler = TransformHandler(gt_frame, est_frame, max_time_between=20)  # 500ms

rospy.loginfo('Listening to frames and computing error, press Ctrl-C to stop')
sleeper = rospy.Rate(1000)
sum_errors = 0.0
num_errors = 0.0

# List to store errors for plotting
error_list = []

try:
    while not rospy.is_shutdown():
        try:
            t = handler.get_transform(gt_frame, est_frame)
        except Exception as e:
            rospy.logwarn(e)
        else:
            # Calculate Euclidean error
            eucl = get_errors(t)
            sum_errors += eucl
            num_errors += 1
            error_list.append(eucl * 1e3)  # Error in mm

        try:
            sleeper.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(e)
        except rospy.exceptions.ROSInterruptException:
            print('Average error (in mm): {:.2f}'.format(sum_errors / num_errors * 1e3))

            # After shutdown, plot the errors and covariances
            if error_list and covariance_list:
                plt.figure()

                # Plot error
                plt.plot(error_list, label="Error", color='green')

                # Plot uncertainty (from odometry covariance data)
                plt.plot(covariance_list, label="Uncertainty (Standard Deviation)", color='red')

                plt.title('Error Fluctuation Along the Path and its Uncertainty')
                plt.xlabel('Messages Received')
                plt.ylabel('Error (m)')
                plt.grid(True)
                plt.legend()
                plt.show()

            exit(0)

except rospy.exceptions.ROSInterruptException:
    pass
