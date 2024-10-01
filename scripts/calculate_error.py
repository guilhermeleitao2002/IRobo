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

parser = argparse.ArgumentParser()
parser.add_argument('--gt_frame', help='The child frame of the GT transform', default='mocap_laser_link')
parser.add_argument('--est_frame', help='The child frame of the estimation transform', default='base_scan')

args, unknown = parser.parse_known_args()

gt_frame = args.gt_frame
est_frame = args.est_frame

rospy.init_node('evaluation_node')

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

# Setup live plotting
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots()
line, = ax.plot([], [], label="Error", color='green')  # Initialize plot with empty data

# Set up plot labels and grid
ax.set_title('Error Fluctuation Along the Path and its Uncertainty')
ax.set_xlabel('Messages Received')
ax.set_ylabel('Error (mm)')
ax.grid(True)
ax.legend()

def update_plot():
    """Update the plot with current data in error_list."""
    line.set_xdata(range(len(error_list)))
    line.set_ydata(error_list)
    
    # Adjust plot limits dynamically
    ax.relim()
    ax.autoscale_view()
    
    # Redraw the plot
    plt.draw()
    plt.pause(0.01)  # Short pause to allow the plot to update

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

            # Update the plot live
            update_plot()

        try:
            sleeper.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(e)
        except rospy.exceptions.ROSInterruptException:
            print('Average error (in mm): {:.2f}'.format(sum_errors / num_errors * 1e3))

            # After shutdown, plot the final error list
            plt.ioff()  # Disable interactive mode
            plt.show()  # Show final plot

            exit(0)

except rospy.exceptions.ROSInterruptException:
    pass
