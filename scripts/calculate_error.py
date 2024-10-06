#!/usr/bin/env python3

import rospy
import sys
import argparse
import numpy as np
from tf2_ros import Buffer, TransformListener
import matplotlib.pyplot as plt

rospy.sleep(7)  # To ensure that it starts only when the robot starts moving

# Initialize global variables
sum_errors = 0.0
num_errors = 0

# List to store errors for plotting
error_list = []

class TransformHandler:
    def __init__(self, gt_frame, est_frame, max_time_between=0.5):  # 500ms
        self.gt_frame = gt_frame
        self.est_frame = est_frame
        self.frames = [gt_frame, est_frame]

        self.tf_buffer = Buffer(cache_time=rospy.Duration(max_time_between))
        self.tf_listener = TransformListener(self.tf_buffer)

    def get_transform(self, fixed_frame, target_frame):
        # Caller should handle the exceptions
        return self.tf_buffer.lookup_transform(target_frame, fixed_frame, rospy.Time(0), rospy.Duration(5.0))  # 5 second timeout

def get_errors(transform):
    tr = transform.transform.translation
    return np.linalg.norm([tr.x, tr.y])

def update_error_plot(line_error, ax_error):
    line_error.set_xdata(range(len(error_list)))
    line_error.set_ydata(error_list)

    # Adjust plot limits dynamically
    ax_error.relim()
    ax_error.autoscale_view()

    # Redraw the plot
    plt.figure(ax_error.figure.number)
    plt.draw()
    plt.pause(0.01)  # Short pause to allow the plot to update

def main():
    global sum_errors, num_errors

    parser = argparse.ArgumentParser()
    parser.add_argument('--gt_frame', help='The child frame of the GT transform', default='mocap_laser_link')
    parser.add_argument('--est_frame', help='The child frame of the estimation transform', default='base_scan')

    args, unknown = parser.parse_known_args()

    gt_frame = args.gt_frame
    est_frame = args.est_frame

    rospy.init_node('error_evaluation_node')

    if rospy.rostime.is_wallclock():
        rospy.logfatal('You should be using simulated time: rosparam set use_sim_time true')
        sys.exit(10)

    rospy.loginfo('Waiting for clock')

    handler = TransformHandler(gt_frame, est_frame, max_time_between=0.5)  # 500ms

    rospy.loginfo('Listening to frames and computing error, press Ctrl-C to stop')
    sleeper = rospy.Rate(10)  # 10 Hz

    # Setup live plotting for Error
    plt.ion()  # Enable interactive mode for Error Plot
    fig_error, ax_error = plt.subplots()
    line_error, = ax_error.plot([], [], label="Error (mm)", color='green')  # Initialize plot for error

    # Set up Error plot labels and grid
    ax_error.set_title('Error Over Time')
    ax_error.set_xlabel('Messages Received')
    ax_error.set_ylabel('Error (mm)')
    ax_error.grid(True)
    ax_error.legend()

    try:
        while not rospy.is_shutdown():
            try:
                # Get the latest transform
                transform = handler.get_transform(gt_frame, est_frame)
            except (rospy.ROSException, rospy.ROSInterruptException) as e:
                rospy.logwarn(f"Transform lookup failed: {e}")
                sleeper.sleep()
                continue  # Skip to the next iteration

            # Calculate Euclidean error
            eucl = get_errors(transform)
            sum_errors += eucl
            num_errors += 1
            error_list.append(eucl * 1e3)  # Error in mm

            # Update the error plot live
            update_error_plot(line_error, ax_error)

            # Sleep to maintain the loop rate
            sleeper.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received. Shutting down.")
    finally:
        # After shutdown, print average error and show final plot
        if num_errors > 0:
            avg_error = (sum_errors / num_errors) * 1e3  # Convert to mm
            print(f'Average error (in mm): {avg_error:.2f}')
        else:
            print('No error data collected.')

        # After shutdown, plot the final error list
        plt.ioff()  # Disable interactive mode
        plt.show()  # Show all figures

        sys.exit(0)

if __name__ == '__main__':
    main()
