#!/usr/bin/env python3

import rospy
import sys
import argparse
import numpy as np
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

# Initialize global variables
sum_errors = 0.0
sum_uncertainty = 0.0
num_errors = 0
num_uncertainty = 0
previous_covariance_1 = 0
previous_covariance_2 = 0
last_covariance_published = 0

# Lists to store errors and uncertainties for plotting
error_list = []
uncertainty_list = []

class TransformHandler:
    def __init__(self, gt_frame, est_frame, max_time_between=0.5):  # 500ms
        self.gt_frame = gt_frame
        self.est_frame = est_frame
        self.frames = [gt_frame, est_frame]

        self.tf_buffer = Buffer(cache_time=rospy.Duration(max_time_between))
        self.tf_listener = TransformListener(self.tf_buffer)

    def get_transform(self, fixed_frame, target_frame):
        # Caller should handle the exceptions
        return self.tf_buffer.lookup_transform(target_frame, fixed_frame, rospy.Time(0), rospy.Duration(5.0))  # 1 second timeout

def get_errors(transform):
    tr = transform.transform.translation
    return np.linalg.norm([tr.x, tr.y])

def get_covariance(data):
    # Access the covariance matrix
    covariance_matrix = data.pose.covariance

    # Extract the relevant variance element for the x dimension (index 0)
    var_x = covariance_matrix[0]  # Diagonal element for x

    # Calculate the standard deviation (uncertainty) for x
    uncertainty_x = np.sqrt(var_x)
    return uncertainty_x

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

def update_uncertainty_plot(line_uncertainty, ax_uncertainty):
    line_uncertainty.set_xdata(range(len(uncertainty_list)))
    line_uncertainty.set_ydata(uncertainty_list)

    # Adjust plot limits dynamically
    ax_uncertainty.relim()
    ax_uncertainty.autoscale_view()

    # Redraw the plot
    plt.figure(ax_uncertainty.figure.number)
    plt.draw()
    plt.pause(0.01)  # Short pause to allow the plot to update

def main():
    global sum_errors, num_errors, sum_uncertainty, num_uncertainty, previous_covariance_1, previous_covariance_2, last_covariance_published

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
    rospy.sleep(0.1)  # Increased sleep time for better stability

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

    # Setup live plotting for Uncertainty
    fig_uncertainty, ax_uncertainty = plt.subplots()
    line_uncertainty, = ax_uncertainty.plot([], [], label="Uncertainty (mm)", color='red')  # Initialize plot for uncertainty

    # Set up Uncertainty plot labels and grid
    ax_uncertainty.set_title('Uncertainty Over Time')
    ax_uncertainty.set_xlabel('Messages Received')
    ax_uncertainty.set_ylabel('Uncertainty (mm)')
    ax_uncertainty.grid(True)
    ax_uncertainty.legend()

    try:
        while not rospy.is_shutdown():
            try:
                # Wait for the next odometry message
                odom_msg = rospy.wait_for_message('/odometry/filtered', Odometry, timeout=1.0)
            except rospy.ROSException as e:
                rospy.logwarn(f"Failed to receive odometry message: {e}")
                sleeper.sleep()
                continue  # Skip to the next iteration

            # Extract covariance and compute uncertainty
            uncertainty = get_covariance(odom_msg)
            sum_uncertainty += uncertainty
            num_uncertainty += 1
            if previous_covariance_2 <= previous_covariance_1 and previous_covariance_1 >= uncertainty:
                uncertainty_list.append(previous_covariance_1 * 1e3)  # Uncertainty in mm
                last_covariance_published = previous_covariance_1
            else:
                uncertainty_list.append(last_covariance_published * 1e3)
            previous_covariance_2 = previous_covariance_1
            previous_covariance_1 = uncertainty

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

            # Update the plots live
            update_error_plot(line_error, ax_error)
            update_uncertainty_plot(line_uncertainty, ax_uncertainty)

            # Sleep to maintain the loop rate
            sleeper.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received. Shutting down.")
    finally:
        # After shutdown, print averages and show final plots
        if num_errors > 0:
            avg_error = (sum_errors / num_errors) * 1e3  # Convert to mm
            print(f'Average error (in mm): {avg_error:.2f}')
        else:
            print('No error data collected.')

        if num_uncertainty > 0:
            avg_uncertainty = (sum_uncertainty / num_uncertainty) * 1e3  # Convert to mm
            print(f'Average uncertainty (in mm): {avg_uncertainty:.2f}')
        else:
            print('No uncertainty data collected.')

        # After shutdown, plot the final error and uncertainty lists
        plt.ioff()  # Disable interactive mode
        plt.show()  # Show all figures

        sys.exit(0)

if __name__ == '__main__':
    main()
