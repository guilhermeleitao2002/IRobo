#!/usr/bin/env python3

import rospy
import sys
import argparse
import numpy as np
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

rospy.sleep(7)  # To ensure that it starts only when the robot starts moving

# Initialize global variables
sum_uncertainty = 0.0
num_uncertainty = 0
previous_covariance_1 = 0
previous_covariance_2 = 0
last_covariance_published = 0

# List to store uncertainties for plotting
uncertainty_list = []

def get_covariance(data):
    # Access the covariance matrix
    covariance_matrix = data.pose.covariance

    # Extract the relevant variance element for the x dimension (index 0)
    var_x = covariance_matrix[0]  # Diagonal element for x

    # Calculate the standard deviation (uncertainty) for x
    uncertainty_x = np.sqrt(var_x)
    return uncertainty_x

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
    global sum_uncertainty, num_uncertainty, previous_covariance_1, previous_covariance_2, last_covariance_published

    rospy.init_node('uncertainty_evaluation_node')

    if rospy.rostime.is_wallclock():
        rospy.logfatal('You should be using simulated time: rosparam set use_sim_time true')
        sys.exit(1)

    rospy.loginfo('Waiting for clock')

    rospy.loginfo('Listening to odometry and computing uncertainty, press Ctrl-C to stop')
    sleeper = rospy.Rate(10)  # 10 Hz

    # Setup live plotting for Uncertainty
    plt.ion()  # Enable interactive mode for Uncertainty Plot
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

            # if previous_covariance_2 <= previous_covariance_1 and previous_covariance_1 >= uncertainty:
            #     uncertainty_list.append(previous_covariance_1)  # Uncertainty in m
            #     last_covariance_published = previous_covariance_1
            # else:
            #     uncertainty_list.append(last_covariance_published * 1e3)
            
            # previous_covariance_2 = previous_covariance_1
            # previous_covariance_1 = uncertainty

            uncertainty_list.append(uncertainty)  # Uncertainty in m

            # Update the uncertainty plot live
            update_uncertainty_plot(line_uncertainty, ax_uncertainty)

            # Sleep to maintain the loop rate
            sleeper.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received. Shutting down.")
    finally:
        # After shutdown, print average uncertainty and show final plot
        if num_uncertainty > 0:
            avg_uncertainty = (sum_uncertainty / num_uncertainty)
            print(f'Average uncertainty (in m): {avg_uncertainty:.2f}')
        else:
            print('No uncertainty data collected.')

        # After shutdown, plot the final uncertainty list
        plt.ioff()  # Disable interactive mode
        plt.show()  # Show all figures

        sys.exit(0)

if __name__ == '__main__':
    main()
