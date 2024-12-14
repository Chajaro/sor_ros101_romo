#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

# Global variables to store the current position
current_x = 0.0
current_y = 0.0
current_theta = 0.0  # Orientation of the turtle

def pose_callback(data):
    """Callback function to update the turtle's position."""
    global current_x, current_y, current_theta
    current_x = data.x
    current_y = data.y
    current_theta = data.theta
    rospy.logdebug(f"Current position: x={current_x}, y={current_y}, theta={current_theta}")

def move_turtle():
    """Main function to guide the turtle to the target."""
    global current_x, current_y, current_theta

    # Initialize the node
    rospy.init_node("turtle_guide_to_xy", anonymous=True)

    # Get target coordinates and threshold from ROS parameters
    target_x = rospy.get_param("target_x", 5.0)
    target_y = rospy.get_param("target_y", 5.0)
    threshold = rospy.get_param("threshold", 0.1)

    # Publisher for turtle velocity commands
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    # Subscriber to track the turtle's position
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    rate = rospy.Rate(10)  # 10 Hz

    vel_msg = Twist()

    while not rospy.is_shutdown():
        # Calculate the distance to the target
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

        # Check if the turtle is within the target threshold
        if distance <= threshold:
            rospy.loginfo("Target Reached")
            break

        # Calculate linear velocity (proportional to distance)
        vel_msg.linear.x = min(1.0, 1.5 * distance)  # Cap speed to 1.0

        # Calculate angular velocity (proportional to angle difference)
        target_angle = math.atan2(target_y - current_y, target_x - current_x)
        angle_difference = target_angle - current_theta
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))  # Normalize angle
        vel_msg.angular.z = 4.0 * angle_difference

        # Log movement info
        rospy.loginfo(f"Moving to target: x={target_x}, y={target_y}")
        rospy.logdebug(f"Distance to target: {distance}, Angle difference: {angle_difference}")

        # Publish the velocity command
        velocity_publisher.publish(vel_msg)

        rate.sleep()

    # Stop the turtle once the target is reached
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

if __name__ == "__main__":
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass
