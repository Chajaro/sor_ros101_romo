#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from ros_tutorial.msg import Cylinder

# Global variables to store density and volume data
density = 0.0
volume = 0.0
density_found = False
volume_found = False

# Callback for density topic
def density_callback(data):
    global density, density_found
    density = data.data
    density_found = True
    rospy.loginfo(f"Received density: {density}")

# Callback for cylinder volume topic
def volume_callback(data):
    global volume, volume_found
    volume = data.volume  # Access the `volume` field of Cylinder message
    volume_found = True
    rospy.loginfo(f"Received volume: {volume}")

# Function to calculate cylinder weight and publish
def calculate_and_publish(pub):
    if density_found and volume_found:
        cylinder_weight = volume * density
        rospy.loginfo(f"Calculated Cylinder Weight: {cylinder_weight} (Volume: {volume}, Density: {density})")
        pub.publish(cylinder_weight)
    else:
        rospy.loginfo("Waiting for both density and volume data...")

# Main function
if __name__ == "__main__":
    rospy.init_node("cylinder_weight_calculator", anonymous=True)

    # Subscriber for density topic
    rospy.Subscriber("/density", Float64, density_callback)

    # Subscriber for cylinder volume topic
    rospy.Subscriber("/cylinder", Cylinder, volume_callback)

    # Publisher for cylinder weight
    pub = rospy.Publisher("/cylinder_weight", Float64, queue_size=10)

    # Loop rate
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        calculate_and_publish(pub)
        rate.sleep()
