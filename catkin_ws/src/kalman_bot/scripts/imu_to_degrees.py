#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import tf
import numpy as np


def imu_callback(data):
    quaternion = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w
    )
    
    try:
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw_radians = euler[2]
        yaw_degrees = yaw_radians * (180.0 / np.pi)
        yaw_pub.publish(yaw_degrees)
        rospy.loginfo(f"Yaw (degrees): {yaw_degrees}")
    except Exception as e:
        rospy.logerr(f"Error in imu_callback: {str(e)}")

def imu_to_degrees():
    rospy.init_node('imu_to_degrees', anonymous=True)

    rospy.Subscriber('/imu/noisy_data', Imu, imu_callback)
    
    global yaw_pub
    yaw_pub = rospy.Publisher('/imu/yaw_degrees', Float64, queue_size=10)

    rospy.loginfo("Node 'imu_to_degrees' started and listening for IMU data.")

    rospy.spin()

if __name__ == '__main__':
    try:
        imu_to_degrees()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted")
