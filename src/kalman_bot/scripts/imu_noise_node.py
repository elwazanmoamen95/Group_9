#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import numpy as np

def add_noise(imu_data):
    noise_mean = 0.0
    noise_stddev = 0.005
    
    noise = np.random.normal(noise_mean, noise_stddev, 4)  # 4 for x, y, z, w
    
    imu_data.orientation.x += noise[0]
    imu_data.orientation.y += noise[1]
    imu_data.orientation.z += noise[2]
    imu_data.orientation.w += noise[3]
    
    return imu_data

def callback(data):
    noisy_data = add_noise(data)
    pub.publish(noisy_data)

if __name__ == '__main__':
    rospy.init_node('imu_noise_node', anonymous=True)
    rospy.Subscriber('/imu', Imu, callback)
    pub = rospy.Publisher('/imu/noisy_data', Imu, queue_size=10)
    rospy.spin()
