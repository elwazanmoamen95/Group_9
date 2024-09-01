#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class KalmanFilter:
    def __init__(self):
        self.x = 0.0  # Initial estimate
        self.P = 1.0  # Initial estimation error covariance
        self.Q = 0.0001  # Process noise covariance
        self.R = 10.0  # Measurement noise covariance
        self.K = 0.0  # Kalman gain

    def update(self, z):
        # Predict
        self.P = self.P + self.Q

        # Update
        self.K = self.P / (self.P + self.R)
        self.x = self.x + self.K * (z - self.x)
        self.P = (1 - self.K) * self.P

        return self.x

def yaw_callback(data):
    filtered_yaw = kalman_filter.update(data.data)
    filtered_yaw_pub.publish(filtered_yaw)
    rospy.loginfo(f"Filtered Yaw (degrees): {filtered_yaw}")

def kalman_filter_node():
    rospy.init_node('kalman_filter_node', anonymous=True)
    
    global kalman_filter
    kalman_filter = KalmanFilter()
    
    rospy.Subscriber('/imu/yaw_degrees', Float64, yaw_callback)
    
    global filtered_yaw_pub
    filtered_yaw_pub = rospy.Publisher('/imu/yaw_kalman', Float64, queue_size=10)
    
    rospy.loginfo("Kalman Filter Node started and listening for Yaw angle data.")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        kalman_filter_node()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted")
