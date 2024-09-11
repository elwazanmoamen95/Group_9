#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


# destination
x_dest = 2.0
y_dest = 3.0
theta_dest = math.pi / 4  

# vars to store current position from odometry topic
x = 0.0
y = 0.0
theta = 0.0


Kp_linear = 0.1
Kp_angular = 0.3


def odometry_callback(data):
    global x, y, theta

    # Get the robot's current position (x, y)
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    # Convert quaternion to yaw angle (theta)
    orientation_q = data.pose.pose.orientation
    siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
    cosy_cosp = 1 - 2 * (orientation_q.y**2 + orientation_q.z**2)
    theta = math.atan2(siny_cosp, cosy_cosp)


# PID controller part
def pid_control(x_dest, y_dest, theta_dest, x, y, theta):
    # calc position errors
    x_error = x_dest - x
    y_error = y_dest - y
    
    # Compute distance to target
    distance_error = math.sqrt(x_error**2 + y_error**2)
    
    # Compute angle to target
    goal_theta = math.atan2(y_error, x_error)
    
    # Compute angular error
    theta_error = goal_theta - theta
    
    # Compute linear and angular velocities
    linear_velocity = Kp_linear * distance_error
    angular_velocity = Kp_angular * theta_error
    
    return linear_velocity, angular_velocity


def main():
    rospy.init_node('pid_controller')
    # Create publisher to send velocity commands
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Subscribe to the /odom topic to get the robot's current position 
    rospy.Subscriber('/odom', Odometry, odometry_callback)

    rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        


        v, omega = pid_control(x_dest,y_dest,theta_dest,x,y,theta)
        

        # publish the velocities
        velocity_msg = Twist()
        velocity_msg.linear.x = v
        velocity_msg.angular.z = omega

        velocity_publisher.publish(velocity_msg)
   
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass