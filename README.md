
# Task 10.1 - Kalman Bot

This project involves deploying a TurtleBot3 simulation in Gazebo and implementing an IMU data processing pipeline that includes quaternion to degree conversion, Gaussian noise addition, and a 1D Kalman Filter. The goal is to filter the Yaw angle data from the IMU and visualize both the noisy and filtered data, providing an enhanced understanding of sensor data in robotics applications.

## Table of Contents
- [Installation](#installation)
- [Setup and Configuration](#setup-and-configuration)
- [Running the Simulation](#running-the-simulation)
- [Adding Noise to IMU Data](#adding-noise-to-imu-data)
- [IMU Data Processing](#imu-data-processing)
- [Kalman Filter Implementation](#kalman-filter-implementation)
- [Visualization](#visualization)
- [Observations](#observations)
- [Conclusion](#conclusion)

## Installation

To get started with the Kalman Bot project, you need to have ROS Noetic installed on Ubuntu 20.04 along with the following ROS packages:

- `turtlebot3`
- `turtlebot3_simulations`
- `rqt_multiplot`

Clone the repository and install the necessary dependencies:

`bash`
`cd ~/catkin_ws/src`
`git clone https://github.com/elwazanmoamen95/Group_9.git`
`cd ~/catkin_ws`
`catkin_make`
`source devel/setup.bash`

## Setup and Configuration

1.  **Set the TurtleBot3 Model:** Before launching the TurtleBot3 simulation, set the TurtleBot3 model environment variable. This project uses the `burger` model.
    
    `export TURTLEBOT3_MODEL=burger` 
    
2.  **Launch Gazebo and RViz:** Start the simulation environment with Gazebo and RViz to visualize the TurtleBot3:
    

    
    `roslaunch turtlebot3_gazebo turtlebot3_world.launch
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch` 
    

## Running the Simulation

To control the TurtleBot3 in the Gazebo environment, use the keyboard teleop node:


`roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` 

Move the robot around to generate IMU and LiDAR data for further processing.

## Adding Noise to IMU Data

To simulate real-world sensor inaccuracies, Gaussian noise is added to the IMU data using the `imu_noise_node.py` script. This node modifies the orientation data by adding random noise and then normalizes the quaternion.

**Running the IMU Noise Node:**


`rosrun kalman_bot imu_noise_node.py` 

This node subscribes to `/imu` and publishes the noisy IMU data to `/imu/noisy_data`.

## IMU Data Processing

The IMU data is processed to convert quaternion orientation to Euler angles (degrees). The script `imu_to_degrees.py` is used for this conversion.

**Running the IMU Processing Node:**

`rosrun kalman_bot imu_to_degrees.py` 

This node subscribes to `/imu/noisy_data` and publishes the yaw in degrees to `/imu/yaw_degrees`.




## Kalman Filter Implementation

The 1D Kalman Filter is implemented to filter the noisy yaw data from the IMU.

**Running the Kalman Filter Node:**


`rosrun kalman_bot kalman_filter.py` 

The filter subscribes to `/imu/yaw_degrees` and publishes the filtered yaw data to `/imu_yaw_kalman`.

## Visualization

Use `rqt_multiplot` to visualize both noisy and filtered IMU yaw data.

**Launch rqt_multiplot:**



`rqt_multiplot` 

**Add the Topics:**

-   Add `/imu/yaw_degrees` to visualize noisy data.
-   Add `/imu_yaw_kalman` to visualize the filtered data.

The plot should show a comparison between the noisy and filtered yaw data, demonstrating the effectiveness of the Kalman Filter.

## Observations

-   **IMU Noise Addition:** The addition of Gaussian noise effectively simulated real-world sensor inaccuracies.
-   **Kalman Filter Performance:** The filter successfully smoothed out the noise in the yaw data. The filtered data closely followed the actual motion while eliminating random fluctuations.

## Conclusion

The Kalman Bot project demonstrates the practical application of IMU data processing and filtering in a robotic simulation environment. Through this project, we learned how to handle sensor data in ROS, apply a Kalman Filter for noise reduction, and visualize the results effectively.
