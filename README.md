
# Task 10.1 - Kalman Bot

This project involves deploying a TurtleBot3 simulation in Gazebo and implementing an IMU data processing pipeline that includes quaternion to degree conversion, Gaussian noise addition, and a 1D Kalman Filter. The goal is to filter the Yaw angle data from the IMU and visualize both the noisy and filtered data.

## Table of Contents
- [Installation](#installation)
- [Setup and Configuration](#setup-and-configuration)
- [Running the Simulation](#running-the-simulation)
- [Adding Noise to IMU Data](#adding-noise-to-imu-data)
- [IMU Data Processing](#imu-data-processing)
- [Kalman Filter Implementation](#kalman-filter-implementation)
- [Visualization](#visualization)

## Installation

To get started with the Kalman Bot you need to have the following ROS packages:

- `turtlebot3`
- `turtlebot3_simulations`
- `rqt_multiplot`

Clone the repository:

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

Gaussian noise is added to the IMU data using the `imu_noise_node.py` script to simulate real-world sensor inaccuracies.

**Running the IMU Noise Node:**


`rosrun kalman_bot imu_noise_node.py` 

This node subscribes to `/imu` and publishes the noisy IMU data to `/imu/noisy_data`.

## IMU Data Processing

The IMU data is processed to convert quaternion orientation to degrees. The script `imu_to_degrees.py` is used for this conversion.

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

The plot should show a comparison between the noisy and filtered yaw data.
