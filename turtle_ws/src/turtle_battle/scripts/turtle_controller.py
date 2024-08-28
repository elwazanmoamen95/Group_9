#!/usr/bin/env python3

import rospy
import subprocess
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8 , String
from turtlesim.srv import SetPen
import sys, select, termios, tty
import os

# File paths
# Construct file paths relative to the current script's location
base_dir = os.path.dirname(os.path.abspath(__file__))
file_path1 = os.path.join(base_dir, "../files/file.txt")
file_path2 = os.path.join(base_dir, "../files/file_start.txt")

# Read start file
start = 0
try:
    with open(file_path2, 'r') as op:
        start = int(op.read().strip())
except IOError as e:
    print(f"Failed to read file: {e}")
    sys.exit(1)

if start:
    print("Game started")
    exit(0)

# Read turtle number
try:
    with open(file_path1, 'r') as op:
        turtle_number = int(op.read().strip())
except IOError as e:
    print(f"Failed to read file: {e}")
    sys.exit(1)

curr_turtle = 1
choose = True
new = False

print('Control an existing turtle? (Y/n)')
ans = input().strip().lower()

if ans == 'y':
    print('Enter turtle number:')
    try:
        input_tur = int(input())
        if 1 <= input_tur <= turtle_number:
            curr_turtle = input_tur
            choose = False
        else:
            print("Automatic selection")
            curr_turtle = turtle_number
            choose = False
    except ValueError:
        print("Invalid input. Using automatic selection.")
        curr_turtle = turtle_number
        choose = False

if choose:
    while choose:
        turtle_number += 1
        command_spawn = f"rosservice call /spawn '{{x: {turtle_number}.0, y: 1.0, theta: 0.0, name: \"turtle{turtle_number}\"}}'"
        command_setpen = f"/turtle{turtle_number}/set_pen"

        try:
            # Use subprocess to call rosservice command
            subprocess.run(command_spawn, shell=True, check=True, capture_output=True, text=True)
            
            # Use rospy.ServiceProxy for setting pen
            rospy.wait_for_service(command_setpen)
            set_pen = rospy.ServiceProxy(command_setpen, SetPen)
            set_pen(0, 0, 0, 0, 1)

            curr_turtle = turtle_number
            new = True
            try:
                with open(file_path1, 'w') as op:
                    op.write(f"{turtle_number}")
            except IOError as e:
                print(f"Failed to write file: {e}")
            choose = False
        except subprocess.CalledProcessError as e:
            print(f"Failed to spawn turtle: {e}")
            turtle_number -= 1

# Terminal settings for non-blocking key press
oldsettings = termios.tcgetattr(sys.stdin)

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, oldsettings)
    return key

def start(data):
    if data.data in [0, 1]:
        try:
            with open(file_path2, 'w') as op:
                op.write(f"{data.data}")
        except IOError as e:
            print(f"Failed to write file: {e}")

# Initialize ROS node
node_name = f"turtle_controller{curr_turtle}"
rospy.init_node(node_name)
turtle_name = f"turtle{curr_turtle}"

# ROS Publishers
attack_pub = rospy.Publisher('/turtle_attack', String, queue_size=10)
cmd_vel_pub = rospy.Publisher(f"/{turtle_name}/cmd_vel", Twist, queue_size=10)

# ROS Subscriber
start_sub = rospy.Subscriber('/game_started', Int8, start)

twist = Twist()

print(f'Control {turtle_name} using WASD, Q to attack.')

while True:
    key = get_key()
    if key == 'w':
        twist.linear.x = 1.0
        twist.angular.z = 0.0
    elif key == 's':
        twist.linear.x = -1.0
        twist.angular.z = 0.0
    elif key == 'a':
        twist.linear.x = 0.0
        twist.angular.z = 3.1416 / 2
    elif key == 'd':
        twist.linear.x = 0.0
        twist.angular.z = -3.1416 / 2
    elif key == 'q':
        attack_pub.publish(turtle_name)
        print(f"{turtle_name} attacks!")
        continue
    elif key == '\x03':  # Ctrl+C
        break
    else:
        continue
    
    cmd_vel_pub.publish(twist)