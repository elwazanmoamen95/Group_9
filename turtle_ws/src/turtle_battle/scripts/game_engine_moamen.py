#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import String
import math
import subprocess

class TurtleBattle:
    def __init__(self):
        rospy.init_node('game_engine', anonymous=True)
        self.turtles = {}  # Dictionary to store turtle states (health, attacks, pose)
        self.pub = rospy.Publisher('/battle_status', String, queue_size=10)
        self.subscribe_turtles()

    def subscribe_turtles(self):
        # Get list of all active topics
        topics = rospy.get_published_topics()
        for topic, _ in topics:
            if '/pose' in topic:  # Filter for pose topics (e.g., /turtleX/pose)
                turtle_name = topic.split('/')[1]
                if turtle_name not in self.turtles:
                    # Initialize turtle state
                    self.turtles[turtle_name] = {'health': 100, 'attacks': 10, 'pose': None}
                    rospy.Subscriber(topic, Pose, self.update_turtle_pose, callback_args=turtle_name)
                    rospy.loginfo(f"Connected to {turtle_name}")

    def update_turtle_pose(self, data, turtle_name):
        self.turtles[turtle_name]['pose'] = data

    def calculate_damage(self):
        turtle_names = list(self.turtles.keys())
        
        #  i = --> int for attack from controller node and updated --<
            turtle1 = turtle_names[i]
            for j in range(len(turtle_names)):
                if j!=i:
                    turtle2 = turtle_names[j]
                
                if self.turtles[turtle1]['pose'] and self.turtles[turtle2]['pose']:
                    if self.attack_within_distance(self.turtles[turtle1]['pose'], self.turtles[turtle2]['pose'])
                        self.process_attack(self,turtle1,turtle2)

    def attack_within_distance(self, pose1, pose2):
        distance = math.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)
        return distance <= 2.0

    def process_attack(self,attacker,target):
        if self.turtles[attacker]['attacks'] > 0:
            self.turtles[target]['health'] -= 50
            self.turtles[attacker]['attacks'] -= 1
            rospy.loginfo(f"{attacker} attacked {turtle2}!")
            if self.turtles[target]['health'] <= 0:
                self.turtles.remove(target)

                # check issues
                # command = f"rosservice call /kill '{{ name: \"{target}\"}}'"
                # subprocess.run(command, shell=True, check=True, capture_output=True, text=True)

    def check_game_over(self):
        all_attacks_used = all(self.turtles[t]['attacks'] == 0 for t in self.turtles)
        any_turtle_dead = any(self.turtles[t]['health'] <= 0 for t in self.turtles)

        if all_attacks_used or any_turtle_dead:
            # Determine the winner
            max_health = max(self.turtles[t]['health'] for t in self.turtles)
            winners = [t for t in self.turtles if self.turtles[t]['health'] == max_health]

            if len(winners) == 1:
                winner = f"{winners[0]} wins!"
            else:
                winner = "It's a draw!"

            # Publish the game result
            self.pub.publish(winner)
            rospy.loginfo(winner)
            rospy.signal_shutdown("Game Over")

    def start_game(self):
        rate = rospy.Rate(10)  # Set a loop rate of 10 Hz
        while not rospy.is_shutdown():
            self.calculate_damage()
            self.check_game_over()
            rate.sleep()

if __name__ == '__main__':
    game = TurtleBattle()
    game.start_game()
