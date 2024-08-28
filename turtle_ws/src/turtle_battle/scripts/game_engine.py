#!/usr/bin/env python3
import rospy
import time
import math
import subprocess
import sys, os
from turtlesim.msg import Pose
from std_msgs.msg import String, Int8
from std_srvs.srv import Empty, EmptyResponse
from turtlesim.srv import Kill

class GameEngine:
    def __init__(self):
        rospy.init_node('game_engine')

        self.turtles = {}  # Dictionary to store turtle states (health, attacks, pose)
        self.start_game_pub = rospy.Publisher('/game_started', Int8, queue_size=1)
        self.start_game_pub.publish(0)  # 0: Game not started, 1: Game started
        self.attack_sub = rospy.Subscriber('/turtle_attack', String, self.attack_callback)
        self.startgame_service = rospy.Service('/start_game', Empty, self.startgame_callback)
        self.game_started = False
        self.rate = rospy.Rate(10)  # Loop at 10 Hz

        self.start_turtlesim_node()  # Start turtlesim node
        self.initialize_files() # create files to 0
        rospy.loginfo("Game Engine initialized.")
        self.game_lobby()

    def initialize_files(self):
        # Construct file paths relative to the current script's location
        base_dir = os.path.dirname(os.path.abspath(__file__))
        file_path1 = os.path.join(base_dir, "../files/file.txt")
        file_path2 = os.path.join(base_dir, "../files/file_start.txt")

        file1 = open(file_path1, 'w') # opens file 1
        file2 = open(file_path2, 'w') # # opens file 2
        
        try:
            file1.write("0")
        except IOError as e:
                print(f"Failed to write file: {e}")
        try:
            file2.write("0")
        except IOError as e:
                print(f"Failed to write file: {e}")

    def game_lobby(self):
        rospy.loginfo("Waiting for players to join...")
        rospy.loginfo(f"Currently joined players: {[f'{turtle_name}' for turtle_name in self.turtles.keys()]}")

        while not self.game_started:
            turtles =  self.get_active_turtles()# Initialize status for all currently active turtles
            for turtle_name in turtles:
                if turtle_name not in self.turtles:
                    self.subscribe_turtles()
                    rospy.loginfo(f"Currently joined players: {[f'{turtle_name}' for turtle_name in self.turtles.keys()]}")
                    rospy.logwarn("Call /start_game service to start game.")

            rospy.sleep(1)  # 1 second delay then check again to prevent overload

    def get_active_turtles(self):
        # Retrieve all topics
        topics = rospy.get_published_topics() # list of [[topic, topic type], ...]
        turtle_topics = [str(topic) for topic, _ in topics if 'pose' in topic] # Filter topics to find turtle pose topics
        turtle_names = [topic.split('/')[1] for topic in turtle_topics] # Extract turtle names from topic names
        # rospy.loginfo(f"Active turtles: {turtle_names}") # remove later just displaying players for testing
        return turtle_names

    def start_turtlesim_node(self):
        rospy.loginfo("Starting turtlesim_node...")
        # List of terminal emulators to try
        terminals = [
            ['gnome-terminal', '--', 'rosrun', 'turtlesim', 'turtlesim_node'],
            ['xterm', '-e', 'rosrun', 'turtlesim', 'turtlesim_node'],
            ['konsole', '-e', 'rosrun', 'turtlesim', 'turtlesim_node'],
        ]
        max_retries = 3
        for attempt in range(max_retries):
            rospy.loginfo(f"Attempt {attempt + 1} of {max_retries}...")
            terminal = terminals[attempt]
            try:
                subprocess.Popen(terminal)
                time.sleep(2)  # Wait for turtlesim to start
                rospy.loginfo(f"turtlesim_node started successfully using {terminal[0]}.")
                self.remove_turtle('turtle1')
                return  # Exit after successful start
            except (subprocess.CalledProcessError, FileNotFoundError) as e:
                rospy.logwarn(f"Failed to start turtlesim_node with {terminal[0]}. Error: {e}")      
            rospy.logwarn("Retrying with the next terminal emulator...")

        rospy.logerr("Failed to start turtlesim_node after all attempts. Exiting game.")
        rospy.loginfo("Ending game...")
        rospy.logerr("Consider installing a terminal emulator like 'gnome-terminal', 'xterm', or 'konsole' to run ROS nodes.")
        rospy.loginfo("You can install one of these with the following commands:")
        rospy.loginfo("sudo apt-get install gnome-terminal")
        rospy.loginfo("sudo apt-get install xterm")
        rospy.loginfo("sudo apt-get install konsole")
        rospy.signal_shutdown("Game ended due to failure to start turtlesim_node.")
        sys.exit("--> Game Engine Closed <--")
            
    def subscribe_turtles(self):
        topics = rospy.get_published_topics()
        for topic, _ in topics:
            if '/pose' in topic:
                turtle_name = topic.split('/')[1]
                if turtle_name not in self.turtles:
                    # Initialize turtle state
                    self.turtles[turtle_name] = {'health': 100, 'attacks': 10, 'pose': None}
                    rospy.Subscriber(topic, Pose, self.update_turtle_pose, callback_args=turtle_name)
                    rospy.loginfo(f"Connected to {turtle_name}")

    def update_turtle_pose(self, data, turtle_name):
        if turtle_name in self.turtles:
            self.turtles[turtle_name]['pose'] = data

    def attack_callback(self, msg: String):
        if not self.game_started:
            return
        else:
            attacker = msg.data
            if self.turtles[attacker]['attacks'] > 0:
                rospy.logwarn(f"{attacker} performed an attack!")
                self.calculate_damage(attacker)
            else:
                rospy.logwarn(f"{attacker} is out of attacks")

            self.display_players_status()
            
    def display_players_status(self):
        print("--------------------------")
        for turtle, state in list(self.turtles.items()): 
             print(f"--> {turtle} <--")
             print(f"Turtle Health: {state['health']}")
             print(f"Turtle Attacks Left: {state['attacks']}")
             print("--------------------------")
             rospy.sleep(0.01)

    def calculate_damage(self, attacker):
        for target, state in list(self.turtles.items()): 
            if target != attacker and self.turtles[attacker]['pose'] and state['pose']:
                if self.attack_within_distance(self.turtles[attacker]['pose'], state['pose']):
                    self.process_attack(attacker, target)
                else:
                    self.turtles[attacker]['attacks'] -= 1
                    rospy.sleep(0.1)
        
    def attack_within_distance(self, pose1, pose2):
        distance = math.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)
        return distance <= 2.0

    def process_attack(self, attacker, target):
        self.turtles[target]['health'] -= 50
        self.turtles[attacker]['attacks'] -= 1
        rospy.logwarn(f"{attacker} attacked {target}!")
        if self.turtles[target]['health'] <= 0:
            rospy.logwarn(f"{target} has been defeated!")
            self.turtles.pop(target)
            self.remove_turtle(target)
            rospy.logwarn(f"{target} has been removed from the game.")

    def remove_turtle(self, turt):
        # Create a service proxy to the /kill service
        kill_turtle = rospy.ServiceProxy('/kill', Kill)
        # Wait for the service to be available
        rospy.wait_for_service('/kill')
        # Call the service with the name of the turtle to be killed
        kill_turtle(turt)


    
    def check_game_over(self):
        all_attacks_used = all(t['attacks'] == 0 for t in self.turtles.values())

        if all_attacks_used or len(self.turtles) == 1:
            max_health = max(self.turtles[t]['health'] for t in self.turtles)
            winners = [t for t in self.turtles if self.turtles[t]['health'] == max_health]

            if len(winners) == 1:
                winner_message = f"{winners[0]} wins!"
            else:
                winner_message = "It's a draw!"

            rospy.sleep(0.11) ## ----------------------------
            rospy.logwarn(winner_message)
            rospy.signal_shutdown("Game Over")
            self.initialize_files()

    def startgame_callback(self, request):
        self.start_game_pub.publish(1)
        rospy.loginfo("Game is Starting...")
        rospy.loginfo("3"); rospy.sleep(1); rospy.loginfo("2"); rospy.sleep(1); rospy.loginfo("1"); rospy.sleep(1)
        rospy.loginfo("GO!")
        self.game_started = True
        return EmptyResponse()

    def run(self):
        while not rospy.is_shutdown():
            if self.game_started:
                self.check_game_over()
            self.rate.sleep()

if __name__ == '__main__':
        engine = GameEngine()
        engine.run()

