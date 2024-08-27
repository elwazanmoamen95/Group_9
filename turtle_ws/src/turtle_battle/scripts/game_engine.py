#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

# Note: still need to: 1- handle collision attacks | 2- update status of each player accordingly | 3- handle winner 

class GameEngine:
    def __init__(self):
        rospy.init_node('game_engine')
        
        self.player_statuses = {} # [turtleID]: health: x, attacks_left x
        # Int32MultiArray: (turtleID1, health1, attacks_left1, turtleID2, health2, ...)
        self.status_pub = rospy.Publisher('/game_status', Int32MultiArray, queue_size=10) 
        self.status_sub = rospy.Subscriber('/game_status', Int32MultiArray, self.status_callback)
        
        self.turtle_subs = {}
        self.attack_sub = rospy.Subscriber('/turtle_attack', String, self.attack_callback)
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            self.update_turtle_subscriptions() # constantly updates the number list of turtles
            self.publish_game_status() # constantly updates the status of all turtles (health, attacks left..)
            rate.sleep()

    def get_active_turtles(self):
        # Retrieve all topics
        topics = rospy.get_published_topics() # list of [[topic, topic type], ...]
        # Filter topics to find turtle pose topics
        turtle_topics = [str(topic) for topic, _ in topics if 'pose' in topic]
        # Extract turtle names from topic names
        turtle_names = [topic.split('/')[1] for topic in turtle_topics]

        rospy.loginfo(f"Active turtles: {turtle_names}") # remove later just displaying players for testing

        return turtle_names
    
    def update_turtle_subscriptions(self):
        turtles = self.get_active_turtles()
        for turtle_name in turtles:
            if turtle_name not in self.turtle_subs:
                self.turtle_subs[turtle_name] = rospy.Subscriber(f'/{turtle_name}/pose', Pose, self.pose_callback)

    def initialize_player_status(self, player_id):
        # Initialize status for a new player (turtle)
        if player_id not in self.player_statuses:
            self.player_statuses[int(player_id)] = {
                'health': 100,  # Default initial health
                'attacks_left': 10  # Default initial attacks left
            }
            rospy.loginfo(f"Initialized status for turtle {player_id}")
            self.publish_game_status()

    def publish_game_status(self):
        status_msg = Int32MultiArray()
        data = []

        for player_id, status in self.player_statuses.items():
            # Check and cast types
            player_id = int(player_id)
            health = int(status['health'])
            attacks_left = int(status['attacks_left'])

            # Append data
            data.extend([player_id, health, attacks_left])
            
        # Set the data field of the message
        status_msg.data = data
        
        rospy.loginfo(f"Publishing game status: {status_msg.data}")
        self.status_pub.publish(status_msg)
    


    # <-- CALLBACK FUNCTIONS -->
    def pose_callback(self, msg: Pose):
        # mechanism to detect if a new turtle has joined.
        turtle_name = str(msg._connection_header['topic'].split('/')[1])
        _, player_id = turtle_name.split("turtle")

        if player_id not in self.player_statuses:
            rospy.loginfo(f"Initializing status for new turtle: {player_id}")
            self.initialize_player_status(player_id)

    def attack_callback(self, msg: String):
        turtle_name = msg.data
        rospy.loginfo(f"Turtle {turtle_name} performed an attack!") 
        # still need to implement attack logic
        # --> to do <--

    def status_callback(self, msg: Int32MultiArray):
        # Get the number of active turtles (players)
        num_players = len(self.get_active_turtles())

        data = msg.data
        expected_length = num_players * 3
        
        # for debugging
        rospy.loginfo(f"Received data: {data}")
        rospy.loginfo(f"Expected length: {expected_length}, Actual length: {len(data)}")


        # Check if the data length matches the expected length
        if len(data) != expected_length:
            rospy.logwarn("Received status message with unexpected length.")
            return
        
        # Process the data in chunks of 3 (id, health, attacks left)
        for i in range(0, expected_length, 3):
            player_id = int(data[i])
            health = int(data[i + 1])
            attacks_left = int(data[i + 2])

            # Update player status
            self.player_statuses[player_id] = {'health': health, 'attacks_left': attacks_left}
            rospy.loginfo(f"Updated Status - Turtle {player_id}: Health={health}, Attacks Left={attacks_left}")


if __name__ == '__main__':
    try:
        GameEngine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
