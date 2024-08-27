#!/usr/bin/env python3
import rospy
import time
from turtlesim.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

# Note: still need to: 1- handle collision attacks | 2- update status of each player accordingly | 3- handle winner 

class GameEngine:
    def __init__(self):
        rospy.init_node('game_engine')
        
        self.player_statuses = {} # [turtleID]: {'health': x, 'attacks_left': x}
        # Int32MultiArray: (turtleID1, health1, attacks_left1, turtleID2, health2, ...)
        self.status_pub = rospy.Publisher('/game_status', Int32MultiArray, queue_size=10) 
        
        self.turtle_subs = {}
        self.attack_sub = rospy.Subscriber('/turtle_attack', String, self.attack_callback)
        

        self.update_interval = 5  # Updates turtle subscriptions every 5 seconds
        self.last_update_time = time.time()

        self.rate = rospy.Rate(10)
        rospy.loginfo("Game Engine has been created")
        self.publish_game_status()
        self.update_turtle_subscriptions()
        self.run()
        


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
            self.player_statuses[player_id] = {
                'health': 100,  # Default initial health
                'attacks_left': 10  # Default initial attacks left
            }
            rospy.loginfo(f"Initialized status for turtle {player_id}")
            self.publish_game_status()

    def publish_game_status(self, update_id = None, update_health = None, update_attacks = None):
        status_msg = Int32MultiArray()
        data = []
        
        
             
        for player_id, status in self.player_statuses.items():

            # Check and cast types
            player_id = int(player_id)
            health = int(status['health'])
            attacks_left = int(status['attacks_left'])
            
            #check if update needed
            if update_id == player_id:
                if update_health != None:
                    health = update_health
                if update_attacks != None:
                    attacks_left = update_attacks

            self.player_statuses[player_id] = {
                'health': health,
                'attacks_left': attacks_left
            }

            # Append data
            data.extend([player_id, health, attacks_left])
            
        # Set the data field of the message
        status_msg.data = data
        
        rospy.loginfo(f"Publishing game status: {status_msg.data}")
        self.status_pub.publish(status_msg)


    # <-- CALLBACK FUNCTIONS -->
    def pose_callback(self, msg: Pose):
        # mechanism to detect if a new turtle has joined.
        turtle_name = str(msg._connection_header['topic']).split('/')[1]
        _, player_id = turtle_name.split("turtle")
        player_id = int(player_id)

        if player_id not in self.player_statuses:
            # rospy.loginfo(f"Initializing status for new turtle: {player_id}")
            self.initialize_player_status(player_id)

    def attack_callback(self, msg: String):
        turtle_name = msg.data
        rospy.loginfo(f"{turtle_name} performed an attack!") 
        # still need to implement attack logic
        # --> to do <--
        
        # for testing purpose
        _, player_id = turtle_name.split("turtle")
        player_id = int(player_id)
        curr_attacks = int(self.player_statuses[player_id]['attacks_left'])
        self.publish_game_status(update_id=player_id, update_attacks=curr_attacks-1) 
        

    def run(self):
        while not rospy.is_shutdown():
            current_time = time.time()
            if current_time - self.last_update_time > self.update_interval:
                self.update_turtle_subscriptions() # updates the number list of turtles every {self.update_interval} seconds
                self.last_update_time = current_time
            
            rospy.loginfo(f"Status: {self.player_statuses}") # for debug
            self.rate.sleep()


if __name__ == '__main__':
    try:
        GameEngine()
    except rospy.ROSInterruptException:
        pass