# Game Engine Documentation

## Overview
This script contains the `Game Engine` part that will go along with the `Control Engine` to build a Turtle battle royal game using `turtlesim` in `ROS` 

## Main Parts of the script

### 1. Initialization of the variables and Setup the game
- **Node Initialization**: The script starts by initializing a ROS node named `turtle_battle_game_engine` using `rospy.init_node`
- **Variables Initialization**:
  - `self.turtles`: Dictionary to store turtle states (health, attacks, pose)
  - `self.start_game_pub`: ROS publisher to send the signal to start the game
  - `self.attack_sub`: A ros subscriber that will listen for attack messages that will be published in a topic in Control node
  - `self.startgame_service`: A ROS service that starts the game
  - `self.rem`: Will store the name of the default turtle `turtle1` to be removed
  - `self.game_started`: A boolean to track game status (started or not)
  - `self.rate`: A loop rate to maintain the rate at 10hz

### 2. Starting the `turtlesim` Node
- `start_turtlesim_node()` uses the `subprocess` module to start the `turtlesim_node` in a new terminal window using `xterm`. This makes the simulation compatible with everything `wsl` , `ubuntu` ..etc and it must be installed using `sudo apt-get install xterm`. 
- After the previous command, it will automatically remove the default turtle upon starting `self.remove_turtle(self.rem)`

### 3. Subscribing to Turtle Topics
- using `self.game_lobby()` it will check if the game has started or not, if still in the lobby it will allow turtles to be spawned using `subscribe_turtles()`: This will look for topics related to turtle positions `pose`. It subscribes to these topics to monitor each turtle's movements.
- It will check first if the active topics is already in `self.turtles` dictionary or not, if not that means a new turtle spawned, and it will be added to the `self.turtles` with default health (100) and attack count (10).
- but if game is already started it will not go in the while loop and will not allow any more turtles to join
### 4. Updating Turtle Position
- `update_turtle_pose(self, data, turtle_name)`: This callback method updates the stored position of each turtle whenever a new position message is received

### 5. Attacking
- `attack_callback(self, msg:String)`: This method gets called when an attack message is received from the control node. It checks if the game has started and then processes the attack by calling `calculate_damage(attacker)`

### 6. Calculating Damage
- `calculate_damage(self, attacker)`: This method checks if the attacking turtle is close enough to other turtles to damage it using the `attack_within_distance(pose1, pose2)` method it takes the position of both the attacked and other turtles.
- If within range, then the if statment will be true and it will call `process_attack(attacker, target)`.
- 
### 7. Checking Attack Distance
- `attack_within_distance(pose1, pose2)`: This  function calculates the distance between two turtles and returns `True` if they are within a 2.0 unit range, allowing an attack to occur.

### 8. Processing an Attack
- `process_attack(self, attacker, target)` will first check if the attacker has attacks left or not, then it will decrement its attacks by 1 and subtract the target health by 50, and then it will print `turtle-x attacked turtle-y`
- Lastly, it will check on the target's health and if its equal or less than zero means this turtle has been defeated and will remove this turtle from `self.turtles` dictionary using `self.turtles.pop(target)` then it will call the function `self.remove_turtle(target)` giving it the target that has been defeated


### 9. Removing a Turtle
- `remove_turtle(self, turt)`: This method removes a turtle from the simulation by calling the `/kill` service, which is provided by `turtlesim` to remove turtles by name

### 10. Checking if the Game is Over
- `check_game_over(self)`: This method checks the conditions to end the game: all turtles have used their attacks, or only one turtle remains. It determines the winner based on health points and its done by getting out the maximum health of the remaining turtles(if there is more than one but all attacks been used) storing it in `max_health` variable, then it will store the remainig turtles with that max health in `winners`. If there is more than one its a draw else it will declare the winner. Then lastly it will send a shutdown signal to end the game.

### 11. Starting the Game
- `startgame_callback(self, request)`: This method is called when the `/start_game` service is called. It sets the game state to started, publishes a message to signal this, and counts down from 3 to start the game.

- There would be a lobby feature, where turtles will login and spawn, but after starting the game, no more turtles will be able to spawn

### 12. Main Loop
- `run(self)`: The main loop that runs as long as the ROS node is active `while not rospy.is_shutdown()`. It checks if the game has started and continuously checks if the game conditions for ending have been met`self.check_game_over()`. runs by the rate 10hz that been initialized before. 



