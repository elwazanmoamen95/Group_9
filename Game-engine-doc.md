# Game Engine Documentation

## Overview

This script implements the Game Engine component for a Turtle Battle Royale game using `turtlesim` in ROS. It manages turtle interactions, game state, and handles the mechanics of the game.

## Main Parts of the Script

1. **Initialization of Variables and Setup**
    - **Node Initialization:** The script initializes a ROS node named `turtle_battle_game_engine` using `rospy.init_node`.
    - **Variables Initialization:**
    - `self.turtles`: Dictionary to store turtle states, including health, attacks, and pose.
    - `self.start_game_pub`: ROS publisher to signal the start of the game.
    - `# Game Engine Documentation

## Overview

This script implements the Game Engine component for a Turtle Battle Royale game using `turtlesim` in ROS. It manages turtle interactions, game state, and handles the mechanics of the game.

## Main Parts of the Script

1. **Initialization of Variables and Setup**
    - **Node Initialization:** The script initializes a ROS node named `turtle_battle_game_engine` using `rospy.init_node`.
    - **Variables Initialization:**
      - `self.turtles`: Dictionary to store turtle states, including health, attacks, and pose.
      - `self.start_game_pub`: ROS publisher to signal the start of the game.
      - `self.attack_sub`: ROS subscriber to listen for attack messages from the control node.
      - `self.startgame_service`: ROS service to initiate the game.
      - `self.rem`: Stores the name of the default turtle (`turtle1`) to be removed.
      - `self.game_started`: Boolean to track whether the game has started.
      - `self.rate`: Loop rate, set to 10 Hz, for the main loop execution.

2. **Starting the Turtlesim Node**
    - `start_turtlesim_node()`: Uses the `subprocess` module to start the `turtlesim_node` in a new terminal window using `xterm` or another terminal emulator. Ensure `xterm` is installed via `sudo apt-get install xterm`. After starting, it removes the default turtle (`turtle1`) using `self.remove_turtle(self.rem)`.

3. **Subscribing to Turtle Topics**
    - `game_lobby()`: Checks if the game has started. If not, it allows turtles to be spawned using `subscribe_turtles()`.
    - `subscribe_turtles()`: Subscribes to turtle pose topics (`/pose`). It monitors new turtles and updates `self.turtles` with default health (100) and attack count (10). Turtles cannot join if the game has already started.

4. **Updating Turtle Position**
    - `update_turtle_pose(data, turtle_name)`: Callback method to update each turtle's stored position when a new pose message is received.

5. **Handling Attacks**
    - `attack_callback(msg)`: Called when an attack message is received from the control node. Processes the attack if the game has started by invoking `calculate_damage(attacker)`.

6. **Calculating Damage**
    - `calculate_damage(attacker)`: Checks if the attacker is within range to damage other turtles using `attack_within_distance(pose1, pose2)`. If in range, calls `process_attack(attacker, target)`.

7. **Checking Attack Distance**
    - `attack_within_distance(pose1, pose2)`: Calculates the distance between two turtles. Returns `True` if they are within a 2.0 unit range, allowing the attack to occur.

8. **Processing an Attack**
    - `process_attack(attacker, target)`: Checks if the attacker has attacks remaining. If so, it decreases the attack count and reduces the target's health by 50. Prints an attack message and checks if the target’s health is zero or below. If so, removes the target from `self.turtles` and calls `self.remove_turtle(target)`.

9. **Removing a Turtle**
    - `remove_turtle(turt)`: Removes a turtle from the simulation using the `/kill` service provided by `turtlesim`.

10. **Checking if the Game is Over**
    - `check_game_over()`: Determines if the game should end based on two conditions: all turtles have exhausted their attacks, or only one turtle remains. If multiple turtles have the maximum health, it’s a draw; otherwise, the turtle with the highest health wins. Sends a shutdown signal to end the game.

11. **Starting the Game**
    - `startgame_callback(request)`: Called when the `/start_game` service is requested. Sets the game state to started, publishes a start signal, and manages a countdown from 3. Prevents additional turtles from joining after the game starts.

12. **Main Loop**
    - `run()`: The main loop that operates while the ROS node is active (`rospy.is_shutdown()` is `False`). Checks if the game has started and continuously verifies if game-ending conditions are met. Runs at the rate of 10 Hz.self.attack_sub`: ROS subscriber to listen for attack messages from the control node.
        - `self.startgame_service`: ROS service to initiate the game.
        - `self.rem`: Stores the name of the default turtle (`turtle1`) to be removed.
        - `self.game_started`: Boolean to track whether the game has started.
        - `self.rate`: Loop rate, set to 10 Hz, for the main loop execution.

2. **Starting the Turtlesim Node**
    - `start_turtlesim_node()`: Uses the `subprocess` module to start the `turtlesim_node` in a new terminal window using `xterm` or another terminal emulator. Ensure `xterm` is installed via `sudo apt-get install xterm`. After starting, it removes the default turtle (`turtle1`) using `self.remove_turtle(self.rem)`.

3. **Subscribing to Turtle Topics**
    - `game_lobby()`: Checks if the game has started. If not, it allows turtles to be spawned using `subscribe_turtles()`.
    - `subscribe_turtles()`: Subscribes to turtle pose topics (`/pose`). It monitors new turtles and updates `self.turtles` with default health (100) and attack count (10). Turtles cannot join if the game has already started.

4. **Updating Turtle Position**
    - `update_turtle_pose(data, turtle_name)`: Callback method to update each turtle's stored position when a new pose message is received.

5. **Handling Attacks**
    - `attack_callback(msg)`: Called when an attack message is received from the control node. Processes the attack if the game has started by invoking `calculate_damage(attacker)`.

6. **Calculating Damage**
    - `calculate_damage(attacker)`: Checks if the attacker is within range to damage other turtles using `attack_within_distance(pose1, pose2)`. If in range, calls `process_attack(attacker, target)`.

7. **Checking Attack Distance**
    - `attack_within_distance(pose1, pose2)`: Calculates the distance between two turtles. Returns `True` if they are within a 2.0 unit range, allowing the attack to occur.

8. **Processing an Attack**
    - `process_attack(attacker, target)`: Checks if the attacker has attacks remaining. If so, it decreases the attack count and reduces the target's health by 50. Prints an attack message and checks if the target’s health is zero or below. If so, removes the target from `self.turtles` and calls `self.remove_turtle(target)`.

9. **Removing a Turtle**
    - `remove_turtle(turt)`: Removes a turtle from the simulation using the `/kill` service provided by `turtlesim`.

10. **Checking if the Game is Over**
    - `check_game_over()`: Determines if the game should end based on two conditions: all turtles have exhausted their attacks, or only one turtle remains. If multiple turtles have the maximum health, it’s a draw; otherwise, the turtle with the highest health wins. Sends a shutdown signal to end the game.

11. **Starting the Game**
    - `startgame_callback(request)`: Called when the `/start_game` service is requested. Sets the game state to started, publishes a start signal, and manages a countdown from 3. Prevents additional turtles from joining after the game starts.

12. **Main Loop**
    - `run()`: The main loop that operates while the ROS node is active (`rospy.is_shutdown()` is `False`). Checks if the game has started and continuously verifies if game-ending conditions are met. Runs at the rate of 10 Hz.