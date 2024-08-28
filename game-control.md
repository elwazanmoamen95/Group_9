# Turtle Controller Documentation

## Overview
This script is part of the `Control Engine` for a Turtle battle game using `turtlesim` in `ROS`. It allows players to control a turtle’s movements and actions in the game, using keyboard inputs to navigate and attack.

## Main Parts of the Script

### 1. Initialization and Setup
- **File Checking**: The script starts by checking if the game has begun by reading from `file_start.txt`. If the game has started, the script exits. If not, it reads the number of turtles from `file.txt`.
- **User Input**: The user is asked if they want to control an existing turtle or spawn a new one. If controlling an existing turtle, the user must enter a valid turtle number. If the input is invalid or if no turtle number is specified, the script automatically selects the highest turtle number.

### 2. Spawning a New Turtle
- **Turtle Spawning**: If a new turtle is required, the script uses `rosservice call /spawn` to create the turtle in the simulation, and sets its pen properties using the `/set_pen` service.
- **File Update**: After spawning a new turtle, the script updates `file.txt` with the new turtle number.

### 3. Setting Up ROS Node and Publishers
- **Node Initialization**: The script initializes a ROS node with a name based on the current turtle.
- **Publishers**: It sets up publishers for turtle movement (`cmd_vel`), attack actions (`/turtle_attack`), and new turtle notifications (`/new_turtle`).

### 4. Keyboard Controls for Turtle Movement
- **Keyboard Input**: The script uses non-blocking terminal settings to read keyboard inputs. The `W`, `A`, `S`, `D` keys control the turtle’s movement forward, left, backward, and right, respectively. Pressing `Q` triggers an attack.
- **Movement and Attack Commands**: Based on the key pressed, the script publishes a `Twist` message to control the turtle’s velocity or an attack command to the appropriate ROS topic.

### 5. Starting the Game and Attacks
- **Game Start Listener**: A ROS subscriber listens to the `/game_started` topic to determine if the game has started. When the game starts, it updates `file_start.txt` accordingly.
- **Attacks**: When `Q` is pressed, the script publishes an attack message to the `/turtle_attack` topic, commanding the turtle to attack.