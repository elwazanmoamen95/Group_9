# Detection Model (XO_model.pt)
We have successfully collected and labeled a dataset of 800 images for training and validating. This dataset has been utilized to train a YOLOv8n model. The dataset can be accessed via the following link: [Dataset](https://drive.google.com/drive/folders/1p8L1BXJy3eCwTQnomwf4Z4hK_xLWokFq?usp=sharing).

# Game Engine (XO_game.py)

## Overview

This project implements a real-time Tic-Tac-Toe game using computer vision and the YOLOv8n  object detection model. The game recognizes hand gestures corresponding to 'X' and 'O' symbols and places them on a 3x3 grid.


## How It Works

### 1. Initialization

- **Grid Setup**: Initializes an empty 3x3 grid for the Tic-Tac-Toe game.
- **Current Player**: Tracks the player whose turn it is ('X' or 'O').


### 2. Drawing Functions

- `draw_grid(image)`: Draws a 3x3 grid on the video frame.
- `draw_symbols_on_grid(image, board)`: Draws the 'X' and 'O' symbols on the grid based on the game board.
- `draw_player_turn(image)`: Displays the current player's turn on the screen.

### 3. Grid Cell Calculation

- `get_grid_cell(x, y, frame_width, frame_height)`: Determines which grid cell a detected symbol falls into based on its coordinates.

### 4. Symbol Placement

- `place_symbol_on_grid_with_timer(current_board, cell_x, cell_y, symbol, current_player)`: Places a symbol on the grid after a 1-second interval if the cell is empty and the symbol matches the current player.

### 5. Game Logic

- `check_winner(board)`: Checks if there is a winner on the board.
- `is_board_full(board)`: Determines if the board is full, resulting in a draw.

### 6. YOLO Object Detection

- `run_yolo_v8(frame, model)`: Uses the YOLO model to detect 'X' and 'O' symbols in the video frame and returns their positions and classes.

### 7. Game Loop

- Captures video frames.
- Draws the grid and symbols.
- Processes object detection results.
- Updates the game state based on the detected symbols.
- Displays the result if a player wins or if the game is a draw.
- Handles user input for restarting or exiting the game.

## Usage

1. **Run the Script**:
   ```bash
   python XO_game.py
   ```

2. **Gameplay**:
   - Place your hand in front of the camera to make a gesture.
   - The system will detect 'X' and 'O' and place the symbols on the grid.
   - The game alternates turns between 'X' and 'O'.
   - The game announces a winner or a draw when appropriate.
   - Press 'R' to restart the game.
   - Press 'Q' to exit the game.

