import cv2
from ultralytics import YOLO
import sys
import os
import time
import logging

current_board = [['' for _ in range(3)] for _ in range(3)]
grid_size = 3
current_player = 'X'
symbol_start_time = None
last_cell_x, last_cell_y = -1, -1

def draw_grid(image):
    height, width = image.shape[:2]
    step_x, step_y = width // grid_size, height // grid_size
    color = (255, 255, 255)
    for i in range(1, grid_size):
        cv2.line(image, (i * step_x, 0), (i * step_x, height), color, 5)
        cv2.line(image, (0, i * step_y), (width, i * step_y), color, 5)

def get_grid_cell(x, y, frame_width, frame_height):
    return int(x // (frame_width // grid_size)), int(y // (frame_height // grid_size))

def place_symbol_on_grid_with_timer(current_board, cell_x, cell_y, symbol, current_player):
    global symbol_start_time, last_cell_x, last_cell_y

    if current_board[cell_y][cell_x] == '' and symbol == current_player: 
        if last_cell_x != cell_x or last_cell_y != cell_y:
            last_cell_x, last_cell_y = cell_x, cell_y
            symbol_start_time = time.time() 
        else:
            if time.time() - symbol_start_time >= 1:
                current_board[cell_y][cell_x] = symbol
                return True
    else:
        symbol_start_time = None
        last_cell_x, last_cell_y = -1, -1 
    return False

def check_winner(board):
    for i in range(grid_size):
        if board[i][0] == board[i][1] == board[i][2] and board[i][0] != '':
            return board[i][0]
        if board[0][i] == board[1][i] == board[2][i] and board[0][i] != '':
            return board[0][i]
    if board[0][0] == board[1][1] == board[2][2] and board[0][0] != '':
        return board[0][0]
    if board[0][2] == board[1][1] == board[2][0] and board[0][2] != '':
        return board[0][2]
    return None

def is_board_full(board):
    return all(cell != '' for row in board for cell in row)

def draw_symbols_on_grid(image, board):
    height, width = image.shape[:2]
    step_x, step_y = width // grid_size, height // grid_size
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale, thickness = 2, 3
    for row in range(grid_size):
        for col in range(grid_size):
            symbol = board[row][col]
            if symbol:
                x, y = col * step_x + step_x // 2, row * step_y + step_y // 2
                color = (0, 0, 255) if symbol == 'X' else (255, 0, 0)
                cv2.putText(image, symbol, (x - 40, y + 20), font, font_scale, color, thickness)

logging.getLogger('ultralytics').setLevel(logging.ERROR)  # or logging.CRITICAL

def run_yolo_v8(frame, model):
    old_stdout = sys.stdout
    sys.stdout = open(os.devnull, 'w')

    results = model(frame)

    sys.stdout = old_stdout
    
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = box.conf[0].cpu().numpy()
            class_id = int(box.cls[0].cpu().numpy())

            if conf > 0.7: 
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                
                symbol = 'Unknown'
                if class_id == 0: 
                    symbol = 'X'
                elif class_id == 1: 
                    symbol = 'O'
                
                cv2.putText(frame, f"{symbol} ({center_x}, {center_y})", (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                return center_x, center_y, symbol

    return None, None, None

def alternate_player_turn(symbol):
    global current_player
    if symbol == current_player:
        current_player = 'O' if current_player == 'X' else 'X'

def draw_player_turn(image):
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    color = (0, 0, 255) if current_player == 'X' else (255, 0, 0)
    cv2.putText(image, f"Player {current_player}'s Turn", (10, 40), font, font_scale, color, 2)

model = YOLO('XO_model.pt')

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open video capture.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break

    frame = cv2.flip(frame, 1)
    frame_height, frame_width = frame.shape[:2]
    draw_grid(frame)

    center_x, center_y, symbol = run_yolo_v8(frame, model)
    if center_x and center_y and symbol == current_player:
        cell_x, cell_y = get_grid_cell(center_x, center_y, frame_width, frame_height)
        if place_symbol_on_grid_with_timer(current_board, cell_x, cell_y, symbol, current_player):
            winner = check_winner(current_board)
            if winner:
                if winner=='X': font_color = (0, 0, 255) 
                else: font_color = (255, 0, 0)
                cv2.putText(frame, f"Player {winner} wins!", (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, font_color, 2)
                cv2.putText(frame, f"Press R to restart or Q to Exit", (200, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, font_color, 2)
                cv2.imshow("Tic-Tac-Toe", frame)
                cv2.waitKey(0)
                current_board = [['' for _ in range(3)] for _ in range(3)]
                current_player = 'X'
            elif is_board_full(current_board):
                cv2.putText(frame, "It's a draw!", (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 100), 2)
                cv2.putText(frame, f"Press R to restart or Q to Exit", (200, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 100), 2)    
                cv2.imshow("Tic-Tac-Toe", frame)
                cv2.waitKey(0)
                current_board = [['' for _ in range(3)] for _ in range(3)]
                current_player = 'X'
            else:
                alternate_player_turn(symbol)

    draw_symbols_on_grid(frame, current_board)
    draw_player_turn(frame)
    cv2.imshow("Tic-Tac-Toe", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or cv2.getWindowProperty('Tic-Tac-Toe', cv2.WND_PROP_VISIBLE) < 1:
        break
    elif key == ord('r'):
        current_board = [['' for _ in range(3)] for _ in range(3)]
        current_player = 'X'

cap.release()
cv2.destroyAllWindows()