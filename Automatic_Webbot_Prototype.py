import math
import os
os.chdir("../..")

from fairis_tools.MyRobot import MyRobot

robot = MyRobot()
maze_file = 'worlds/mazes/Labs/Lab5/Lab5_SmallMaze1.xml'
robot.load_environment(maze_file)
robot.move_to_start()

starting_x = robot.starting_position.x
starting_y = robot.starting_position.y
starting_theta = robot.starting_position.theta
        
def detectMazeInconsistencies(maze):
    for i in range(3):
        for j in range(4):
            pos1 = i * 4 + j
            pos2 = i * 4 + j + 4
            hWall1 = maze[pos1].south
            hWall2 = maze[pos2].north
            assert hWall1 == hWall2, " Cell " + str(pos1) + "'s south wall doesn't equal cell " + str(
                pos2) + "'s north wall! ('" + str(hWall1) + "' != '" + str(hWall2) + "')"

    for i in range(4):
        for j in range(3):
            pos1 = i * 4 + j
            pos2 = i * 4 + j + 1
            vWall1 = maze[pos1].east
            vWall2 = maze[pos2].west
            assert vWall1 == vWall2, " Cell " + str(pos1) + "'s east wall doesn't equal cell " + str(
                pos2) + "'s west wall! ('" + str(vWall1) + "' != '" + str(vWall2) + "')"

def printMaze(maze, hRes=4, vRes=2):
    assert hRes > 0, "Invalid horizontal resolution"
    assert vRes > 0, "Invalid vertical resolution"

    hChars = 4 * (hRes + 1) + 2
    vChars = 4 * (vRes + 1) + 1

    output = [" "] * (hChars * vChars - 1)

    for i in range(1, hChars - 2):
        output[i] = "_"

    for i in range(hChars * (vChars - 1) + 1, hChars * (vChars - 1) + hChars - 2):
        en_dash = '\u2013'
        output[i] = en_dash

    for i in range(hChars, hChars * (vChars - 1), hChars):
        output[i] = "|"

    for i in range(2 * hChars - 2, hChars * (vChars - 1), hChars):
        output[i] = "|"

    for i in range(hChars - 1, hChars * vChars - 1, hChars):
        output[i] = "\n"

    for i in range((vRes + 1) * hChars, hChars * (vChars - 1), (vRes + 1) * hChars):
        for j in range(hRes + 1, hChars - 2, hRes + 1):
            output[i + j] = "·"

    for i in range(4):
        for j in range(4):
            cellNum = i * 4 + j
            if maze[cellNum].visited:
                continue
            origin = (i * hChars * (vRes + 1) + hChars + 1) + (j * (hRes + 1))
            for k in range(vRes):
                for l in range(hRes):
                    output[origin + k * hChars + l] = "?"

    for i in range(3):
        for j in range(4):
            cellNum = i * 4 + j
            origin = ((i + 1) * hChars * (vRes + 1) + 1) + (j * (hRes + 1))
            hWall = maze[cellNum].south
            for k in range(hRes):
                output[origin + k] = "-" if hWall == 'W' else " " if hWall == 'O' else "?"

    for i in range(4):
        for j in range(3):
            cellNum = i * 4 + j
            origin = hChars + (hRes + 1) * (j + 1) + i * hChars * (vRes + 1)
            vWall = maze[cellNum].east
            for k in range(vRes):
                output[origin + k * hChars] = "|" if vWall == 'W' else " " if vWall == 'O' else "?"

    print(''.join(output))

# Function to determine the index of the current grid cell based on robot's position
def determine_locate_cells_index(x, y):
    if 1 <= y <= 2:
        row = 0
    elif 0 <= y < 1:
        row = 1
    elif -1 <= y < 0:
        row = 2
    elif -2 <= y < -1:
        row = 3
    else:
        row = None

    if -2 <= x <= -1:
        column = 0
    elif -1 < x <= 0:
        column = 1
    elif 0 < x <= 1:
        column = 2
    elif 1 < x <= 2:
        column = 3
    else:
        column = None

    current_idx = maze_cells.index((row, column))
    return current_idx

# Define the maze cells as before
maze_cells = [(x, y) for x in range(4) for y in range(4)]

# Define a Cell class to represent each grid cell in the maze
class Cell:
    def __init__(self, west, north, east, south, visited=False):
        self.west = west
        self.north = north
        self.east = east
        self.south = south
        self.visited = visited

# Create an initial maze configuration with all walls present
maze = [Cell('W', 'W', 'W', 'W') for _ in range(16)]

def move_to_next_cell(current_idx):
    if robot.get_lidar_range_image()[400] > 1.0:
        robot.move_distance(1.0, 10.0)
    elif robot.get_lidar_range_image()[200] > 1.0:
        robot.rotate_bot_degrees(math.pi + math.pi/1.5, 3)
        robot.fix_direction()
        robot.move_distance(1.0, 10.0)
    elif robot.get_lidar_range_image()[600] > 1.0:
        robot.rotate_bot_degrees(math.pi/1.5, 3)
        robot.fix_direction()
        robot.move_distance(1.0, 10.0)
    else:
        robot.rotate_bot_degrees(math.pi + math.pi/2, 3)
        robot.fix_direction()
        robot.move_distance(1.0, 10.0)
        
def next_cell_index(current_idx):         
    if robot.get_compass_reading() in range(85, 95):
        next_cell = current_idx - 4
    elif robot.get_compass_reading() in range(175, 185):
        next_cell = current_idx - 1
    elif robot.get_compass_reading() in range(265, 275):
        next_cell = current_idx + 4
    else:
        next_cell = current_idx + 1
    
    return next_cell

def update_maze_with_robot_position(current_idx):
    print("Current cell:", current_idx, "Theta:", robot.get_compass_reading())

    if maze[current_idx].visited == True:
        print("Cell already visited!")
        move_to_next_cell(current_idx)
        next_cell = next_cell_index(current_idx)
    else:
        maze[current_idx].visited = True
        next_cell = next_cell_index(current_idx)
        
        if robot.get_compass_reading() in range(85, 95):
            print("Facing North")
            if robot.get_lidar_range_image()[400] < 0.7:
                maze[current_idx].north = 'W'
            else:
                maze[current_idx].north = 'O'
                maze[next_cell].south = 'O'
                
            maze[current_idx].south = 'O' if robot.get_lidar_range_image()[0] > 0.7 else'W'
            
            maze[current_idx].west = 'W' if robot.get_lidar_range_image()[200] < 0.7 else'O'
                
            maze[current_idx].east = 'W' if robot.get_lidar_range_image()[600] < 0.7 else 'O'

        elif robot.get_compass_reading() in range(175, 185): 
            print("Facing West")
            if robot.get_lidar_range_image()[400] < 0.7:
                maze[current_idx].west = 'W'
            else:
                maze[current_idx].west = 'O' 
                maze[next_cell].east = 'O'
            
            maze[current_idx].east = 'O' if robot.get_lidar_range_image()[0] > 0.7 else 'W'

            maze[current_idx].north = 'W' if robot.get_lidar_range_image()[600] < 0.7 else 'O'
                
            maze[current_idx].south = 'W' if robot.get_lidar_range_image()[200] < 0.7 else 'O'
            
        elif robot.get_compass_reading() in range(265, 275):
            print("Facing South")
            maze[current_idx].west = 'W' if robot.get_lidar_range_image()[600] < 0.7 else 'O'
        
            if robot.get_lidar_range_image()[400] < 0.7:
                maze[current_idx].south = 'W'
            else:
                maze[current_idx].south = 'O'
                maze[next_cell].north = 'O'

            maze[current_idx].north = 'O' if robot.get_lidar_range_image()[0] > 0.7 else 'W'
            
            maze[current_idx].east = 'W' if robot.get_lidar_range_image()[200] < 0.7 else 'O'

        else:
            print("Facing East")
            if robot.get_lidar_range_image()[400] < 0.7:
                maze[current_idx].east = 'W'
            elif robot.get_lidar_range_image()[400] > 0.7:
                maze[current_idx].east = 'O'
                maze[next_cell].west = 'O'
            
            maze[current_idx].west = 'O' if robot.get_lidar_range_image()[0] > 0.7 else 'W'

            maze[current_idx].north = 'W' if robot.get_lidar_range_image()[200] < 0.7 else 'O'
        
            maze[current_idx].south = 'W' if robot.get_lidar_range_image()[600] < 0.7 else 'O'
       
        print("Current Cell Wall Configuration:")
        print("North Wall:", maze[current_idx].north)
        print("East Wall:", maze[current_idx].east)
        print("South Wall:", maze[current_idx].south)
        print("West Wall:", maze[current_idx].west)
        move_to_next_cell(current_idx) 
        next_cell = next_cell_index(current_idx)     
    print("Next cell:", next_cell)
    print("-------------------------------------------------")
    return next_cell

if __name__ == "__main__":
    current_index = determine_locate_cells_index(starting_x, starting_y)
    maze[current_index].visited = False
    while robot.experiment_supervisor.step(robot.timestep) != -1:
        printMaze(maze)
        current_index = update_maze_with_robot_position(current_index)
        all_cells_visited = all(cell.visited for cell in maze)
        if all_cells_visited:
            print("Hey Pavan! You Visited all the cells in the maze, Printing final Maze!")
            printMaze(maze)
            robot.stop()
            break
