
#!/usr/bin/env python3
# Github repository:  https://github.com/MayankD409/A_star_algorithm_on_rigid_robot.git

import pygame
import numpy as np
import time
import heapq
import math
import os
import cv2

# Define a class to represent nodes in the search space
class Node:
    def __init__(self, x, y, theta, cost, parent_id, c2g=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost
        self.parent_id = parent_id
        self.c2g = c2g 
        
    # Define comparison method for priority queue
    def __lt__(self, other):
        return self.cost + self.c2g < other.cost + other.c2g

# Define possible actions and associated cost increments
def move_up_60_deg(x, y, theta, step_size, cost):
    theta = theta + 60
    x = x + (step_size * np.cos(np.radians(theta)))
    y = y + (step_size * np.sin(np.radians(theta)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x, y, theta, cost

def move_up_30_deg(x, y, theta, step_size, cost):
    theta = theta + 30
    x = x + (step_size * np.cos(np.radians(theta)))
    y = y + (step_size * np.sin(np.radians(theta)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x, y, theta, cost

def zero_movement(x, y, theta, step_size, cost):
    theta = theta + 0
    x = x + (step_size * np.cos(np.radians(theta)))
    y = y + (step_size * np.sin(np.radians(theta)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x, y, theta, cost

def move_30_down_deg(x, y, theta, step_size, cost):
    theta = theta - 30
    x = x + (step_size * np.cos(np.radians(theta)))
    y = y + (step_size * np.sin(np.radians(theta)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x, y, theta, cost

def move_down_60_deg(x, y, theta, step_size, cost):
    theta = theta - 60
    x = x + (step_size * np.cos(np.radians(theta)))
    y = y + (step_size * np.sin(np.radians(theta)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x, y, theta, cost

# Function to check if a point is inside a rectangle
def is_point_inside_rectangle(x, y, vertices):
    x_min = min(vertices[0][0], vertices[1][0], vertices[2][0], vertices[3][0])
    x_max = max(vertices[0][0], vertices[1][0], vertices[2][0], vertices[3][0])
    y_min = min(vertices[0][1], vertices[1][1], vertices[2][1], vertices[3][1])
    y_max = max(vertices[0][1], vertices[1][1], vertices[2][1], vertices[3][1])
    return x_min <= x <= x_max and y_min <= y <= y_max

# Function to check if a point is inside a hexagon
def is_point_inside_hexagon(x, y, center_x, center_y, side_length):
    cx, cy = center_x, center_y
    vertices = []
    angle_deg = 60
    angle_rad = math.radians(angle_deg)
    for i in range(6):
        px = cx + side_length * math.cos(angle_rad * i + math.radians(30))
        py = cy + side_length * math.sin(angle_rad * i + math.radians(30))
        vertices.append((px, py))
    odd_nodes = False
    j = 5
    for i in range(6):
        if (vertices[i][1] < y and vertices[j][1] >= y) or (vertices[j][1] < y and vertices[i][1] >= y):
            if (vertices[i][0] + (y - vertices[i][1]) / (vertices[j][1] - vertices[i][1]) * (vertices[j][0] - vertices[i][0])) < x:
                odd_nodes = not odd_nodes
        j = i
    return odd_nodes

# Function to check if a point is inside a C-shaped block
def is_point_inside_block(point, vertices):
    odd_nodes = False
    j = len(vertices) - 1
    for i in range(len(vertices)):
        if (vertices[i][1] < point[1] and vertices[j][1] >= point[1]) or (vertices[j][1] < point[1] and vertices[i][1] >= point[1]):
            if (vertices[i][0] + (point[1] - vertices[i][1]) / (vertices[j][1] - vertices[i][1]) * (vertices[j][0] - vertices[i][0])) < point[0]:
                odd_nodes = not odd_nodes
        j = i
    return odd_nodes

# Function to create configuration space with obstacles
def Configuration_space(width, height, robot_radius, clearance):
    obs_space = np.full((height, width), 0)
    
    for y in range(height):
        for x in range(width):
            # Creating buffer space for obstacles    
            rectangle1_buffer_vts = [(50 - (robot_radius + clearance), 250), (87.5 + (robot_radius + clearance), 250), (87.5 + (robot_radius + clearance), 50 - (robot_radius + clearance)), (50 - (robot_radius + clearance), 50 - (robot_radius + clearance))]
            rectangle2_buffer_vts = [(137.5 - (robot_radius + clearance), 200 + (robot_radius + clearance)), (175 + (robot_radius + clearance), 200 - (robot_radius + clearance)), (175 + (robot_radius + clearance), 0), (137.5 - (robot_radius + clearance), 0)]
            cblock_buffer_vts = [(450 - (robot_radius + clearance), 225 + (robot_radius + clearance)), (450 - (robot_radius + clearance), 187.5 - (robot_radius + clearance)), (510 - (robot_radius + clearance), 187.5 - (robot_radius + clearance)), (510 - (robot_radius + clearance), 62.5 + (robot_radius + clearance)), (450 - (robot_radius + clearance), 62.5 + (robot_radius + clearance)), 
                                 (450 - (robot_radius + clearance), 25 - (robot_radius + clearance)), (550 + (robot_radius + clearance), 25 - (robot_radius + clearance)), (550 + (robot_radius + clearance), 225 + (robot_radius + clearance))]

            rect1_buffer = is_point_inside_rectangle(x,y, rectangle1_buffer_vts)
            rect2_buffer = is_point_inside_rectangle(x, y, rectangle2_buffer_vts)
            hexa_buffer = is_point_inside_hexagon(x, y, 325, 125, 75 + (robot_radius + clearance))
            cblock_buffer = is_point_inside_block((x, y), cblock_buffer_vts)
            
            # Setting buffer space constraints to obtain obstacle space
            if cblock_buffer or rect1_buffer or rect2_buffer or hexa_buffer:
                obs_space[y, x] = 1
             
            # Plotting actual obstacle space using half-plane equations
            rectangle1_vts = [(50, 250), (87.5, 250), (87.5, 50), (50, 50)]
            rectangle2_vts = [(137.5, 200), (175, 200), (175, 0), (137.5, 0)]
            cblock_vertices = [(450, 225), (450, 187.5), (510, 187.5), (510, 62.5), (450, 62.5), (450, 25), (550, 25), (550, 225)]

            rect1 = is_point_inside_rectangle(x, y, rectangle1_vts)
            rect2 = is_point_inside_rectangle(x, y, rectangle2_vts)
            hexa = is_point_inside_hexagon(x, y, 325, 125, 75)
            cblock = is_point_inside_block((x, y), cblock_vertices)

            # Setting the constraints to obtain the obstacle space without buffer
            if cblock or rect1 or rect2 or hexa:
                obs_space[y, x] = 2
                
    # Adding clearance for walls
    for i in range(height):
        for j in range(3 + (robot_radius + clearance)):
            obs_space[i][j] = 1
            obs_space[i][width - j - 1] = 1

    for i in range(width):
        for j in range(3 + (robot_radius + clearance)):  
            obs_space[j][i] = 1
            obs_space[height - j - 1][i] = 1 

    return obs_space

# Function to check if a move is valid
def is_valid(x, y, obs_space):
    height, width = obs_space.shape
    
    # Check if coordinates are within the boundaries of the obstacle space and not occupied by an obstacle
    if x < 0 or x >= width or y < 0 or y >= height or obs_space[y][x] == 1 or obs_space[y][x] == 2:
        return False
    
    return obs_space[y, x] == 0

# Function to calculate Euclidean distance between two points
def euclidean_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)

# Function to check if the goal node is reached
def is_goal(present, goal):
    dt = euclidean_distance((present.x, present.y), (goal.x, goal.y))             
    if dt <= 1.5:
        return True
    else:
        return False
    
# A* algorithm implementation
def a_star(start_position, goal_position, obstacle_space, step_size):                       
    if is_goal(start_position, goal_position):
        return None, 1

    goal = goal_position
    start = start_position
    
    moves = [move_up_60_deg, move_up_30_deg, zero_movement, move_down_60_deg, move_up_60_deg]   
    unexplored = {}  # Dictionary of all unexplored nodes
    
    start_coords = (start.x, start.y)  # Generating a unique key for identifying the node
    unexplored[start_coords] = start
    
    explored = {}  # Dictionary of all explored nodes
    all_nodes = []  # Stores all nodes that have been traversed, for visualization purposes.
    
    visited = set()  # Set to keep track of visited coordinates
    
    while unexplored:
        # Select the node with the lowest combined cost and heuristic estimate
        present_coords = min(unexplored, key=lambda k: unexplored[k].cost + unexplored[k].c2g)
        present_node = unexplored.pop(present_coords)
        
        all_nodes.append([present_node.x, present_node.y, present_node.theta])
        
        if is_goal(present_node, goal):
            goal.parent_id = present_node.parent_id
            goal.cost = present_node.cost
            print("Goal Node found")
            return all_nodes, 1

        explored[present_coords] = present_node

        for move in moves:
            x, y, theta, cost = move(present_node.x, present_node.y, present_node.theta, step_size, present_node.cost)
            c2g = euclidean_distance((x, y), (goal.x, goal.y))  
   
            new_node = Node(x, y, theta, cost, present_node, c2g)   
            new_coords = (new_node.x, new_node.y)
   
            if not is_valid(new_node.x, new_node.y, obstacle_space) or new_coords in explored:
                continue
            
            if new_coords in visited:
                # Skip adding the node if its coordinates have already been visited
                continue
            
            if new_coords not in unexplored:
                unexplored[new_coords] = new_node
                visited.add(new_coords)  # Add the new coordinates to the visited set
            elif new_node.cost < unexplored[new_coords].cost:
                unexplored[new_coords] = new_node
   
    return all_nodes, 0

# Function to backtrack and generate shortest path
def backtrack(goal_node):  
    x_path = []
    y_path = []
    x_path.append(goal_node.x)
    y_path.append(goal_node.y)

    parent_node = goal_node.parent_id
    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        parent_node = parent_node.parent_id
        
    x_path.reverse()
    y_path.reverse()
    
    return x_path, y_path

# Function to check if a line segment is valid
def is_valid_line(start, end, obs_space):
    # Bresenham's line algorithm to check for intersection between line segment and obstacles
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    err = dx - dy

    while True:
        if obs_space[y0][x0] == 1 or obs_space[y0][x0] == 2:  # Check for obstacle intersection
            return False
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return True

# Function to draw a hexagon
def draw_hexagon(screen, color, center_x, center_y, side_length):
    vertices = []
    angle_deg = 60
    angle_rad = math.radians(angle_deg)
    for i in range(6):
        x = center_x + side_length * math.cos(angle_rad * i + math.radians(30))  # Adding 30 degrees to start with vertex up
        y = center_y + side_length * math.sin(angle_rad * i + math.radians(30))
        vertices.append((x, y))
    pygame.draw.polygon(screen, color, vertices)

# Function to draw a hexagon with padding
def draw_padded_hexagon(screen, color, center_x, center_y, side_length, padding):
    enlarged_side_length = side_length + padding
    draw_hexagon(screen, color, center_x, center_y, enlarged_side_length)

# Function to draw C obstacle
def draw_C(screen, color):
    vertices = [(450, 225), (450, 187.5), (510, 187.5), (510, 62.5), (450, 62.5), (450, 25), (550, 25), (550, 225)]
    pygame.draw.polygon(screen, color, vertices)

# Function to draw a vector
def draw_vector(screen, color, start, end):
    BLUE = (0, 0, 255)
    pygame.draw.line(screen, color, start, end, width=1)
    pygame.draw.circle(screen, BLUE, end, 2)

# Function to plot the path
def plot_path(start_node, goal_node, x_path, y_path, all_nodes, clearance, frame_rate, step_size):
    BLUE = (0, 0, 255)
    RED = (255, 0, 0)
    WHITE = (255, 255, 255)
    LIGHT_GREY = (190, 190, 190)
    DARK_GREY = (100, 100, 100)

    padding = clearance
    center_x, center_y = 325, 125
    side_length = 75

    # Initialize Pygame and plot the map
    pygame.init()
    screen = pygame.display.set_mode((600, 250))
    clock = pygame.time.Clock()
    if not os.path.exists("frames"):
        os.makedirs("frames")
    
    frame_count = 0
    # Counter to keep track of frames

    running = True
    while running and frame_count < (len(all_nodes) + len(x_path) - 1) / 2:  # Terminate loop once all frames are saved
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill(LIGHT_GREY)
        padding_rect = pygame.Rect(padding, padding, 600 - 2 * padding, 250 - 2 * padding)
        pygame.draw.rect(screen, WHITE, padding_rect)
        draw_padded_hexagon(screen, LIGHT_GREY, center_x, center_y, side_length, padding)
        draw_hexagon(screen, DARK_GREY, center_x, center_y, side_length)
        cblock_vertices = [(450 - (clearance), 225 + (clearance)), (450 - (clearance), 187.5 - (clearance)), (510 - (clearance), 187.5 - (clearance)), (510 - (clearance), 62.5 + (clearance)), (450 - (clearance), 62.5 + (clearance)), 
                                 (450 - (clearance), 25 - (clearance)), (550 + (clearance), 25 - (clearance)), (550 + (clearance), 225 + (clearance))]
        pygame.draw.polygon(screen, LIGHT_GREY, cblock_vertices)
        draw_C(screen, DARK_GREY)
        pygame.draw.rect(screen, LIGHT_GREY, pygame.Rect(50 - clearance, 0, 37.5 + 2 * clearance, 200 + clearance))  # Rectangle1 Clearance 
        pygame.draw.rect(screen, LIGHT_GREY, pygame.Rect(137.5 - clearance, 50 - clearance, 37.5 + 2 * clearance, 200 + clearance))  # Rectangle2 Clearance 
        pygame.draw.rect(screen, DARK_GREY, pygame.Rect(50, 0, 37.5, 200))  # Rectangle1 Obstacle
        pygame.draw.rect(screen, DARK_GREY, pygame.Rect(137.5, 50, 37.5, 200))  # Rectangle2 Obstacle
        pygame.draw.rect(screen, RED, (start_node.x, 250 - start_node.y, 4, 4))  # Invert y-axis for start node
        pygame.draw.rect(screen, RED, (goal_node.x, 250 - goal_node.y, 4, 4))  # Invert y-axis for goal node
        for i in range(len(all_nodes) - 1):
            current_node = all_nodes[i]
            next_node = all_nodes[i + 1]
            start = (current_node[0], 250 - current_node[1])  # Invert y-axis for current node
            end = (current_node[0] + step_size * math.cos(math.radians(next_node[2])),
                   250 - (current_node[1] + step_size * math.sin(math.radians(next_node[2]))))  # Calculate end point based on step size and direction
            draw_vector(screen, (0, 255, 0), start, end)
            frame_count += 1
            if frame_count % 100 == 0:  # Save frame every 100th frame
                pygame.image.save(screen, os.path.join("frames", f"frame_{frame_count}.png"))
            pygame.display.update()
        for i in range(len(x_path) - 1):
            pygame.draw.line(screen, RED, (x_path[i], 250 - y_path[i]), (x_path[i + 1], 250 - y_path[i + 1]), width=4)
            pygame.image.save(screen, os.path.join("frames", f"frame_{frame_count}.png"))
            frame_count += 1
            pygame.display.update()

        clock.tick(frame_rate)  # Ensure frame rate

    pygame.quit()


def frames_to_video(frames_dir, output_video):
    frames = [img for img in os.listdir(frames_dir) if img.endswith(".png")]
    frames.sort(key=lambda x: int(x.split("_")[1].split(".")[0]))  # Sort frames by frame number
    frame = cv2.imread(os.path.join(frames_dir, frames[0]))
    height, width, layers = frame.shape
    # print("Creating Videowriter")
    video = cv2.VideoWriter(output_video, cv2.VideoWriter_fourcc(*'mp4v'), 35, (width, height))
    print("Writing Video")
    for frame in frames:
        video.write(cv2.imread(os.path.join(frames_dir, frame)))

    cv2.destroyAllWindows()
    video.release()


if __name__ == '__main__':
    width = 600
    height = 250
    s_t = 30
    g_t = 30
    c2g = 0
    
    # Taking start and end node coordinates as input from the user
    CLEARANCE = int(input("Enter the desired CLEARANCE: "))
    robot_radius = int(input("Enter the desired robot radius: "))
    robot_step_size = int(input("Enter the desired step size (1 - 10): "))
    print("Minimum x and y values will be addition of clearance (of wall) and robot radius:",CLEARANCE+robot_radius)
    start_input_x = input("Enter the Start X: ")
    start_input_y = input("Enter the Start Y: ")
    start_theta = int(input("Enter the Theta_Start: "))

    start_x = int(start_input_x)
    start_y = int(start_input_y)

    end_input_x = input("Enter the End X: ")
    end_input_y = input("Enter the End Y: ")
    end_theta = int(input("Enter the Theta_End: "))
    
    end_x = int(end_input_x)
    end_y = int(end_input_y)
    
    if start_theta % 30 != 0 or end_theta % 30 != 0:
        print("Please enter valid theta values. Theta should be a multiple of 30 degrees.")
        exit()

    if robot_step_size < 1 or robot_step_size > 10:
        print("Please enter a valid step size between 1 to 10 inclusive.")
        exit()

    print("Setting up Configuration space. Wait a few seconds....")
    obs_space = Configuration_space(width, height, robot_radius, CLEARANCE)
    # Define start and goal nodes
    start_point = Node(start_x, start_y, start_theta, 0.0, -1,c2g)  # Start node with cost 0 and no parent
    goal_point = Node(end_x, end_y, end_theta, 0.0, -1, c2g)  # You can adjust the goal node coordinates as needed
    save_dir = "frames"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    timer_begin = time.time()
    traversed_nodes, goal_found = a_star(start_point, goal_point, obs_space, robot_step_size)
    timer_end = time.time()
    print("Time taken to explore:", timer_end - timer_begin, "seconds")

    if goal_found:
        x_path, y_path = backtrack(goal_point)
        optimal_cost = goal_point.cost  # Cost of the optimal path
        print("Optimal path cost:", optimal_cost)
        plot_path(start_point, goal_point, x_path, y_path, traversed_nodes, CLEARANCE, frame_rate=30, step_size=robot_step_size)
        output_video = "output_video.mp4"
        print("Generating Video")
        frames_to_video(save_dir, output_video)
        print("Video created successfully!")
    else:
        print("Goal not found!")

