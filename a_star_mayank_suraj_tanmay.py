
#!/usr/bin/env python3
# Github repository:  https://github.com/MayankD409/Dijkstra_Point_Robot.git

import pygame
import numpy as np
import time
import heapq
import math
import os
import cv2

########## DEFINING A NODE CLASS TO STORE NODES AS OBJECTS ###############

class Node:
    def __init__(self, x, y, theta, cost, parent_id):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost
        self.parent_id = parent_id
    
    def __lt__(self, other):
        return self.cost < other.cost

########### DEFINING ACTIONS TO BE PERFORMED ##############
########### CALCULATING COST TO COME FOR ALL ACTIONS ########

def move_60up(x, y, theta, step_size, cost):
    theta_rad = np.radians(theta)
    new_x = x + step_size * np.cos(theta_rad + np.radians(60))
    new_y = y + step_size * np.sin(theta_rad + np.radians(60))
    new_x = round(new_x)
    new_y = round(new_y)
    cost = 1 + cost
    return new_x, new_y, theta + 60, cost

def move_30up(x, y, theta, step_size, cost):
    theta_rad = np.radians(theta)
    new_x = x + step_size * np.cos(theta_rad + np.radians(30))
    new_y = y + step_size * np.sin(theta_rad + np.radians(30))
    new_x = round(new_x)
    new_y = round(new_y)
    cost = 1 + cost
    return new_x, new_y, theta + 30, cost

def move_0(x, y, theta, step_size, cost):
    theta_rad = np.radians(theta)
    new_x = x + step_size * np.cos(theta_rad)
    new_y = y + step_size * np.sin(theta_rad)
    new_x = round(new_x)
    new_y = round(new_y)
    cost = 1 + cost
    return new_x, new_y, theta, cost

def move_30down(x, y, theta, step_size, cost):
    theta_rad = np.radians(theta)
    new_x = x + step_size * np.cos(theta_rad - np.radians(30))
    new_y = y + step_size * np.sin(theta_rad - np.radians(30))
    new_x = round(new_x)
    new_y = round(new_y)
    cost = 1 + cost
    return new_x, new_y, theta - 30, cost

def move_60down(x, y, theta, step_size, cost):
    theta_rad = np.radians(theta)
    new_x = x + step_size * np.cos(theta_rad - np.radians(60))
    new_y = y + step_size * np.sin(theta_rad - np.radians(60))
    new_x = round(new_x)
    new_y = round(new_y)
    cost = 1 + cost
    return new_x, new_y, theta - 60, cost

# Define a heuristic function for A*
def heuristic(node, goal):
    # Using Euclidean distance as the heuristic
    dx = node.x - goal.x
    dy = node.y - goal.y
    return math.sqrt(dx*dx + dy*dy)

############ CONFIGURATION SPACE CONSTRUCTION WITH OBSTACLES ############
def Configuration_space(width,height, robot_radius):
    obs_space = np.full((height, width),0)
    
    for y in range(0, height) :
        for x in range(0, width):
            
            rect_1_1_buffer = (x + 5 + robot_radius) - 50  
            rect_1_2_buffer = (y + 5 + robot_radius) - 50
            rect_1_3_buffer = (x - 5 - robot_radius) - 87.5
            rect_1_3_bffer = (y - 5 - robot_radius) - 250    

            rect_2_1_buffer = (x + 5 + robot_radius) - 137.5  
            rect_2_2_buffer = (y + 5 + robot_radius) - 0  
            rect_2_3_buffer = (x - 5 - robot_radius) - 175
            rect_2_4_buffer = (y - 5 - robot_radius) - 200 
     
            hexagon_6_b = (y + 5 + robot_radius) + 0.58*(x + 5 +robot_radius) - 237.549
            hexagon_5_b = (y + 5 + robot_radius) - 0.58*(x - 5 - robot_radius) + 137.501
            hexagon_4_b = (x - 6.5 - robot_radius) - 389.95
            hexagon_3_b = (y - 5 - robot_radius) + 0.58*(x - 5 - robot_radius) - 387.501
            hexaagon_2_b = (y - 5 - robot_radius) - 0.58*(x + 5 + robot_radius) - 12.46
            hexagon_1_b = (x + 6.5 + robot_radius) - 260.05
       
            temp1_b = (x + 5 + robot_radius) - 225
            temp2_b = (x + 5 + robot_radius) - 510
            temp3_b = (x - 5 - robot_radius) - 550
            temp4_b = (y + 5 + robot_radius) - 25
            temp5_b = (y - 5 - robot_radius) - 62.5
            temp6_b = (y + 5 + robot_radius) - 187.5
            temp7_b = (y - 5 - robot_radius) - 225
           
            if((temp1_b>0 and temp2_b<0 and temp4_b>0 and temp5_b<0) or(temp2_b>0 and temp3_b<0 and temp4_b>0 and temp7_b<0) or (temp6_b>0 and temp7_b<0 and temp1_b>0 and temp2_b<0) or (rect_1_1_buffer>0 and rect_1_2_buffer>0 and rect_1_3_buffer<0 and rect_1_3_bffer<0) or (rect_2_1_buffer>0 and rect_2_3_buffer<0 and rect_2_4_buffer<0 and rect_2_2_buffer>0) or (hexagon_6_b>0 and hexagon_5_b>0 and hexagon_4_b<0 and hexagon_3_b<0 and hexaagon_2_b<0 and hexagon_1_b>0)):
                obs_space[y, x] = 1
             
            
            winidow_1 = (y) - 5
            window_2 = (y) - 495
            window_3 = (x) - 5
            window_4 = (x) - 1195 

           
            rect_2_1 = (x) - 137.5  
            rect_2_2 = (y) - 0
            rect_2_4 = (x) - 175
            rect_2_3 = (y) - 200 
           
            rect_11 = (x) - 50  
            rect_12 = (y) - 50
            rect_13 = (x) - 87.5
            rect_14 = (y) - 250
            
          
            h6 = (y) + 0.58*(x) - 237.549
            h5 = (y) - 0.58*(x) + 137.501
            h4 = (x) - 389.95
            h3 = (y) + 0.58*(x) - 387.501
            h2 = (y) - 0.58*(x) - 12.46
            h1 = (x) - 260.05 
            
        
            t1 = (x) - 450
            t2 = (x) - 510
            t3 = (x) - 550
            t4 = (y) - 25
            t5 = (y) - 62.5
            t6 = (y) - 187.5
            t7 = (y) - 225

          
            if((h6>0 and h5>0 and h4<0 and h3<0 and h2<0 and h1>0) or (rect_11>0 and rect_12>0 and rect_13<0 and rect_14<0 ) or (rect_2_1>0  and rect_2_3<0 and rect_2_4<0 and rect_2_2>0) or (t1>0 and t2<0 and t4>0 and t5<0) or (t2>0 and t3<0 and t4>0 and t7<0) or (t6>0 and t7<0 and t1>0 and t2<0) or (winidow_1<0) or (window_2>0) or (window_3<0) or (window_4>0)):
                obs_space[y, x] = 2
                
    ####### CLEARANCE FOR THE WALLS ########
    for i in range(height):
        for j in range(6):
            obs_space[i][j] = 1
            obs_space[i][width - j - 1] = 1
    
    for i in range(width):
        for j in range(6):  # Mark the first 5 columns of the top and bottom boundaries as unreachable
            obs_space[j][i] = 1
            obs_space[height - j - 1][i] = 1  # Also mark the first 5 columns of the bottom boundary as unreachable

    return obs_space

############## CHECK IF THE GIVEN MOVE IS VALID OR NOT ###############

def is_valid(x, y, obs_space):
    height, width = obs_space.shape
    if x < 0 or x >= width or y < 0 or y >= height or obs_space[y][x] == 1 or obs_space[y][x] == 2:
        return False
    return obs_space[y, x] == 0

############## CHECK IF THE GOAL NODE IS REACHED ###############

def is_goal(present, goal):
    if (present.x == goal.x) and (present.y == goal.y):
        return True
    else:
        return False


# A* algorithm implementation
def astar(start_node, goal_node, obs_space, theta, step_size):
    if is_goal(start_node, goal_node):
        return None, 1

    possible_moves = [move_30up, move_30down, move_0, move_60down, move_60up]
    open_nodes = {(start_node.x, start_node.y): start_node}
    closed_nodes = {}
    priority_queue = []
    heapq.heappush(priority_queue, [start_node.cost + heuristic(start_node, goal_node), start_node])

    traversed_nodes = []
    
    while priority_queue:
        current_node = heapq.heappop(priority_queue)[1]
        traversed_nodes.append([current_node.x, current_node.y])
        current_node_coords = (current_node.x, current_node.y)

        if is_goal(current_node, goal_node):
            goal_node.parent_id, goal_node.cost = current_node.parent_id, current_node.cost
            print("Goal Node found")
            return traversed_nodes, 1

        if current_node_coords in closed_nodes:
            continue

        closed_nodes[current_node_coords] = current_node
        for move in possible_moves:
            new_x, new_y, new_theta, new_cost = move(current_node.x, current_node.y, current_node.theta, step_size, current_node.cost)
            new_node = Node(new_x, new_y, new_theta, new_cost, current_node)
            new_node_coords = (new_node.x, new_node.y)

            if is_valid(new_node.x, new_node.y, obs_space) and new_node_coords not in closed_nodes:
                new_node_cost = new_node.cost + heuristic(new_node, goal_node)
                if new_node_coords in open_nodes:
                    if new_node_cost < open_nodes[new_node_coords].cost + heuristic(open_nodes[new_node_coords], goal_node):
                        open_nodes[new_node_coords].cost = new_node.cost
                        open_nodes[new_node_coords].parent_id = new_node.parent_id
                        # Update the priority queue
                        heapq.heappush(priority_queue, [new_node_cost, open_nodes[new_node_coords]])
                else:
                    open_nodes[new_node_coords] = new_node
                    heapq.heappush(priority_queue, [new_node_cost, new_node])

    return traversed_nodes, 0



########### BACKTRACK AND GENERATE SHORTEST PATH ############

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
    
    return x_path,y_path

#########  PLOT OBSTACLES SPACE, EXPLORED NODES, SHORTEST PATH  #######

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

def plot_path(start_node, goal_node, x_path, y_path, all_nodes, frame_rate):
    BLUE = (0, 0, 255)
    RED = (255, 0, 0)
    WHITE = (255, 255, 255)
    LIGHT_GREY = (190, 190, 190)
    DARK_GREY = (100, 100, 100)

    padding = 5
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
    while running and frame_count < len(all_nodes) + len(x_path) - 1:  # Terminate loop once all frames are saved
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        screen.fill(LIGHT_GREY)
        padding_rect = pygame.Rect(padding, padding, 600 - 2 * padding, 250 - 2 * padding)
        pygame.draw.rect(screen, WHITE, padding_rect)
        draw_padded_hexagon(screen, LIGHT_GREY, center_x, center_y, side_length, padding)
        draw_hexagon(screen, DARK_GREY, center_x, center_y, side_length)
        cblock_vertices = [(447.5, 227.5), (447.5, 185), (507.5, 185), (507.5, 65), (447.5, 65), (447.5, 22.5), (552.5, 22.5), (552.5, 227.5)]
        pygame.draw.polygon(screen, LIGHT_GREY, cblock_vertices)
        draw_C(screen, DARK_GREY)
        pygame.draw.rect(screen, LIGHT_GREY, pygame.Rect(47.5, 0, 42.5, 202.5)) # Rectangle1 Clearance 
        pygame.draw.rect(screen, LIGHT_GREY, pygame.Rect(135, 47.5, 42.5, 202.5)) # Rectangle1 Obstacle 
        pygame.draw.rect(screen, DARK_GREY, pygame.Rect(50, 0, 37.5, 200)) # Rectangle2 Clearance
        pygame.draw.rect(screen, DARK_GREY, pygame.Rect(137.5, 50, 37.5, 200)) # Rectangle2 Obstacle

        pygame.draw.rect(screen, RED, (start_node.x, 250 - start_node.y, 10, 10))  # Invert y-axis for start node
        pygame.draw.rect(screen, RED, (goal_node.x, 250 - goal_node.y, 10, 10))  # Invert y-axis for goal node

        for node in all_nodes:
            pygame.draw.rect(screen, (190, 190, 0), (node[0], 250 - node[1], 1, 1))  # Invert y-axis for explored nodes
            frame_count += 1
            # 3if frame_count % 250 == 0:  # Save frame every 100th frame
            pygame.image.save(screen, os.path.join("frames", f"frame_{frame_count}.png"))
            pygame.display.update()

        for i in range(len(x_path) - 1):
            pygame.draw.line(screen, BLUE, (x_path[i], 250 - y_path[i]), (x_path[i + 1], 250 - y_path[i + 1]), width=4)
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
    print("Creating Videowriter")
    video = cv2.VideoWriter(output_video, cv2.VideoWriter_fourcc(*'mp4v'), 60, (width, height))
    print("Writing Video")
    for frame in frames:
        video.write(cv2.imread(os.path.join(frames_dir, frame)))

    cv2.destroyAllWindows()
    video.release()

if __name__ == '__main__':
    width = 600
    height = 250
    robot_radius = 5
    print("Wait few seconds for the input prompt...")
    obs_space = Configuration_space(width, height, robot_radius)
    
    # Taking start and end node coordinates as input from the user
    # start_input_x = input("Enter the Start X: ")
    # start_input_y = input("Enter the Start Y: ")

    # start_x = int(start_input_x)
    # start_y = int(start_input_y)

    # # end_input_x = input("Enter the End X: ")
    # # end_input_y = input("Enter the End Y: ")

    # end_x = int(end_input_x)
    # end_y = int(end_input_y)

    start_x = 10
    start_y = 10

    # end_input_x = input("Enter the End X: ")
    # end_input_y = input("Enter the End Y: ")

    end_x = 200
    end_y = 200

    # Ask user for theta and step size
    theta = float(input("Enter the angle theta: "))
    step_size = float(input("Enter the step size: "))

    # Define start and goal nodes
    start_point = Node(start_x, start_y, theta, 0, -1)  # Start node with cost 0 and no parent
    goal_point = Node(end_x, end_y, theta, 0, -1)  # You can adjust the goal node coordinates as needed

    save_dir = "frames"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    timer_begin = time.time()
    # Pass theta and step_size to astar function
    traversed_nodes, goal_found = astar(start_point, goal_point, obs_space, theta, step_size)
    timer_end = time.time()
    print("Time taken to explore:", timer_end - timer_begin, "seconds")

    if goal_found:
        x_path, y_path = backtrack(goal_point)
        optimal_cost = goal_point.cost  # Cost of the optimal path
        print("Optimal path cost:", optimal_cost)
        plot_path(start_point, goal_point, x_path, y_path, traversed_nodes, frame_rate=60)
        output_video = "output_video.mp4"
        print("Generating Video")
        frames_to_video(save_dir, output_video)
        print("Video created successfully!")
    else:
        print("Goal not found!")
        




