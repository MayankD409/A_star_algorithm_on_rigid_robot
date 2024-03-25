# A\* Algorithm for Path Planning in Rigid Robot Environment

This repository contains Python code implementing the A\* algorithm for path planning in a rigid robot environment. The code utilizes Pygame for visualization and numpy for array operations.

## Overview

Path planning is a crucial aspect of robotics, where the objective is to find an optimal path for a robot to navigate from a starting point to a goal point while avoiding obstacles. The A\* algorithm is a popular choice for solving such path planning problems efficiently.

The provided Python script includes:

1. **Node Class**: Defines a node object to represent points in the configuration space.
2. **Action Functions**: Define actions that the robot can take and calculate the cost associated with each action.
3. **Configuration Space Construction**: Generates the configuration space with obstacles.
4. **A\* Algorithm Implementation**: Implements the A\* algorithm to find the optimal path from the start to the goal while avoiding obstacles.
5. **Path Backtracking**: Backtracks from the goal node to the start node to generate the optimal path.
6. **Visualization**: Utilizes Pygame to visualize the configuration space, explored nodes, and the optimal path.

## Usage

To use the code:

1. Clone the repository: `git clone https://github.com/MayankD409/A_star_algorithm_on_rigid_robot.git`
2. Install the required dependencies: `pygame`, `numpy`, `opencv-python`
3. Run the Python script: `python A_star_algorithm.py`
4. Follow the prompts to input the desired parameters such as clearance, robot radius, step size, start and end coordinates, and start and end orientations.

## Input Parameters

- **Clearance**: The clearance distance around obstacles.
- **Robot Radius**: The radius of the robot.
- **Step Size**: The step size for each movement of the robot.
- **Start Point**: The starting coordinates and orientation of the robot.
- **End Point**: The goal coordinates and orientation of the robot.

## Output

The script generates a visualization of the configuration space, explored nodes, and the optimal path from the start to the goal. Additionally, it outputs a video (`output_video.mp4`) demonstrating the path planning process.

## Example Usage

```bash
python A_star_algorithm.py
```

Group Details:
Member 1 : Mayank Deshpande ==> UID: 120387333
Member 2 : Tanmay Pancholi  ==> UID: 120116711
Member 3 : Suraj Kalwaghe   ==> UID: 120417634
