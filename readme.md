# Dijkstra Path Planning for Mobile Point Robot
Implementation of Dijkstra Path Planning algorithm for a mobile point robot with a given obstacle map, dynamic start positions and goal positions.

![dijkstra](https://github.com/Shyam-pi/Dijkstra-Path-Planning/assets/57116285/417b9f17-9ce9-4c7f-a5b4-2161575866f1)

#### Objective:
Implement the Dijkstra algorithm to find the shortest path for a mobile point robot from a start point to a goal point while avoiding obstacles in the environment.

#### Key Components:
1. **Node Class**:
   - Represents a node in the exploration tree.
   - Contains attributes such as parent node, state (position), index, and cost-to-come.
   - Provides methods to generate possible actions (movements) from the current node.

2. **Map Class**:
   - Manages the obstacle map and exploration tree.
   - Utilizes the Dijkstra algorithm to explore the environment and find the shortest path.
   - Provides methods to initialize the map, populate the exploration graph, and backtrack to find the optimal path.

3. **Visualization**:
   - Utilizes Matplotlib to visualize the obstacle map, explored path, and optimal path.
   - Displays the exploration process and final path through an animation.

#### Workflow:
1. **Map Initialization**:
   - Creates the obstacle map with defined obstacles.
   - Accepts user input for the start and goal positions.

2. **Exploration**:
   - Populates the exploration graph using the Dijkstra algorithm.
   - Explores possible actions from each node while considering obstacle constraints.
   - Continues until the goal node is reached or no solution is found.

3. **Backtracking**:
   - Traces back from the goal node to the start node to find the optimal path.

4. **Visualization**:
   - Displays the obstacle map, explored path, and optimal path using Matplotlib.
   - Animates the exploration process and pathfinding results for visualization.

#### Usage Guide for Dijkstra Path Planning Algorithm:

#### Note:
- **'dijkstra.mp4'** is a video demonstration of using the script **'dijkstra.py'** with predefined start positions of (6,6) and (440,125), specified as (x,y) coordinates.
- **'dijkstra.gif'** is a GIF generated using the same inputs as 'dijkstra.mp4' but at 60 FPS, resulting in smoother animation.
- In the obstacle map displayed in the Matplotlib window, black elements represent actual obstacle locations, while gray parts indicate the bloated portions of obstacles and walls to accommodate obstacle size.
- The slight smudging of gray areas in the explored nodes plot is due to the dot size used for plotting, not due to exploration within obstacle regions.

#### Steps to Run the Algorithm:
1. **Download and Setup**:
   - Download the project folder and open it in a Python IDE like VSCode.
   - Run the script **'dijkstra.py'**, which is a standalone script.

2. **Map Initialization**:
   - Wait for a second or two for the program to determine the obstacle map.

3. **Input Start Position**:
   - Enter the x and y coordinates of the starting position when prompted.
   - Re-enter values until valid coordinates are provided.
   - The coordinates must not be negative, outside the map boundaries, or inside any obstacles after bloating by 5mm.

4. **Input Goal Position**:
   - Repeat the process for entering the goal position coordinates.

5. **Path Planning**:
   - Wait for approximately 1 to 4 seconds for the Dijkstra algorithm to find the optimal path.
   - Upon successful pathfinding, a message "SUCCESS WOOHOO!" will be displayed, along with the time taken to find the solution.

6. **Visualization**:
   - A Matplotlib animation window will open, showing the explored nodes in red color and the optimal path in green.
   - If the algorithm fails to find a solution, the exploration nodes will be displayed in a Matplotlib window.
   - To save the animation as a GIF, uncomment the line 'anim.save('dijkstra.gif', writer='imagemagick', fps=60)'.

7. **Additional Notes**:
   - To skip displaying the animated plot, comment out 'plt.show()'.
   - Ensure to follow the instructions and provide valid inputs for a smooth execution of the algorithm.

#### Conclusion:
The Dijkstra path planning algorithm efficiently finds the shortest path for a mobile point robot while navigating through obstacle-rich environments. It serves as a fundamental technique for robot motion planning in various real-world scenarios.
