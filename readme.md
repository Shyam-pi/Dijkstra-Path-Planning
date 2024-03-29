# Dijkstra
Implementation of Dijkstra for a given obstacle map with dynamic start positions and goal positions.

GITHUB LINK : https://github.com/Shyam-pi/Dijkstra

Note:

1. 'dijkstra.mp4' is a short demo of how to use the script - 'dijkstra.py' with the input start position as (6,6) and (440,125), in the form of (x,y) coordinates
2. 'dijkstra.gif' is a GIF generated using the same inputs as the previous point. However it's in 60 FPS and hence is much smoother.
3. In the obstacle map shown in the matplotlib window, black color elements indicate actual obstacle locations, the small gray parts indicate the bloated portion of the obstacles and walls to accomodate the size of the obstacle.
4. When the explored nodes get plotted in the map, one can see that the gray colors showing the bloated walls getting smudged a bit - this is NOT because of nodes being explored in the obstacle regions but rather due to the size of the dots used for plotting the explored nodes. (smaller size made the explored portion look sparse due to low resolution)

Steps to run:

1. Download the project folder and open it in VSCode. Run 'dijkstra.py' which is a standalone script.
2. Wait for a second or two for the program to determine the obstacle map.
3. After this, the user will be prompted to enter the x, followed by the y coordinates of the starting position. The user will keep on getting prompted to re-enter the values until valid coordinate values are entered. (The x and y values can't be negative, or outside the map or inside any of the obstacles given after bloating by 5mm)
4. Once the correct coordinates for start position are entered, the same needs to be done for the goal position.
5. Once this is done, according to the start and goal locations, wait for around '1 - 4 seconds' for the Dijkstra algorithm to get the optimal path, when a message saying "SUCCESS WOOHOO!" gets displayed on the screen along with the time taken to solve the same.
6. Post this, a matplotlib animation window opens up, showing the nodes explored in red color, in the order they were explored, followed by the optimal path in green.
7. If the algorithm is unable to solve for the optimal path using the start and goal points (Open list gets empty before reaching the goal node), the algorithm terminates, followed by a matplotlib window displaying the nodes explored.
8. If you don't want to display the animated plot, comment out 'plt.show()'
9. If you want to save the animation in the form of a GIF, uncomment the previous line 'anim.save('dijkstra.gif', writer = 'imagemagick', fps=60)'