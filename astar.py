#!/usr/bin/env python3

from PIL import Image, ImageDraw
import math
import time
try:
    import queue
except ImportError:
    import Queue as queue
import sys
import os.path
import pickle
from discretemap import *

###################################
# BEGIN Global Variable Definitions
dmap = None  # This is our discrete map
# END Global Variable Definitions
###################################

# This class will be used in our search
class SearchNode:
    def __init__(self, state, parent, cost):
        self.parent = parent  # Pointer to parent node (will be None for root node)
        self.state = state  # The state (grid cell) this node points to
        self.cost = cost  # The cumulative cost of this search node (sum of all actions to get here)
        self.h = 0  # This node's heuristic value (must be set separately)
    def __lt__(self, other):
        return self.h < other.h

# This function returns the Euclidean distance between two grid cells
def euclidean_distance_grid(a, b):
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    return d

# This function sets up everything we need for the search and runs it
# chat gpt helped to get this working 
def run_search(start, goal, search_type):
    # Create the start node
    start_node = SearchNode(start, None, 0)
    start_node.h = euclidean_distance_grid(start, goal)

    # Create Fringe (Priority Queue)
    fringe = queue.PriorityQueue()
    priority = start_node.cost + start_node.h if search_type == "A*" else (start_node.h if search_type == "Greedy" else start_node.cost)
    fringe.put((priority, start_node))

    print(f"Starting {search_type} search from grid cell {start_node.state} to goal cell {goal}")
    print(f"Starting priority is {priority}")

    # Run the search
    goal_node, expansions = a_star(fringe, goal, search_type)

    # Extract path from the goal_node and return it
    path = []
    if goal_node is not None:
        print("Found a solution!")
        cur_node = goal_node
        while cur_node is not None:
            path.append(cur_node.state)
            cur_node = cur_node.parent
        path.reverse()

    return path, expansions

# This is the main search function that performs A*, Greedy, or Uniform Cost search
def a_star(fringe, goal, search_type):
    closed = []  # This keeps track of which grid cells we have already visited
    expansions = 0  # This keeps track of the number of expansions

    # Stay in loop as long as we have unexpanded nodes
    while not fringe.empty():
        # Get the node out of the fringe. Format of items in the fringe is (priority, searchNode)
        current_node = fringe.get()[1]  # This gets the node to expand

        # Make sure that we didn't already expand a node pointing to this state
        if current_node.state not in closed:
            expansions += 1  # Increment our expansions counter

            # Test for the goal
            if current_node.state == goal:
                # We found it! Return the goal node.
                print(f"Found the goal after {expansions} node expansions")
                return current_node, expansions

            # Add expanded node's state to the closed list
            closed.append(current_node.state)

            # Get neighbors of the current node
            neighbors = get_neighbors(current_node.state, closed)

            # For each neighbor (a grid cell tuple(gx,gy))
            for neighbor_cell in neighbors:
                # Compute the cost to get to neighbor_cell from the current cell
                step_cost = euclidean_distance_grid(current_node.state, neighbor_cell)
                
                # Compute the cumulative cost to get to neighbor_cell
                cumulative_cost = current_node.cost + step_cost

                # Create a new SearchNode object for it
                neighbor_node = SearchNode(neighbor_cell, current_node, cumulative_cost)
                
                # Compute the new node's heuristic value
                neighbor_node.h = euclidean_distance_grid(neighbor_cell, goal)
                
                # Compute the priority based on search type
                if search_type == "A*":
                    priority = neighbor_node.cost + neighbor_node.h
                elif search_type == "Greedy":
                    priority = neighbor_node.h
                elif search_type == "Uniform":
                    priority = neighbor_node.cost

                # Add it to the fringe, with proper priority
                fringe.put((priority, neighbor_node))

    # We have exhausted all possible paths. No solution found
    print(f"Didn't find a path after {expansions} expansions")
    return None, expansions

# This function should return a list of the grid cells adjacent to g.
def get_neighbors(g, closed):
    global dmap
    neighbors = []

    # Define all possible movements (up, down, left, right, and diagonals)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1),  # Left, Right, Up, Down
                  (-1, -1), (-1, 1), (1, -1), (1, 1)]  # Diagonal neighbors

    for dx, dy in directions:
        neighbor_x = g[0] + dx
        neighbor_y = g[1] + dy
        neighbor = (neighbor_x, neighbor_y)

        # Check if the neighbor is within grid bounds
        if 0 <= neighbor_x < dmap.grid_width and 0 <= neighbor_y < dmap.grid_height:
            # Check if the neighbor is not in the closed list and not occupied
            if neighbor not in closed and neighbor not in dmap.occupied:
                neighbors.append(neighbor)

    return neighbors

if __name__ == '__main__':
    # Ensure a .world file is provided as command-line argument
    if len(sys.argv) < 2:
        print("Usage: python ai_labs/astar.py <world/world.world>")
        sys.exit(1)

    # Load the map from the .world file
    world_file = sys.argv[1]

    dmap = DiscreteMap(world_file, 5)
    dmap.expand_obstacles(0.175)

    # Convert start and goal to grid coordinates
    start = dmap.map_to_grid(dmap.start)
    goal = dmap.map_to_grid(dmap.goal)

    # Run each search type and display/save results
    search_types = ["A*", "Greedy", "Uniform"]
    for search_type in search_types:
        print(f"\nRunning {search_type} search on {world_file}")

        # Run the search
        start_time = time.time()
        path, expansions = run_search(start, goal, search_type)
        end_time = time.time()
        duration = end_time - start_time

        # Calculate solution quality if a solution was found
        solution_quality = 0.0
        if path:
            for i in range(1, len(path)):
                solution_quality += euclidean_distance_grid(path[i-1], path[i])

        # Print out results
        print(f"\nSearch Type: {search_type}")
        print(f"Solution Quality: {solution_quality}")
        print(f"Time: {duration} seconds")
        print(f"Number of Expansions: {expansions}")
        #print(f"Path: {path}")

        # Save an image of the path
        display_image_file = world_file[:-6] + f"_{search_type}_path.png"
        dmap.display_path(path, display_image_file)
        print(f'Saved {search_type} path to file: {display_image_file}\n')

