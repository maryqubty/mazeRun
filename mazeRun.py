import open3d as o3d 
import numpy as np
import tkinter as tk
from tkinter import Button
from tkinter import filedialog


# Prompt user to select a PLY file
file_path = filedialog.askopenfilename(
    title="Select a PLY File", 
    filetypes=[("PLY Files", "*.ply")]
)
if not file_path:
    print("No file selected. Exiting.")
    exit()
# Load the selected PLY file
mazeUrl = file_path
# Load the PLY file as a mesh
originalMaze = o3d.io.read_triangle_mesh(mazeUrl)
# Visualize the mesh
originalMaze.compute_vertex_normals()

# Sample point clouds from the mesh
mazePcd= originalMaze.sample_points_uniformly(number_of_points=50000)

#Now we're going to identify walls and open areas (roads) in a the maze based on depth differences
#Identify depth axis based on variance - for each different maze model:
point_coords = np.asarray(mazePcd.points)
axis_variances = np.var(point_coords, axis=0)
depth_axis = np.argmin(axis_variances)  # Axis with least variance is the depth
print("Depth axis:", depth_axis)
# Get depth values
mazeDepth = np.asarray(mazePcd.points)[:, depth_axis] #depth_axis is the depth axis
#save the sorted depth values in an output file - sorted
np.savetxt("sorted_depth_values.txt", np.sort(mazeDepth), delimiter=",")

# Analyze depth distribution to dynamically adjust thresholds
depth_values = np.asarray(mazePcd.points)[:, depth_axis]
min_depth, max_depth = np.min(depth_values), np.max(depth_values)

#Instead of fixed percentiles, analyze the depth distribution dynamically: Use clustering with KMeans to segment depth dynamically
from sklearn.cluster import KMeans

depth_values_reshaped = depth_values.reshape(-1, 1)
kmeans = KMeans(n_clusters=3, random_state=0).fit(depth_values_reshaped)
cluster_centers = sorted(kmeans.cluster_centers_.flatten())
walls_depth_threshold, road_depth_threshold, ground_depth_threshold = cluster_centers

print("Dynamically calculated thresholds:")
print("walls:", walls_depth_threshold)
print("Road:", road_depth_threshold)
print("Ground:", ground_depth_threshold)

'''
Axis Selection:
Axis 1 (e.g., Y in many systems): Depth values increase from bottom to top.
Axis 2 (e.g., Z in many systems): Depth values may increase in the opposite direction (top to bottom or front to back).
Axis 3 : Left-To-Right.
'''
#If the axis direction is inverted, invert the depth values before clustering:
if depth_axis == 2:  # Assuming Axis 2 needs to be inverted
    depth_values = -depth_values
# Map categories to ensure correct visualization
wall_points = point_coords[depth_values < walls_depth_threshold]


# Create a tkinter window
window = tk.Tk()
window.title("Maze Segmentation")
# Set the size of the window (width x height)
window.geometry("400x300")  # Example size, adjust as needed

# Function to visualize road points
def visualize_roads():
    if len(wall_points) == 0:
        print("No road points to display.")
    else:
        walls_pcd = o3d.geometry.PointCloud()
        walls_pcd.points = o3d.utility.Vector3dVector(wall_points)
        o3d.visualization.draw_geometries([walls_pcd], window_name="Show cloud-points of walls")


# Add buttons to the window
btn_walls = Button(window, text="Show cloud-points of walls", command=visualize_roads)
btn_walls.pack(pady=20)  # Adjust padding as needed


#second section of project:--------------------------------------------------------------------------
'''
To identify an open path from a given point to the exit of a 3D maze, I want to develop an algorithm using the depth information from the point cloud.
Key Concept:
Open Paths: These are regions where there are no cloud points (walls) blocking the way along the depth axis.
Planning to treat the problem as a graph traversal where:
Nodes: Represent regions of the maze.
Edges: Represent open connections between regions.
Exit: The farthest point along the depth axis.

'''
#Define the Search Space:
#Divide the maze into a grid or voxelized representation based on its dimensions. Each cell in the grid corresponds to a small region of the maze.
'''
Initialize a binary map where:
-> 1 represents a blocked cell (contains wall points).
-> 0 represents an open cell (no wall points).
'''

def create_grid(wall_points, grid_resolution, point_coords):
    print("Creating grid...")
    # Step 1: Define grid parameters
    x_min, x_max = point_coords[:, 0].min(), point_coords[:, 0].max()
    y_min, y_max = point_coords[:, 1].min(), point_coords[:, 1].max()
    z_min, z_max = point_coords[:, 2].min(), point_coords[:, 2].max()

    # Determine grid dimensions
    x_range = int((x_max - x_min) / grid_resolution)
    y_range = int((y_max - y_min) / grid_resolution)
    z_range = int((z_max - z_min) / grid_resolution)
    maze_grid = np.zeros((x_range, y_range, z_range))

    # Mark cells as blocked if they contain wall points
    for point in wall_points:
        x_idx = int((point[0] - x_min) / grid_resolution)
        y_idx = int((point[1] - y_min) / grid_resolution)
        z_idx = int((point[2] - z_min) / grid_resolution)
        maze_grid[x_idx, y_idx, z_idx] = 1

    print(f"Grid Dimensions: {maze_grid.shape}")
    print(f"Number of Walls: {np.sum(maze_grid == 1)}")
    print(f"Number of Open Cells: {np.sum(maze_grid == 0)}")

    return maze_grid


# Visualize the Voxelized Grid
def visualize_grid():
    np.save("maze_grid.npy", maze_grid)
    print("Grid saved to 'maze_grid.npy'. You can inspect it with numpy or other tools.")

    print("Visualizing the voxelized grid...")
    grid_points = []
    x_min, y_min, z_min = point_coords[:, 0].min(), point_coords[:, 1].min(), point_coords[:, 2].min()
    for x in range(maze_grid.shape[0]):
        for y in range(maze_grid.shape[1]):
            for z in range(maze_grid.shape[2]):
                if maze_grid[x, y, z] == 1:
                    grid_points.append([
                        x_min + x * grid_resolution,
                        y_min + y * grid_resolution,
                        z_min + z * grid_resolution,
                    ])
    
    grid_pcd = o3d.geometry.PointCloud()
    grid_pcd.points = o3d.utility.Vector3dVector(np.array(grid_points))
    o3d.visualization.draw_geometries([grid_pcd], window_name="Voxelized Grid")
    
# Add Grid Visualization Button
btn_visualize_grid = Button(window, text="Visualize Grid", command=visualize_grid)
btn_visualize_grid.pack(pady=20)

#Add a Buffer Around Walls
#Add a small buffer zone around blocked cells to account for wall thickness.
from scipy.ndimage import binary_dilation

def add_wall_buffer(maze_grid):
    print("Adding wall buffer...")
    return binary_dilation(maze_grid, structure=np.ones((3, 3, 3)))


#Implement Pathfinding:
#Use a pathfinding algorithm A* (A-Star)
from queue import PriorityQueue

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def find_path(start, goal, maze_grid):
    print("Finding path...")
    queue = PriorityQueue()
    queue.put((0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not queue.empty():
        _, current = queue.get()

        if current == goal:
            break

        for dx, dy, dz in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]:
            next_cell = (current[0] + dx, current[1] + dy, current[2] + dz)
            if (
                0 <= next_cell[0] < maze_grid.shape[0] and
                0 <= next_cell[1] < maze_grid.shape[1] and
                0 <= next_cell[2] < maze_grid.shape[2] and
                maze_grid[next_cell] == 0
            ):
                new_cost = cost_so_far[current] + 1
                if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                    cost_so_far[next_cell] = new_cost
                    priority = new_cost + heuristic(goal, next_cell)
                    queue.put((priority, next_cell))
                    came_from[next_cell] = current

    # Reconstruct the path
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

#start point and exit point selection:
# Function to select points manually through visualization
start_point = None
end_point = None
# Function to visualize and allow user to pick points
picked_points = []

# Function to visualize and allow user to pick points
def select_points():
    global mazePcd, picked_points
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Select Start and Exit Points")
    vis.add_geometry(mazePcd)
    print("Please select two points: Start and Exit. Press Q to confirm your selections.")
    
    # Run the interaction and check the state
    vis.run()  # Interactive picking
    print("Interaction completed. Attempting to retrieve picked points...")
    
    # Do not destroy the window for debugging
    # vis.destroy_window()
    
    # Retrieve picked points
    picked_indices = vis.get_picked_points()
    print(f"Picked Indices: {picked_indices}")
    
    picked_points = [mazePcd.points[idx] for idx in picked_indices]
    print(f"Picked Points: {picked_points}")
    
    if len(picked_points) >= 2:
        start_point = np.array(picked_points[0])
        end_point = np.array(picked_points[1])
        print(f"Start Point: {start_point}")
        print(f"Exit Point: {end_point}")
    else:
        print("Please select at least two points.")


btn_select_points = Button(window, text="Select Start and Exit Points", command=select_points)
btn_select_points.pack(pady=20)

#main:
#Create grid and add buffer
grid_resolution = 0.5  # Adjust as needed
maze_grid = create_grid(wall_points, grid_resolution, point_coords)
maze_grid = add_wall_buffer(maze_grid)


#this next part of code is gonna be changed to be dynamically chosen for exh different maze model by clicking on the start and exit point
# Step 3: Specify start and exit points (in grid indices)
start_point = (10, 10, 10)  # Example start point (adjust as needed)
print("Start Point:", start_point)
exit_point = (maze_grid.shape[0] - 2, maze_grid.shape[1] - 2, maze_grid.shape[2] - 2)  # Example exit point
print("Exit Point:", exit_point)

# Step 4: Find the path
#path = find_path(start_point, exit_point, maze_grid)

def find_and_visualize_path():
    path_coords = find_path(start_point, exit_point, maze_grid)
    print("Path found with", len(path_coords), "points.")
    print("Path Coordinates:", path_coords)
    # Visualize the path
    path_pcd = o3d.geometry.PointCloud()
    path_pcd.points = o3d.utility.Vector3dVector(path_coords)
    o3d.visualization.draw_geometries([mazePcd, path_pcd], window_name="Path Visualization")

btn_path = Button(window, text="Find Path", command=find_and_visualize_path)
btn_path.pack(pady=20)

# Run the tkinter main loop
window.mainloop()

#end of the code for now