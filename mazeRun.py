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

#If the axis direction is inverted, invert the depth values before clustering:
if depth_axis == 2:  # Assuming Axis 2 needs to be inverted
    depth_values = -depth_values
    
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

# Map categories to ensure correct visualization
wall_points = point_coords[depth_values < road_depth_threshold]

# Create a tkinter window
window = tk.Tk()
window.title("Maze Segmentation")
# Set the size of the window (width x height)
window.geometry("400x500")  # Example size, adjust as needed

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
'''

# Function to get the grid index of a point
def get_grid_indice(point, grid_resolution, point_coords):
    x_min = point_coords[:, 0].min()
    y_min = point_coords[:, 1].min()
    z_min = point_coords[:, 2].min()
    x_idx = int((point[0] - x_min) / grid_resolution)
    y_idx = int((point[1] - y_min) / grid_resolution)
    z_idx = int((point[2] - z_min) / grid_resolution)
    point_indice = tuple([x_idx, y_idx, z_idx])
    return point_indice

'''
# Visualize the Voxelized Grid
def visualize_grid():
    np.savetxt("maze_grid.txt", maze_grid.reshape(-1, maze_grid.shape[2]), fmt="%d", delimiter=",")
    print("Grid saved to 'maze_grid.txt'.")

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
'''

# Function to visualize the 2D grid and select a point
def visualize_2d_grid():
    global s_point
    print("Visualizing the 2D grid...")

    # Flatten the 3D grid along the depth axis to create a 2D representation
    #maze_grid_2d = np.any(maze_grid, axis=depth_axis).astype(int)
    
    # Generate grid points (both walls and walkable)
    grid_points_2d = []
    colors = []
    x_min, y_min = point_coords[:, 0].min(), point_coords[:, 1].min()
    for x in range(maze_grid_2d.shape[0]):
        for y in range(maze_grid_2d.shape[1]):
            grid_points_2d.append([x_min + x * grid_resolution, y_min + y * grid_resolution])
            # Color the walls black and the paths white 
            if maze_grid_2d[x, y] == 1:  # Wall
                colors.append([0, 0, 0])  # Black
            else:  # Path (walkable)
                colors.append([1, 1, 1])  # White
    
    # Convert 2D grid points to a PointCloud
    grid_pcd_2d = o3d.geometry.PointCloud()
    grid_pcd_2d.points = o3d.utility.Vector3dVector(
        np.array([[point[0], point[1], 0] for point in grid_points_2d])  # Set depth to 0
    )
    grid_pcd_2d.colors = o3d.utility.Vector3dVector(np.array(colors))  # Assign colors

    # Visualize the 2D grid as points
    #o3d.visualization.draw_geometries([grid_pcd_2d], window_name="2D Grid Visualization")
       
    # Use Open3D's VisualizerWithEditing for interactive selection
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=f"Select Point")
    vis.add_geometry(grid_pcd_2d)
    vis.run()
    vis.destroy_window()

    # Get the selected points
    selected_indices = vis.get_picked_points()
    if selected_indices:
        selected_idx = selected_indices[-1]  # Make sure to get the last selected point
        selected_point = np.asarray(grid_pcd_2d.points)[selected_idx]
         # Extract the 2D point by ignoring the z coordinate (removing depth of grid)
        selected_point_2d = selected_point[:2]  # Just take the first two elements (x, y)
        selected_point_2d = np.round(selected_point_2d )   # Round the values to the nearest integer
        print(f"2D Point Selected: {selected_point_2d}")
        s_point = tuple(selected_point_2d) # Store the selected point as a tuple for pathfinding later
    else:
        print(f"No point selected.")
        s_point = None

# Add 2D Grid Visualization Button
btn_visualize_2d_grid = Button(window, text="Visualize 2D Grid", command=visualize_2d_grid)
btn_visualize_2d_grid.pack(pady=20)


s_point = None  # Global variable to store the last selected point
def visualize_point_cloud_with_selection():
    global s_point
    print("Visualizing the point cloud with interactive point selection...")

    # Use Open3D's VisualizerWithEditing for interactive selection
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Select Points in Point Cloud")
    vis.add_geometry(mazePcd)  # Use the point cloud directly
    vis.run()
    vis.destroy_window()

    # Get the selected points
    selected_indices = vis.get_picked_points()
    if selected_indices:
        selected_idx = selected_indices[-1]  # Use the last selected point
        selected_point = np.asarray(mazePcd.points)[selected_idx]
        print(f"Point Cloud Point Selected: {selected_point}")
        s_point = selected_point  # Store the selected point for pathfinding later
    else:
        print("No point selected.")
        s_point = None

# Add Point Cloud Visualization Button
btn_visualize_point_cloud = Button(window, text="Visualize Point Cloud", command=visualize_point_cloud_with_selection)
btn_visualize_point_cloud.pack(pady=20)



# Function to select the start point
def select_start_point():
    global s_point, start_point
    #call the function to visualize the 3D grid and select a point
    print("Select the start point...")
    visualize_point_cloud_with_selection()
    # Remove the depth axis coordinate from the selected point
    indice= get_grid_indice(s_point, grid_resolution, point_coords)
    selected_point_2d = np.delete(indice, depth_axis)

    #check if point s valid and walkable:
    if selected_point_2d is not None: 
        print(f"Start Point Selected: {selected_point_2d}")
        start_point = tuple(selected_point_2d)
    else:
        print("No start point selected.")
    s_point = None # Reset the selected point

# Add button to select start point
btn_select_start = Button(window, text="Select Start Point", command=select_start_point)
btn_select_start.pack(pady=20)


# Function to select the exit point
def select_exit_point():
    global s_point, exit_point
    print("Select the exit point...")
    visualize_point_cloud_with_selection()
    # Remove the depth axis coordinate from the selected point
    indice= get_grid_indice(s_point, grid_resolution, point_coords)
    selected_point_2d = np.delete(indice, depth_axis)

    if selected_point_2d is not None:
        print(f"Exit Point Selected: {selected_point_2d}")
        exit_point = tuple(selected_point_2d)
    else:
        print("No exit point selected.")
    s_point = None  # Reset the selected point

# Add button to select exit point
btn_select_exit = Button(window, text="Select Exit Point", command=select_exit_point)
btn_select_exit.pack(pady=20)

'''
#Add a Buffer Around Walls
#Add a small buffer zone around blocked cells to account for wall thickness.
from scipy.ndimage import binary_dilation

def add_wall_buffer(maze_grid):
    print("Adding wall buffer...")
    return binary_dilation(maze_grid, structure=np.ones((3, 3, 3)))
'''

#Implement Pathfinding:
#Use a pathfinding algorithm A* (A-Star)
from queue import PriorityQueue

# Heuristic function
def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

# Pathfinding function
def find_path_2d(start, goal, maze_grid_2d, depth_axis):
    print("find_path_2d... start = {start}, goal = {goal}")
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

        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            next_cell = (current[0] + dx, current[1] + dy)
            if (
                0 <= next_cell[0] < maze_grid_2d.shape[0] and
                0 <= next_cell[1] < maze_grid_2d.shape[1] and
                maze_grid_2d[next_cell] == 0
            ):
                new_cost = cost_so_far[current] + 1
                if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                    cost_so_far[next_cell] = new_cost
                    priority = new_cost + heuristic(goal, next_cell)
                    queue.put((priority, next_cell))
                    came_from[next_cell] = current

    path = []
    current = goal
    while current is not None:
        path.append(current)
        if current not in came_from:
            print(f"Path reconstruction failed at {current}. Partial path: {path}")
            break
        current = came_from[current]
    path.reverse()
    return path




'''
#Create grid and add buffer
grid_resolution = 0.5  # Adjust as needed
maze_grid = create_grid(wall_points, grid_resolution, point_coords)
maze_grid = add_wall_buffer(maze_grid)

# Collapse the grid along the depth axis to create a 2D grid
maze_grid_2d = np.any(maze_grid, axis=depth_axis).astype(int)
'''

from scipy.ndimage import binary_dilation

def create_2d_grid(wall_points, grid_resolution, point_coords, depth_axis):
    print("Creating 2D grid...")

    # Step 1: Define grid parameters for 2D (ignoring depth)
    axes = [i for i in range(3) if i != depth_axis]
    x_axis, y_axis = axes
    x_min, x_max = point_coords[:, x_axis].min(), point_coords[:, x_axis].max()
    y_min, y_max = point_coords[:, y_axis].min(), point_coords[:, y_axis].max()

    # Determine 2D grid dimensions
    x_range = int((x_max - x_min) / grid_resolution)
    y_range = int((y_max - y_min) / grid_resolution)
    maze_grid_2d = np.zeros((x_range, y_range), dtype=int)

    # Mark cells as blocked if they contain wall points
    for point in wall_points:
        x_idx = int((point[x_axis] - x_min) / grid_resolution)
        y_idx = int((point[y_axis] - y_min) / grid_resolution)
        if 0 <= x_idx < x_range and 0 <= y_idx < y_range:
            maze_grid_2d[x_idx, y_idx] = 1

    print(f"2D Grid Dimensions: {maze_grid_2d.shape}")
    print(f"Number of Walls: {np.sum(maze_grid_2d == 1)}")
    print(f"Number of Open Cells: {np.sum(maze_grid_2d == 0)}")

    return maze_grid_2d

def add_wall_buffer_2d(maze_grid_2d):
    print("Adding wall buffer to 2D grid...")
    return binary_dilation(maze_grid_2d, structure=np.ones((3, 3))).astype(int)

# Example usage:
grid_resolution = 0.5  # Adjust as needed

maze_grid_2d = create_2d_grid(wall_points, grid_resolution, point_coords, depth_axis)
maze_grid_2d = add_wall_buffer_2d(maze_grid_2d)



# Step 3: Specify start and exit points (in grid indices) (default values):
# Find coordinates where the grid value is 0 (walkable)
walkable_points = np.column_stack(np.where(maze_grid_2d == 0))
blocked_points = np.column_stack(np.where(maze_grid_2d == 1))
# Pick the first and last walkable points
start_point = (0,0) 
exit_point = (0,0)
print("Start Point (2D):", start_point)
print("Exit Point (2D):", exit_point)

# Step 4: Find the path
#path = find_path(start_point, exit_point, maze_grid)
def find_and_visualize_path():
    # Step 1: Find the path in grid indices (2D)
    path_indices_2d = find_path_2d(start_point, exit_point, maze_grid_2d, depth_axis)
    print("Path found with", len(path_indices_2d), "points.")
    print("Path Indices (2D):", path_indices_2d)

    # Step 2: Convert path 2D grid indices back to 3D real-world coordinates
    path_coords = []
    x_min, y_min, z_min = point_coords[:, 0].min(), point_coords[:, 1].min(), point_coords[:, 2].min()
    max_depth = point_coords[:, depth_axis].max()  # Get the maximum value along the depth axis
    max_depth_idx = int((max_depth - [x_min, y_min, z_min][depth_axis]) / grid_resolution)

    for idx_2d in path_indices_2d:
        idx_3d = [0, 0, 0]  # Initialize a 3D index
        idx_3d[depth_axis] = max_depth_idx  # Set the depth axis to the max depth index

        # Assign the 2D indices to the other two axes
        axes_2d = [i for i in range(3) if i != depth_axis]
        idx_3d[axes_2d[0]], idx_3d[axes_2d[1]] = idx_2d

        real_coords = [
            x_min + idx_3d[0] * grid_resolution,
            y_min + idx_3d[1] * grid_resolution,
            z_min + idx_3d[2] * grid_resolution,
        ]
        path_coords.append(real_coords)
    
    # Step 3: Convert path_coords to numpy array for visualization
    path_coords = np.array(path_coords)

    # Step 4: Visualize walls and path
    # Walls visualization (point cloud)
    walls_pcd = o3d.geometry.PointCloud()
    walls_pcd.points = o3d.utility.Vector3dVector(wall_points)

    # Path visualization (point cloud)
    path_pcd = o3d.geometry.PointCloud()
    path_pcd.points = o3d.utility.Vector3dVector(path_coords)

    # Color the path black (RGB: [0, 0, 0])
    path_pcd.colors = o3d.utility.Vector3dVector(np.zeros_like(path_coords))  # Set all colors to black

    # Visualize the path and walls
    o3d.visualization.draw_geometries(
        [walls_pcd, path_pcd],
        window_name="Path Visualization in Open Spaces",
        width=800,
        height=600
    )

btn_path = Button(window, text="Find Path", command=find_and_visualize_path)
btn_path.pack(pady=20)


def visualize_selected_points():
    # Convert the selected start and exit points from 2D grid indices to 3D real-world coordinates
    x_min, y_min, z_min = point_coords[:, 0].min(), point_coords[:, 1].min(), point_coords[:, 2].min()
    max_depth = point_coords[:, depth_axis].max()  # Get the maximum value along the depth axis
    max_depth_idx = int((max_depth - [x_min, y_min, z_min][depth_axis]) / grid_resolution)

    selected_points = []
    for point in [start_point, exit_point]:
        idx_3d = [0, 0, 0]  # Initialize a 3D index
        idx_3d[depth_axis] = max_depth_idx  # Set the depth axis to the max depth index

        # Assign the 2D indices to the other two axes
        axes_2d = [i for i in range(3) if i != depth_axis]
        idx_3d[axes_2d[0]], idx_3d[axes_2d[1]] = point

        real_coords = [
            x_min + idx_3d[0] * grid_resolution,
            y_min + idx_3d[1] * grid_resolution,
            z_min + idx_3d[2] * grid_resolution,
        ]
        selected_points.append(real_coords)
    
    # Convert the selected points to a numpy array
    selected_points = np.array(selected_points)

    # Visualize the walls and selected points
    # Walls visualization (point cloud)
    walls_pcd = o3d.geometry.PointCloud()
    walls_pcd.points = o3d.utility.Vector3dVector(wall_points)

    # Selected points visualization (point cloud)
    selected_pcd = o3d.geometry.PointCloud()
    selected_pcd.points = o3d.utility.Vector3dVector(selected_points)

    # Color the selected points red (RGB: [1, 0, 0])
    selected_pcd.colors = o3d.utility.Vector3dVector(np.array([[1, 0, 0]] * len(selected_points)))

    # Visualize the walls and selected points
    o3d.visualization.draw_geometries(
        [walls_pcd, selected_pcd],
        window_name="Selected Points on Walls",
        width=800,
        height=600
    )

# Add the button to the UI
btn_visualize_points = Button(window, text="Visualize Selected Points", command=visualize_selected_points)
btn_visualize_points.pack(pady=20)


# Run the tkinter main loop
window.mainloop()

#end of the code for now