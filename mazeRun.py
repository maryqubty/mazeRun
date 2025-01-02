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
#First let's get the start and exit point:
# Global variables to store start and end points
start_point = None
end_point = None

selected_points = []

# Callback function to select points
def pick_points(vis):
    picked_indices = vis.get_picked_points()
    if picked_indices:
        point_coords = np.asarray(mazePcd.points)
        for idx in picked_indices:
            selected_points.append(point_coords[idx])
        print(f"Selected points: {selected_points}")
    return False

# Function to visualize and select points
def select_points():
    global mazePcd, selected_points
    selected_points = []  # Reset selected points
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Select Points")
    vis.add_geometry(mazePcd)
    vis.run()  # Open the visualization window
    vis.destroy_window()
    if len(selected_points) >= 2:
        print(f"Start point: {selected_points[0]}")
        print(f"Exit point: {selected_points[1]}")
    else:
        print("Please select at least two points.")

# Add buttons to the window
btn_select_points = Button(window, text="Select Start and Exit Points", command=select_points)
btn_select_points.pack(pady=20)



'''
# Step 3: Convert coordinates to grid indices
start_coord, exit_coord = picked_coords[0], picked_coords[1]
start_point = (
    int((start_coord[0] - x_min) / grid_resolution),
    int((start_coord[1] - y_min) / grid_resolution),
    int((start_coord[2] - z_min) / grid_resolution)
)
exit_point = (
    int((exit_coord[0] - x_min) / grid_resolution),
    int((exit_coord[1] - y_min) / grid_resolution),
    int((exit_coord[2] - z_min) / grid_resolution)
)

print("Start Point:", start_point)
print("Exit Point:", exit_point)

'''
# Run the tkinter main loop
window.mainloop()

#end of the code for now