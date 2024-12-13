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

# Run the tkinter main loop
window.mainloop()


#end of the code for now